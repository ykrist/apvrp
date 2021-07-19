use grb::prelude::*;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use tracing::*;
use itertools::Itertools;
use std::collections::{HashSet, HashMap};
use std::hash::{Hash, BuildHasher};
use smallvec::SmallVec;
use anyhow::Context;

use crate::*;
use crate::tasks::TaskId;
use crate::model::mp::MpVars;
use crate::solution::*;
use crate::model::cb::{CutType, Cb, CbError};
use crate::utils::HashMapExt;

use super::*;


thread_local! {
  static ENV: grb::Env = {
    let ctx_msg = "create SP environment";
    let mut e = Env::empty().context(ctx_msg).unwrap();
    e.set(param::OutputFlag, 0)
      .and_then(|e| e.set(param::Threads, 1))
      .context(ctx_msg).unwrap();
    e.start().context(ctx_msg).unwrap()
  };
}

pub struct TimingConstraints {
  pub edge_constraints: Map<(Task, Task), Constr>,
  pub lb: Map<PvTask, Constr>,
  pub ub: Map<PvTask, Constr>,
}

impl TimingConstraints {
  pub fn build(lu: &Lookups, sol: &Solution, model: &mut Model, vars: &Map<Task, Var>) -> Result<Self> {
    // TODO: we don't actually need the routes here, can just use var values
    let _span = error_span!("constraints").entered();

    let mut edge_constraints = map_with_capacity(2 * vars.len());
    let mut lb = map_with_capacity(vars.len());
    let mut ub = map_with_capacity(vars.len());

    let mut add_bounds = |pvt: PvTask, t: Task, model: &mut Model| -> Result<()> {
      // Constraints (3e)
      lb.insert(pvt, model.add_constr("", c!(vars[&t] >= pvt.t_release))?);
      ub.insert(pvt, model.add_constr("", c!(vars[&t] <= pvt.t_deadline - pvt.tt))?);
      Ok(())
    };

    for (_, pv_route) in &sol.pv_routes {
      let mut pv_route = pv_route.iter();
      let mut pt1 = *pv_route.next().unwrap();
      let mut t1 = lu.tasks.pvtask_to_task[&pt1];
      add_bounds(pt1, t1, model)?;

      for &pt2 in pv_route {
        let t2 = lu.tasks.pvtask_to_task[&pt2];
        debug_assert_ne!((pt1.ty, pt2.ty), (TaskType::Request, TaskType::Request));

        if pt2.ty == TaskType::Request {
          // Constraints (3c) (loading time)
          trace!(?pt1, ?pt2, "loading");
          let c = c!(vars[&t1] + t1.tt + lu.data.srv_time[&t1.end] <= vars[&t2]);
          edge_constraints.insert((t1, t2), model.add_constr("", c)?);
        } else if pt1.ty == TaskType::Request {
          // Constraints (3d) (unloading time)
          trace!(?pt1, ?pt2, "unloading");
          let c = c!(vars[&t1] + t1.tt + lu.data.srv_time[&t1.end] <= vars[&t2]);
          edge_constraints.insert((t1, t2), model.add_constr("", c)?);
        }
        add_bounds(pt2, t2, model)?;
        pt1 = pt2;
        t1 = t2;
      }
    }

    // Constraints (3b)
    for (_, route) in &sol.av_routes {
      for (&t1, &t2) in route.iter().tuple_windows() {
        if t1 != lu.tasks.odepot && t2 != lu.tasks.ddepot {
          if edge_constraints.contains_key(&(t1, t2)) {
            trace!(?t1, ?t2, "AV TT constraint dominated by loading/unloading constraint")
          } else {
            trace!(?t1, ?t2, "av_sync");
            let c = c!(vars[&t1] + t1.tt + lu.data.travel_time[&(t1.end, t2.start)] <= vars[&t2]);
            edge_constraints.insert((t1, t2), model.add_constr("", c)?);
          }
        }
      }
    }
    Ok(TimingConstraints { edge_constraints, ub, lb })
  }
}


pub struct TimingSubproblem<'a> {
  pub vars: Map<Task, Var>,
  // Tasks which appear in the objective
  pub second_last_tasks: SmallVec<[Task; NUM_AV_UB]>,
  pub cons: TimingConstraints,
  pub model: Model,
  pub mp_sol: &'a Solution,
  pub lu: &'a Lookups,
}

impl<'a> TimingSubproblem<'a> {
  pub fn build(lu: &'a Lookups, sol: &'a Solution) -> Result<TimingSubproblem<'a>> {
    let _span = error_span!("sp_build").entered();

    let mut model = ENV.with(|env| Model::with_env("subproblem", env))?;
    let vars: Map<_, _> = {
      let mut v = map_with_capacity(sol.pv_routes.iter().map(|(_, tasks)| tasks.len()).sum());
      for (_, pv_route) in &sol.pv_routes {
        for t in pv_route {
          let t = lu.tasks.pvtask_to_task[t];
          #[cfg(debug_assertions)]
            v.insert(t, add_ctsvar!(model, name: &format!("T[{:?}]", &t))?);
          #[cfg(not(debug_assertions))]
            v.insert(t, add_ctsvar!(model)?);
        }
      }
      v
    };

    let cons = TimingConstraints::build(lu, sol, &mut model, &vars)?;

    let mut second_last_tasks = SmallVec::new();

    let obj = sol.av_routes.iter()
      .map(|(_, route)| {
        let second_last_task = &route[route.len() - 2];
        second_last_tasks.push(*second_last_task);
        trace!(?second_last_task);
        vars[second_last_task]
      })
      .grb_sum();
    model.update()?;
    trace!(obj=?obj.with_names(&model));
    model.set_objective(obj, Minimize)?;

    Ok(TimingSubproblem { vars, cons, model, mp_sol: sol, second_last_tasks, lu })
  }

  // pub fn add_cuts(mut self, cb: &mut super::cb::Cb, estimate: Time) -> Result<()> {
  //   let _s = error_span!("add_cuts", estimate).entered();
  //
  //   let mut iter = 0;
  //   loop {
  //     self.model.optimize()?;
  //     let status = self.model.status();
  //     let _s = debug_span!("lp_solve_loop", ?iter).entered();
  //     debug!(?status);
  //     match status? {
  //       Status::Infeasible => {
  //         self.model.compute_iis()?;
  //         cb.enqueue_cut(self.build_iis_cut(cb)?, CutType::LpFeas);
  //       }
  //
  //       Status::Optimal => {
  //         if iter > 0 {
  //           return Ok(());
  //         }
  //         let obj = self.model.get_attr(attr::ObjVal)?.round() as Time;
  //         // trace!(obj); // FIXME this assertion is not quite right - need to manually minimise theta
  //         // #[cfg(debug_assertions)] {
  //         //   if estimate > obj {
  //         //     error!(obj,T=?get_var_values(&self.model, &self.vars)?.collect_vec(), "invalid Benders cut(s)");
  //         //     let sol = SpSolution::from_sp(&self)?;
  //         //     sol.pretty_print(&self.data);
  //         //     for (ty, i, cut) in &cb.cut_cache {
  //         //       if ty.is_opt_cut() {
  //         //         let (lhs, rhs) = cut.evaluate(&cb.var_vals);
  //         //         warn!(?ty, idx=i, ?obj, ?lhs, ?rhs, cut=?cut.with_names(&cb.var_names),  "possible invalid opt cut");
  //         //       }
  //         //     }
  //         //     let mut solution = self.mp_sol.clone();
  //         //     solution.objective = Some(obj);
  //         //     let err = CbError::InvalidBendersCut { estimate, obj, solution };
  //         //     cb.error = Some(err.clone());
  //         //     return Err(err.into());
  //         //   }
  //         // }
  //
  //         if estimate < obj {
  //           cb.enqueue_cut(self.build_mrs_cut(cb)?, CutType::LpOpt);
  //         }
  //
  //         return Ok(());
  //       }
  //
  //       other => {
  //         let err = CbError::Status(other);
  //         error!(status=?other, "{}", err);
  //         return Err(err.into());
  //       }
  //     }
  //     iter += 1;
  //   }
  // }

  // #[tracing::instrument(level = "trace", skip(self, cb))]
  // fn build_iis_cut(&mut self, cb: &Cb) -> Result<IneqExpr> {
  //   let mut tasks = HashSet::new();
  //
  //   let model = &mut self.model;
  //   let mut retain_non_iis_constr = |t: &PvTask, c: &mut Constr| -> Result<bool> {
  //     if model.get_obj_attr(attr::IISConstr, &c)? > 0 {
  //       trace!(?t, ?c, c_group="x_cons", "remove SP constr");
  //       tasks.insert(*t);
  //       model.remove(*c)?;
  //       Ok(false)
  //     } else {
  //       Ok(true)
  //     }
  //   };
  //
  //   self.cons.unloading.retain_ok(&mut retain_non_iis_constr)?;
  //   self.cons.loading.retain_ok(&mut retain_non_iis_constr)?;
  //   self.cons.lb.retain_ok(&mut retain_non_iis_constr)?;
  //   self.cons.ub.retain_ok(&mut retain_non_iis_constr)?;
  //
  //   let mut lhs = tasks.iter().map(|t| cb.mp_vars.x[t]).grb_sum();
  //   let mut n = tasks.len();
  //
  //   let model = &mut self.model;
  //   self.cons.av_sync.retain_ok(|&(t1, t2), &mut c| -> Result<bool> {
  //     if model.get_obj_attr(attr::IISConstr, &c)? > 0 {
  //       trace!(?t1, ?t2, c_group="y_cons", "remove SP constr");
  //       model.remove(c)?;
  //
  //       for a in cb.sets.avs() {
  //         if let Some(&y) = cb.mp_vars.y.get(&(a, t1, t2)) {
  //           lhs += y;
  //           n += 1;
  //         }
  //       }
  //       Ok(false) // remove this constraint
  //     } else {
  //       Ok(true) // keep this constraint
  //     }
  //   })?;
  //
  //   Ok(c!( lhs <= n - 1 ))
  // }
  //
  // #[tracing::instrument(level = "trace", skip(self, cb))]
  // fn build_mrs_cut(&self, cb: &Cb) -> Result<IneqExpr> {
  //   let sp_obj = self.model.get_attr(attr::ObjVal)?;
  //
  //   let mut ysum = Expr::default();
  //   let mut n_x_y_terms = 0i32;
  //
  //   for (&(t1, t2), c) in &self.cons.av_sync {
  //     trace!(?t1, ?t2, dual=?self.model.get_obj_attr(attr::Pi, c), slack=?self.model.get_obj_attr(attr::Slack, c));
  //     if self.model.get_obj_attr(attr::Pi, c)?.abs() > 1e-6 {
  //       for a in cb.sets.avs() {
  //         if let Some(&y) = cb.mp_vars.y.get(&(a, t1, t2)) {
  //           ysum += y;
  //         }
  //       }
  //       n_x_y_terms += 1;
  //     }
  //   }
  //
  //   #[cfg(debug_assertions)] {
  //     fn log_nonzero_dual_tasks<'a>(model: &grb::Model, cgroup: &str, cons: impl IntoIterator<Item=(&'a PvTask,&'a Constr)>) -> Result<()> {
  //       for (t, c) in cons {
  //         let dual =model.get_obj_attr(attr::Pi, c)?;
  //         if dual.abs() > 0.0001  {
  //           trace!(?t, ?dual, ?cgroup, "non-zero dual");
  //         }
  //       }
  //       Ok(())
  //     }
  //
  //     log_nonzero_dual_tasks(&self.model, "start_req", &self.cons.loading)?;
  //     log_nonzero_dual_tasks(&self.model, "end_req", &self.cons.unloading)?;
  //     log_nonzero_dual_tasks(&self.model, "ub", &self.cons.ub)?;
  //     log_nonzero_dual_tasks(&self.model, "lb", &self.cons.lb)?;
  //   }
  //
  //   fn add_tasks_with_nonzero_dual<'a>(model: &grb::Model, taskset: &mut Set<Task>, cons: impl IntoIterator<Item=(&'a Task,&'a Constr)>) -> Result<()> {
  //     for (t, c) in cons {
  //       if model.get_obj_attr(attr::Pi, c)?.abs() > 0.0001  {
  //          taskset.insert(*t);
  //       }
  //     }
  //     Ok(())
  //   }
  //
  //   let mut critial_tasks = Set::default();
  //   add_tasks_with_nonzero_dual(&self.model, &mut critial_tasks, &self.cons.unloading)?;
  //   add_tasks_with_nonzero_dual(&self.model, &mut critial_tasks, &self.cons.loading)?;
  //   add_tasks_with_nonzero_dual(&self.model, &mut critial_tasks, &self.cons.ub)?;
  //   add_tasks_with_nonzero_dual(&self.model, &mut critial_tasks, &self.cons.lb)?;
  //   n_x_y_terms += critial_tasks.len() as i32;
  //
  //   let xsum = critial_tasks.into_iter().map(|t| cb.mp_vars.x[&t]).grb_sum();
  //
  //   let mut theta_sum = Expr::default();
  //   for &t in &self.second_last_tasks {
  //     for a in cb.sets.avs() {
  //       if let Some(&theta) = cb.mp_vars.theta.get(&(a, t)) {
  //         theta_sum += theta;
  //         ysum += cb.mp_vars.y[&(a, t, cb.tasks.ddepot)];
  //         n_x_y_terms += 1;
  //       }
  //     }
  //   }
  //   let sol = SpSolution::from_sp(self)?;
  //   let cut = c!(sp_obj*(1 - n_x_y_terms + ysum + xsum) <= theta_sum);
  //   trace!(n_x_y_terms, cut=?cut.with_names(&cb.var_names), ?sol);
  //   Ok(cut)
  // }
}

impl<'a> Subproblem<'a> for TimingSubproblem<'a> {
  type Optimal = ();
  type Infeasible = ();
  type IisConstraintSets = std::iter::Once<Iis>;

  fn solve(&mut self) -> Result<SpStatus<(), ()>> {
    self.model.optimize()?;
    match self.model.status()? {
      Status::Optimal => {
        let obj_val = self.model.get_attr(attr::ObjVal)?.round() as Time;
        Ok(SpStatus::Optimal(obj_val, ()))
      }
      Status::Infeasible => Ok(SpStatus::Infeasible(())),
      other => {
        let err = CbError::Status(other);
        error!(status=?other, "{}", err);
        Err(err.into())
      }
    }
  }

  #[instrument(level="trace", skip(self))]
  fn extract_and_remove_iis(&mut self, _: ()) -> Result<Self::IisConstraintSets> {
    #[cfg(debug_assertions)]
    fn find_iis_bound<'a>(model: &Model, cons: impl IntoIterator<Item=(&'a PvTask, &'a Constr)>) -> Result<Option<PvTask>> {
      let mut task = None;
      for (t, c) in cons {
        if model.get_obj_attr(attr::IISConstr, c)? > 0 {
          assert_eq!(task, None);
          task = Some(*t);
        }
      }
      Ok(task)
    }

    #[cfg(not(debug_assertions))]
    fn find_iis_bound<'a>(model: &Model, cons: impl IntoIterator<Item=(&'a PvTask, &'a Constr)>) -> Result<Option<PvTask>> {
      for (t, c) in cons {
        if model.get_obj_attr(attr::IISConstr, c)? > 0 {
          return Ok(Some(*t));
        }
      }
      Ok(None)
    }

    self.model.compute_iis()?;

    let bounds = match find_iis_bound(&self.model, &self.cons.ub)? {
      Some(ub) => {
        let lb = find_iis_bound(&self.model, &self.cons.lb)?.expect("IIS has no lower bound");
        Some((lb, ub))
      }
      None => {
        debug_assert_eq!(find_iis_bound(&self.model, &self.cons.lb)?, None);
        None
      }
    };

    let mut iis_succ = Map::default();

    { // Split borrows
      let cons = &mut self.cons.edge_constraints;
      let model = &mut self.model;

      self.cons.edge_constraints.retain_ok::<_, anyhow::Error>(|(t1, t2), c| {
        if model.get_obj_attr(attr::IISConstr, c)? > 0 {
          trace!(?t1, ?t2); // FIXME remove constraint from lookup as well ya dos cunt
          iis_succ.insert(*t1, *t2);
          model.remove(*c)?;
          Ok(false)
        } else {
          Ok(true)
        }
      })?;
    }

    trace!(?iis_succ);

    let iis = if let Some((lb_task, ub_task)) = bounds {
      // Path IIS
      let mut path = SmallVec::with_capacity(iis_succ.len() + 1);

      let mut t = self.lu.tasks.pvtask_to_task[&lb_task];
      path.push(t);

      while iis_succ.len() > 0 {
        t = iis_succ.remove(&t).expect("bad IIS path");
        path.push(t);
      }
      debug_assert_eq!(path.last(), self.lu.tasks.pvtask_to_task.get(&ub_task));

      Iis::Path(PathIis { ub: ub_task, lb: lb_task, path } )
    } else {
      // Path IIS
      let mut cycle = SmallVec::with_capacity(iis_succ.len());

      let mut t = *iis_succ.keys().next().expect("IIS has no edge constraints");
      cycle.push(t);

      while iis_succ.len() > 1 { // keep last one
        t = iis_succ.remove(&t).expect("bad IIS cycle");
        cycle.push(t);
      }
      Iis::Cycle(cycle)
    };

    Ok(std::iter::once(iis))
  }

  fn add_optimality_cuts(&mut self, cb: &mut cb::Cb, _: ()) -> Result<()> {
    let sp_obj = self.model.get_attr(attr::ObjVal)?;
    let _s = trace_span!("optimality_cut", obj=%sp_obj).entered();
    let mut lhs = Expr::default();

    let mut num_edges_constraints = 0;
    for ((t1, t2), c) in &self.cons.edge_constraints {
      let dual = self.model.get_obj_attr(attr::Pi, c)?;
      if dual.abs() > 0.1 {
        num_edges_constraints += 1;
        trace!(?t1, ?t2, ?dual, "non-zero edge dual");
        for var in cb.mp_vars.max_weight_edge_sum(self.lu, *t1, *t2) {
          lhs += var;
        }
      }
    }

    let mut num_lbs = 0;
    for (t, c) in &self.cons.lb {
      let dual = self.model.get_obj_attr(attr::Pi, c)?;
      if dual.abs() > 0.1 {
        num_lbs += 1;
        trace!(?t, ?dual, "non-zero LB dual");
        for t in &self.lu.tasks.pvtask_to_similar_pvtask[t] {
          lhs += cb.mp_vars.x[t];
        }
      }
    }

    #[cfg(debug_assertions)] {
      for (t, c) in &self.cons.ub {
        let dual = self.model.get_obj_attr(attr::Pi, c)?;
        assert!(dual.abs() < 0.1, "non-zero dual on UB");
      }
    }

    trace!(%num_lbs, %num_edges_constraints);
    lhs = sp_obj * (1 - num_lbs - num_edges_constraints + lhs);

    let rhs = self.lu.sets.avs()
      .cartesian_product(&self.second_last_tasks)
      .filter_map(|(a, &t)| cb.mp_vars.theta.get(&(a,t)))
      .grb_sum();

    cb.enqueue_cut(c!(lhs <= rhs), CutType::LpOpt);
    Ok(())
  }
}