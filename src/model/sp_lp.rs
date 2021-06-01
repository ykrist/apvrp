use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use super::mp::MpVars;
use super::cb::{CbError};
use tracing::*;
use crate::TaskType::ODepot;
use itertools::Itertools;
use crate::solution::*;
use crate::model::cb::{CutType, Cb};
use std::collections::{HashSet, HashMap};
use std::hash::{Hash, BuildHasher};
use crate::utils::HashMapExt;
use smallvec::SmallVec;
use super::*;

pub struct TimingConstraints {
  pub av_sync: Map<(Task, Task), Constr>,
  pub loading: Map<Task, Constr>,
  pub unloading: Map<Task, Constr>,
  pub lb: Map<Task, Constr>,
  pub ub: Map<Task, Constr>,
}

impl TimingConstraints {
  pub fn build(data: &Data, tasks: &Tasks, sol: &Solution, model: &mut Model, vars: &Map<Task, Var>) -> Result<Self> {
    let _span = error_span!("sp_cons").entered();




    let mut loading = Map::default();
    let mut unloading = Map::default();
    let mut lb = map_with_capacity(vars.len());
    let mut ub = map_with_capacity(vars.len());

    let mut add_bounds = |t: Task, model: &mut Model| -> Result<()> {
      // Constraints (3e)
      lb.insert(t, model.add_constr("", c!(vars[&t] >= t.t_release))?);
      ub.insert(t, model.add_constr("", c!(vars[&t] <= t.t_deadline - t.tt))?);
      Ok(())
    };

    let mut dominated_av_sync_constraints = Set::default();

    for (_, pv_route) in &sol.pv_routes {
      let mut pv_route = pv_route.iter();
      let mut t1 = *pv_route.next().unwrap();
      add_bounds(t1, model)?;

      for &t2 in pv_route {
        debug_assert_ne!((t1.ty, t2.ty), (TaskType::Request, TaskType::Request));

        if t2.ty == TaskType::Request {
          // Constraints (3c) (loading time)
          let c = c!(vars[&t1] + t1.tt + data.srv_time[&t1.end] <= vars[&t2]);
          loading.insert(t1, model.add_constr("", c)?);
          dominated_av_sync_constraints.insert((t1, t2));
        } else if t1.ty == TaskType::Request {
          // Constraints (3d) (unloading time)
          let c = c!(vars[&t1] + t1.tt + data.srv_time[&t1.end] <= vars[&t2]);
          unloading.insert(t2, model.add_constr("", c)?);
          dominated_av_sync_constraints.insert((t1, t2));
        }
        add_bounds(t2, model)?;
        t1 = t2;
      }
    }

    // Constraints (3b)
    let mut av_sync = map_with_capacity(vars.len()); // slightly too big but close enough
    for (_, route) in &sol.av_routes {
      for (&t1, &t2) in route.iter().tuple_windows() {
        if t1 != tasks.odepot && t2 != tasks.ddepot && !dominated_av_sync_constraints.contains(&(t1, t2)) {
          trace!(?t1, ?t2, "av_sync");
          let c = c!(vars[&t1] + t1.tt + data.travel_time[&(t1.end, t2.start)] <= vars[&t2]);
          av_sync.insert((t1, t2), model.add_constr("", c)?);
        }
      }
    }
    Ok(TimingConstraints { av_sync, loading, unloading, ub, lb })
  }
}


pub struct TimingSubproblem<'a> {
  pub vars: Map<Task, Var>,
  pub second_last_tasks: SmallVec<[Task; NUM_AV_UB]>,
  pub cons: TimingConstraints,
  pub model: Model,
  pub mp_sol: &'a Solution,
  pub data: &'a Data,
  pub tasks: &'a Tasks,
}


impl<'a> TimingSubproblem<'a> {
  pub fn build(env: &Env, data: &'a Data, tasks: &'a Tasks, sol: &'a Solution) -> Result<TimingSubproblem<'a>> {
    let _span = error_span!("sp_build").entered();

    let mut model = Model::with_env("subproblem", env)?;
    let vars: Map<_, _> = {
      let mut v = map_with_capacity(sol.pv_routes.iter().map(|(_, tasks)| tasks.len()).sum());
      for (_, tasks) in &sol.pv_routes {
        for &t in tasks {
          #[cfg(debug_assertions)]
            v.insert(t, add_ctsvar!(model, name: &format!("T[{:?}]", &t))?);
          #[cfg(not(debug_assertions))]
            v.insert(t, add_ctsvar!(model)?);
        }
      }
      v
    };

    let cons = TimingConstraints::build(data, tasks, sol, &mut model, &vars)?;

    let mut obj_constant = 0.0;
    let mut second_last_tasks = SmallVec::new();

    let obj = sol.av_routes.iter()
      .map(|(_, route)| {
        let second_last_task = &route[route.len() - 2];
        second_last_tasks.push(*second_last_task);
        obj_constant += (second_last_task.tt + data.travel_time[&(second_last_task.end, Loc::Ad)]) as f64;
        trace!(?second_last_task);
        vars[second_last_task]
      })
      .grb_sum() + obj_constant;
    model.update()?;
    trace!(?obj_constant, obj=?obj.with_names(&model));
    model.set_objective(obj, Minimize)?;

    Ok(TimingSubproblem { vars, cons, model, mp_sol: sol, data, second_last_tasks, tasks })
  }

  pub fn add_cuts(mut self, cb: &mut super::cb::Cb, estimate: Time) -> Result<()> {
    let _s = error_span!("add_cuts", estimate).entered();

    let mut iter = 0;
    loop {
      self.model.optimize()?;
      let status = self.model.status();
      let _s = debug_span!("lp_solve_loop", ?iter).entered();
      debug!(?status);
      match status? {
        Status::Infeasible => {
          self.model.compute_iis()?;
          cb.enqueue_cut(self.build_iis_cut(cb)?, CutType::LpFeas);
        }

        Status::Optimal => {
          if iter > 0 {
            return Ok(());
          }
          let obj = self.model.get_attr(attr::ObjVal)?.round() as Time;
          // trace!(obj); // FIXME this assertion is not quite right - need to manually minimise theta
          // #[cfg(debug_assertions)] {
          //   if estimate > obj {
          //     error!(obj,T=?get_var_values(&self.model, &self.vars)?.collect_vec(), "invalid Benders cut(s)");
          //     let sol = SpSolution::from_sp(&self)?;
          //     sol.pretty_print(&self.data);
          //     for (ty, i, cut) in &cb.cut_cache {
          //       if ty.is_opt_cut() {
          //         let (lhs, rhs) = cut.evaluate(&cb.var_vals);
          //         warn!(?ty, idx=i, ?obj, ?lhs, ?rhs, cut=?cut.with_names(&cb.var_names),  "possible invalid opt cut");
          //       }
          //     }
          //     let mut solution = self.mp_sol.clone();
          //     solution.objective = Some(obj);
          //     let err = CbError::InvalidBendersCut { estimate, obj, solution };
          //     cb.error = Some(err.clone());
          //     return Err(err.into());
          //   }
          // }

          if estimate < obj {
            cb.enqueue_cut(self.build_mrs_cut(cb)?, CutType::LpOpt);
          }

          return Ok(());
        }

        other => {
          let err = CbError::Status(other);
          error!(status=?other, "{}", err);
          return Err(err.into());
        }
      }
      iter += 1;
    }
  }

  #[tracing::instrument(level = "trace", skip(self, cb))]
  fn build_iis_cut(&mut self, cb: &Cb) -> Result<IneqExpr> {
    let mut tasks = HashSet::new();

    let model = &mut self.model;
    let mut retain_non_iis_constr = |t: &Task, c: &mut Constr| -> Result<bool> {
      if model.get_obj_attr(attr::IISConstr, &c)? > 0 {
        trace!(?t, ?c, c_group="x_cons", "remove SP constr");
        tasks.insert(*t);
        model.remove(*c)?;
        Ok(false)
      } else {
        Ok(true)
      }
    };

    self.cons.unloading.retain_ok(&mut retain_non_iis_constr)?;
    self.cons.loading.retain_ok(&mut retain_non_iis_constr)?;
    self.cons.lb.retain_ok(&mut retain_non_iis_constr)?;
    self.cons.ub.retain_ok(&mut retain_non_iis_constr)?;

    let mut lhs = tasks.iter().map(|t| cb.mp_vars.x[t]).grb_sum();
    let mut n = tasks.len();

    let model = &mut self.model;
    self.cons.av_sync.retain_ok(|&(t1, t2), &mut c| -> Result<bool> {
      if model.get_obj_attr(attr::IISConstr, &c)? > 0 {
        trace!(?t1, ?t2, c_group="y_cons", "remove SP constr");
        model.remove(c)?;

        for a in cb.sets.avs() {
          if let Some(&y) = cb.mp_vars.y.get(&(a, t1, t2)) {
            lhs += y;
            n += 1;
          }
        }
        Ok(false) // remove this constraint
      } else {
        Ok(true) // keep this constraint
      }
    })?;

    Ok(c!( lhs <= n - 1 ))
  }


  #[tracing::instrument(level = "trace", skip(self, cb))]
  fn build_mrs_cut(&self, cb: &Cb) -> Result<IneqExpr> {
    let sp_obj = self.model.get_attr(attr::ObjVal)?;

    let mut ysum = Expr::default();
    let mut n_x_y_terms = 0i32;

    for (&(t1, t2), c) in &self.cons.av_sync {
      trace!(?t1, ?t2, dual=?self.model.get_obj_attr(attr::Pi, c), slack=?self.model.get_obj_attr(attr::Slack, c));
      if self.model.get_obj_attr(attr::Pi, c)?.abs() > 1e-6 {
        for a in cb.sets.avs() {
          if let Some(&y) = cb.mp_vars.y.get(&(a, t1, t2)) {
            ysum += y;
          }
        }
        n_x_y_terms += 1;
      }
    }

    #[cfg(debug_assertions)] {
      fn log_nonzero_dual_tasks<'a>(model: &grb::Model, cgroup: &str, cons: impl IntoIterator<Item=(&'a Task,&'a Constr)>) -> Result<()> {
        for (t, c) in cons {
          let dual =model.get_obj_attr(attr::Pi, c)?;
          if dual.abs() > 0.0001  {
            trace!(?t, ?dual, ?cgroup, "non-zero dual");
          }
        }
        Ok(())
      }

      log_nonzero_dual_tasks(&self.model, "start_req", &self.cons.loading)?;
      log_nonzero_dual_tasks(&self.model, "end_req", &self.cons.unloading)?;
      log_nonzero_dual_tasks(&self.model, "ub", &self.cons.ub)?;
      log_nonzero_dual_tasks(&self.model, "lb", &self.cons.lb)?;
    }

    fn add_tasks_with_nonzero_dual<'a>(model: &grb::Model, taskset: &mut Set<Task>, cons: impl IntoIterator<Item=(&'a Task,&'a Constr)>) -> Result<()> {
      for (t, c) in cons {
        if model.get_obj_attr(attr::Pi, c)?.abs() > 0.0001  {
           taskset.insert(*t);
        }
      }
      Ok(())
    }

    let mut critial_tasks = Set::default();
    add_tasks_with_nonzero_dual(&self.model, &mut critial_tasks, &self.cons.unloading)?;
    add_tasks_with_nonzero_dual(&self.model, &mut critial_tasks, &self.cons.loading)?;
    add_tasks_with_nonzero_dual(&self.model, &mut critial_tasks, &self.cons.ub)?;
    add_tasks_with_nonzero_dual(&self.model, &mut critial_tasks, &self.cons.lb)?;
    n_x_y_terms += critial_tasks.len() as i32;

    let xsum = critial_tasks.into_iter().map(|t| cb.mp_vars.x[&t]).grb_sum();

    let mut theta_sum = Expr::default();
    for &t in &self.second_last_tasks {
      for a in cb.sets.avs() {
        if let Some(&theta) = cb.mp_vars.theta.get(&(a, t)) {
          theta_sum += theta;
          ysum += cb.mp_vars.y[&(a, t, cb.tasks.ddepot)];
          n_x_y_terms += 1;
        }
      }
    }
    let sol = SpSolution::from_sp(self)?;
    let cut = c!(sp_obj*(1 - n_x_y_terms + ysum + xsum) <= theta_sum);
    trace!(n_x_y_terms, cut=?cut.with_names(&cb.var_names), ?sol);
    Ok(cut)
  }
}

impl<'a>  SpSolve for TimingSubproblem<'a> {
  type OptInfo = ();
  type InfInfo = ();

  fn solve(&mut self) -> Result<SpStatus<(), ()>> {
    self.model.optimize()?;
    match self.model.status()? {
      Status::Optimal => {
        let obj_val = self.model.get_attr(attr::ObjVal)?.round() as Time;
        Ok(SpStatus::Optimal(obj_val, ()))
      },
      Status::Infeasible => Ok(SpStatus::Infeasible(())),
      other => {
        let err = CbError::Status(other);
        error!(status=?other, "{}", err);
        Err(err.into())
      }
    }
  }

  fn extract_and_remove_iis(&mut self, _: ()) -> Result<SpConstraints> {
    fn pop_x_constraints(model: &mut grb::Model, cons: &mut Map<Task, Constr>, iis: &mut SpConstraints, f: impl Fn(Task) -> SpConstr) -> Result<()> {
      let retain = |t: &Task, c: &mut Constr| -> Result<bool> {
        if model.get_obj_attr(attr::IISConstr, &c)? > 0 {
          let spc = f(*t);
          iis.push(spc);
          trace!(?t, c=?spc, "remove SP constr");
          model.remove(*c)?;
          Ok(false)
        } else {
          Ok(true)
        }
      };
      cons.retain_ok(retain)
    }

    fn add_x_constraints(model: &mut grb::Model, cons: &mut Map<Task, Constr>, iis: &mut SpConstraints, f: impl Fn(Task) -> SpConstr) -> Result<()> {
      for (&t, c )in cons {
        if model.get_obj_attr(attr::IISConstr, c)? > 0 {
          iis.push( f(t));
        }
      }
      Ok(())
    }

    self.model.compute_iis()?;
    let mut iis = SpConstraints::new();

    pop_x_constraints(&mut self.model, &mut self.cons.unloading, &mut iis, SpConstr::Unloading)?;
    pop_x_constraints(&mut self.model, &mut self.cons.loading, &mut iis, SpConstr::Loading)?;
    add_x_constraints(&mut self.model, &mut self.cons.ub, &mut iis, SpConstr::Ub)?;
    add_x_constraints(&mut self.model, &mut self.cons.lb, &mut iis, SpConstr::Lb)?;

    let model = &mut self.model;

    self.cons.av_sync.retain_ok(|&(t1, t2), &mut c| -> Result<bool> {
      if model.get_obj_attr(attr::IISConstr, &c)? > 0 {
        let spc = SpConstr::AvTravelTime(t1, t2);
        iis.push(spc);
        trace!(?t1, ?t2, c=?spc, "remove SP constr");
        model.remove(c)?;
        Ok(false) // remove this constraint
      } else {
        Ok(true) // keep this constraint
      }
    })?;

    Ok(iis)
  }

  fn extract_mrs(&self, _: ()) -> Result<MrsInfo> {
    fn add_x_constraints(model: &grb::Model, cons: &Map<Task, Constr>, mrs: &mut SpConstraints, f: impl Fn(Task) -> SpConstr) -> Result<()> {
      for (t, c) in cons {
        let dual = model.get_obj_attr(attr::Pi, c)?;
        if dual.abs() > 0.1 {
          let spc = f(*t);
          mrs.push(spc);
          trace!(?t, c=?spc, ?dual, "non-zero dual");
        }
      }
      Ok(())
    }

    let mut mrs = SpConstraints::new();

    add_x_constraints(&self.model, &self.cons.unloading, &mut mrs, SpConstr::Unloading)?;
    add_x_constraints(&self.model, &self.cons.loading, &mut mrs, SpConstr::Loading)?;
    add_x_constraints(&self.model, &self.cons.ub, &mut mrs, SpConstr::Ub)?;
    add_x_constraints(&self.model, &self.cons.lb, &mut mrs, SpConstr::Lb)?;

    for (&(t1, t2), c) in &self.cons.av_sync {
      let dual = self.model.get_obj_attr(attr::Pi, c)?;
      if dual.abs() > 0.1 {
        let spc = SpConstr::AvTravelTime(t1, t2);
        mrs.push(spc);
        trace!(?t1, ?t2, c=?spc, ?dual, "non-zero dual");
      }
    }

    for &t in &self.second_last_tasks {
      mrs.push(SpConstr::AvTravelTime(t, self.tasks.ddepot));
    }

    Ok(smallvec::smallvec![mrs])
  }
}