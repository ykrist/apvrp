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
use crate::utils::{HashMapExt, IoContext};

use super::*;
use std::path::Path;
use std::io::BufWriter;
use std::fs::File;
use daggylp::Weight;


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

    let mut add_bounds = |pvt: PvTask, t: &Task, model: &mut Model| -> Result<()> {
      if lb.contains_key(&pvt) {
        return Ok(())
      }
      // Constraints (3e)
      trace!(
        p_lb=pvt.t_release,
        p_ub=(pvt.t_deadline - pvt.tt),
        lb=t.t_release,
        ub=(t.t_deadline - t.tt),
        ?t, ?pvt, "bounds"
      );
      lb.insert(pvt, model.add_constr("", c!(vars[t] >= pvt.t_release))?);
      ub.insert(pvt, model.add_constr("", c!(vars[t] <= pvt.t_deadline - pvt.tt))?);
      Ok(())
    };

    for (_, pv_route) in &sol.pv_routes {
      let mut pv_route = pv_route.iter();
      let mut pt1 = *pv_route.next().unwrap();
      let mut t1 = lu.tasks.pvtask_to_task[&pt1];
      add_bounds(pt1, &t1, model)?;


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
        add_bounds(pt2, &t2, model)?;
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
          if v.contains_key(&t) {
            debug_assert_eq!(pv_route.first(), pv_route.last());
            continue;
          };
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
      .filter_map(|(_, route)| {
        if route.last().unwrap().is_depot() {
          let second_last_task = &route[route.len() - 2];
          second_last_tasks.push(*second_last_task);
          trace!(?second_last_task);
          Some(vars[second_last_task])
        } else {
          None
        }
      })
      .grb_sum();

    model.update()?;
    trace!(obj=?obj.with_names(&model));
    model.set_objective(obj, Minimize)?;

    Ok(TimingSubproblem { vars, cons, model, mp_sol: sol, second_last_tasks, lu })
  }

  pub fn write_debug_graph(&self, path: impl AsRef<Path>) -> Result<()> {
    use std::io::Write;

    let mut writer = BufWriter::new(File::create(&path).write_context(&path)?);
    let mut task_order = map_with_capacity(self.cons.lb.len());

    for (k,&pt) in self.cons.lb.keys().enumerate() {
      let t = self.lu.tasks.pvtask_to_task[&pt];
      let obj = if self.second_last_tasks.contains(&t) { 1 } else { 0 };
      task_order.insert(t, k);
      writeln!(writer, "{} {} {}", pt.t_release, pt.t_deadline, obj)?;
    }
    writeln!(writer, "edges")?;

    for ((t1, t2), c) in &self.cons.edge_constraints {
      let rhs : f64 = self.model.get_obj_attr(attr::RHS, c)?;
      let d = rhs.abs().round() as Time;
      let i = task_order[t1];
      let j = task_order[t2];
      writeln!(writer, "{} {} {}", i, j, d)?;
    }

    Ok(())
  }

  #[instrument(level="error", name="debug_iis", skip(self))]
  pub fn debug_infeasibility(&mut self) -> Result<()> {
    self.model.compute_iis()?;
    for (t, lb) in &self.cons.lb {
      if self.model.get_obj_attr(attr::IISConstr, lb)? > 0 {
        error!(cons="lb", ?t, val=?self.model.get_obj_attr(attr::RHS, lb)?);
      }
    }
    for (t, ub) in &self.cons.ub {
      if self.model.get_obj_attr(attr::IISConstr, ub)? > 0 {
        error!(cons="ub", ?t, val=?self.model.get_obj_attr(attr::RHS, ub)?);
      }
    }
    for ((t1, t2), c) in &self.cons.edge_constraints {
      if self.model.get_obj_attr(attr::IISConstr, c)? > 0 {
        error!(cons="edge", ?t1, ?t2, val=?-self.model.get_obj_attr(attr::RHS, c)?);
      }
    }
    Ok(())
  }
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

      cons.retain_ok::<_, anyhow::Error>(|(t1, t2), c| {
        if model.get_obj_attr(attr::IISConstr, c)? > 0 {
          trace!(?t1, ?t2);
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

    let mut num_edges_constraints = 0i32;
    for ((t1, t2), c) in &self.cons.edge_constraints {
      let dual = self.model.get_obj_attr(attr::Pi, c)?;
      if dual.abs() > 0.1 {
        num_edges_constraints += 1;
        trace!(?t1, ?t2, ?dual, "non-zero edge dual");
        cb.mp_vars.max_weight_edge_sum(self.lu, *t1, *t2).sum_into(&mut lhs);
      }
    }

    self.second_last_tasks.iter()
      .flat_map(|&t| cb.mp_vars.y_sum_av(cb.lu, t, cb.lu.tasks.ddepot))
      .sum_into(&mut lhs);

    let mut num_lbs = 0i32;
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
      for (_, c) in &self.cons.ub {
        let dual = self.model.get_obj_attr(attr::Pi, c)?;
        assert!(dual.abs() < 0.1, "non-zero dual on UB");
      }
    }

    let num_obj_tasks = self.second_last_tasks.len() as i32;
    trace!(num_lbs, num_edges_constraints, num_obj_tasks);
    lhs = sp_obj * (1 - num_lbs - num_edges_constraints - num_obj_tasks + lhs);

    let rhs = self.lu.sets.avs()
      .cartesian_product(&self.second_last_tasks)
      .filter_map(|(a, &t)| cb.mp_vars.theta.get(&(a,t)))
      .grb_sum();

    cb.enqueue_cut(c!(lhs <= rhs), CutType::LpOpt);
    Ok(())
  }
}