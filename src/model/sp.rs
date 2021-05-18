use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use super::mp::MpVars;
use super::cb::{CbError};
use tracing::{error_span, debug, trace, error};
use crate::TaskType::ODepot;
use itertools::Itertools;
use crate::solution::*;
use crate::model::cb::{CutType, Cb};
use std::collections::HashSet;

pub struct SpConstraints {
  av_sync: Map<(Task, Task), Constr>,
  start_req: Map<Task, Constr>,
  end_req: Map<Task, Constr>,
  lb: Map<Task, Constr>,
  ub: Map<Task, Constr>,
}

impl SpConstraints {
  pub fn build(data: &Data, tasks: &Tasks, av_routes: &Vec<(Avg, AvPath)>, pv_routes: &Vec<(Pv, PvPath)>, model: &mut Model, vars: &Map<Task, Var>) -> Result<Self> {
    let _span = error_span!("sp_cons").entered();

    // Constraints (3b)
    let mut av_sync = map_with_capacity(vars.len()); // slightly too big but close enough
    for (_, route) in av_routes {
      for (&t1, &t2) in route.iter().tuple_windows() {
        if t1 != tasks.odepot && t2 != tasks.ddepot {
          trace!(?t1, ?t2, "av_sync");
          let c = c!(vars[&t1] + t1.tt + data.travel_time[&(t1.end, t2.start)] <= vars[&t2]);
          av_sync.insert((t1, t2), model.add_constr("", c)?);
        }
      }
    }


    let mut start_req = Map::default();
    let mut end_req = Map::default();
    let mut lb = map_with_capacity(vars.len());
    let mut ub = map_with_capacity(vars.len());

    let mut add_bounds = |t: Task, model: &mut Model| -> Result<()> {
      // Constraints (3e)
      lb.insert(t, model.add_constr("", c!(vars[&t] >= t.t_release))?);
      ub.insert(t, model.add_constr("", c!(vars[&t] <= t.t_deadline - t.tt))?);
      Ok(())
    };

    for (_, pv_route) in pv_routes {
      let mut pv_route = pv_route.iter();
      let mut t1 = *pv_route.next().unwrap();
      add_bounds(t1, model)?;

      for &t2 in pv_route {
        debug_assert_ne!((t1.ty, t2.ty), (TaskType::Request, TaskType::Request));

        if t2.ty == TaskType::Request {
          // Constraints (3c)
          let c = c!(vars[&t1] + t1.tt + data.srv_time[&t2.start] <= vars[&t2]);
          start_req.insert(t1, model.add_constr("", c)?);
        } else if t1.ty == TaskType::Request {
          // Constraints (3d)
          let c = c!(vars[&t1] + t1.tt + data.srv_time[&t1.end] <= vars[&t2]);
          end_req.insert(t2, model.add_constr("", c)?);
        }
        add_bounds(t2, model)?;
        t1 = t2;
      }
    }
    Ok(SpConstraints { av_sync, start_req, end_req, ub, lb })
  }
}
//
// #[derive(Clone)]
// pub enum BendersCut {
//   Optimality(OptCut),
//   Feasibility(FeasCut),
// }
//
// impl BendersCut {
//   pub fn into_ineq(self, sets: &Sets, tasks: &Tasks, vars: &MpVars) -> IneqExpr {
//     match self {
//       BendersCut::Optimality(cut) => cut.into_ineq(sets, vars),
//       BendersCut::Feasibility(cut) => cut.into_ineq(sets, vars),
//     }
//   }
// }

#[derive(Clone, Debug)]
pub struct OptCut {
  /// Subproblem objective when this cut is active.
  pub sp_obj: f64,
  /// Task-to-task pairs which must ALL be active for the cut to be active
  pub task_pairs: Vec<(Task, Task)>,
  /// Tasks which appear in the subproblem objective.
  pub obj_tasks: Vec<Task>,
}

impl OptCut {
  pub fn build(sp: &TimingSubproblem) -> Result<Self> {
    let _span = error_span!("opt_cut").entered();

    let sp_obj = sp.model.get_attr(attr::ObjVal)?;
    let mut task_pairs = Vec::new();
    for (&(t1, t2), c) in &sp.cons.av_sync {
      trace!(?t1, ?t2, dual=?sp.model.get_obj_attr(attr::Pi, c));
      // if t2.ty == TaskType::DDepot
      //   || model.get_obj_attr(attr::Pi, c)?.abs() > 1e-8
      // {
      task_pairs.push((t1, t2));
      // }
    }

    let mut obj_tasks = Vec::with_capacity(sp.vars.len());
    for (t, var) in &sp.vars {
      if sp.model.get_obj_attr(attr::Obj, var)? > 0.001 {
        obj_tasks.push(*t);
      }
    }
    trace!(task_pairs=?&task_pairs);

    Ok(OptCut { sp_obj, task_pairs, obj_tasks })
  }

  pub fn into_ineq(self, sets: &Sets, vars: &MpVars) -> IneqExpr {
    let _span = error_span!("opt_cut").entered();

    let lhs = sets.avs()
      .cartesian_product(self.obj_tasks)
      .filter_map(|(av, t)| vars.theta.get(&(av, t)))
      .grb_sum();

    let mut ysum = Expr::default();
    let n_pairs = self.task_pairs.len() as isize;

    for (t1, t2) in self.task_pairs {
      trace!(?t1, ?t2);
      for av in sets.avs() {
        if let Some(&y) = vars.y.get(&(av, t1, t2)) {
          ysum = ysum + y;
        }
      }
    }

    c!(lhs >= self.sp_obj*(1 - n_pairs + ysum))
  }
}

// #[derive(Clone)]
// pub struct FeasCut {
//   pub iis_constr: Vec<Constr>,
//   pub task_pairs: Vec<(Task, Task)>,
//   pub tasks: HashSet<Task>,
// }
//
// impl FeasCut {
//   pub fn build(model: &Model, cons: &SpConstraints) -> Result<Self> {
//     let mut tasks = HashSet::new();
//     let mut task_pairs = Vec::new();
//
//     let x_cons = cons.end_req.iter()
//       .chain(&cons.start_req)
//       .chain(&cons.lb)
//       .chain(&cons.ub);
//
//     for (&t, c) in x_cons {
//       if model.get_obj_attr(attr::IISConstr, c)? > 0 {
//         tasks.push(t);
//       }
//     }
//
//     for (&pair, c) in &cons.av_sync {
//       if model.get_obj_attr(attr::IISConstr, c)? > 0 {
//         task_pairs.push(pair);
//       }
//     }
//     Ok(FeasCut { tasks, task_pairs })
//   }
//
//   pub fn into_ineq(self, sets: &Sets, vars: &MpVars) -> IneqExpr {
//     let n_pairs = self.task_pairs.len();
//     let n_x = self.tasks.len();
//     let xsum = self.tasks.iter()
//       .map(|t| vars.x[t])
//       .grb_sum();
//     let ysum = self.task_pairs.into_iter()
//       .flat_map(|(t1, t2)|
//         sets.avs().filter_map(move |av| vars.y.get(&(av, t1, t2))))
//       .grb_sum();
//
//     c!(xsum + ysum <= n_pairs + n_x - 1)
//   }
// }


pub struct TimingSubproblem<'a> {
  pub vars: Map<Task, Var>,
  pub cons: SpConstraints,
  pub model: Model,
  pub av_routes: &'a Vec<(Avg, AvPath)>,
  pub pv_routes: &'a Vec<(Pv, PvPath)>,
  pub data: &'a Data,
}


impl<'a> TimingSubproblem<'a> {
  pub fn build(env: &Env, data: &'a Data, tasks: &Tasks, av_routes: &'a Vec<(Avg, AvPath)>, pv_routes: &'a Vec<(Pv, PvPath)>) -> Result<TimingSubproblem<'a>> {
    let _span = error_span!("sp_build").entered();

    let mut model = Model::with_env("subproblem", env)?;
    let vars: Map<_, _> = {
      let mut v = map_with_capacity(pv_routes.iter().map(|(_, tasks)| tasks.len()).sum());
      for (_, tasks) in pv_routes {
        for &t in tasks {
          v.insert(t, add_ctsvar!(model, name: &format!("T[{:?}]", &t))?);
        }
      }
      v
    };

    let cons = SpConstraints::build(data, tasks, av_routes, pv_routes, &mut model, &vars)?;

    let mut obj_constant = 0.0;
    let obj = av_routes.iter()
      .map(|(_, route)| {
        let second_last_task = &route[route.len() - 2];
        obj_constant += (second_last_task.tt + data.travel_time[&(second_last_task.end, Loc::Ad)]) as f64;
        trace!(?second_last_task);
        vars[second_last_task]
      })
      .grb_sum() + obj_constant;
    model.update()?;
    trace!(?obj_constant, obj=?obj.with_names(&model));
    model.set_objective(obj, Minimize)?;

    Ok(TimingSubproblem { vars, cons, model, av_routes, pv_routes, data })
  }

  #[tracing::instrument(level = "error", skip(self, cb))]
  pub fn add_cuts(mut self, cb: &mut super::cb::Cb, estimate: Time) -> Result<()> {
    let mut round = 0;
    loop {
      self.model.optimize()?;
      let status = self.model.status();
      debug!(?status, round);
      match status? {
        Status::Infeasible => {
          self.model.compute_iis()?;
          cb.enqueue_cut(self.build_iis_cut(cb)?, CutType::LpFeas);
        }

        Status::Optimal => {
          let obj = self.model.get_attr(attr::ObjVal)?.round() as Time;
          debug!(obj);
          trace!(T=?get_var_values(&self.model, &self.vars)?.collect_vec());
          if estimate > obj {
            SpSolution::from_sp(&self)?.pretty_print(self.data);
            error!(obj, "invalid Benders cut");
            return Err(CbError::Assertion.into());
          }

          if estimate < obj {
            // let cut = OptCut::build(&self)?;
            // cb.enqueue_cut(cut.into_ineq(cb.sets, &cb.mp_vars), CutType::LpOpt);
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
      round += 1;
    }
  }

  fn build_iis_cut(&mut self, cb: &Cb) -> Result<IneqExpr> {
    let mut tasks = HashSet::new();

    let x_cons = self.cons.end_req.iter()
      .chain(&self.cons.start_req)
      .chain(&self.cons.lb)
      .chain(&self.cons.ub);

    for (&t, &c) in x_cons {
      if self.model.get_obj_attr(attr::IISConstr, &c)? > 0 {
        self.model.remove(c)?;
        tasks.insert(t);
      }
    }

    let mut lhs = tasks.iter().map(|t| cb.mp_vars.x[t]).grb_sum();
    let mut n = tasks.len();

    for (&(t1, t2), &c) in &self.cons.av_sync {
      if self.model.get_obj_attr(attr::IISConstr, &c)? > 0 {
        self.model.remove(c)?;
        for a in cb.sets.avs() {
          if let Some(&y) = cb.mp_vars.y.get(&(a, t1, t2)) {
            lhs = lhs + y;
            n += 1;
          }
        }
      }
    }

    Ok(c!( lhs <= n - 1 ))
  }

  #[tracing::instrument(level = "trace", skip(self, cb))]
  fn build_mrs_cut(&self, cb: &Cb) -> Result<IneqExpr> {
    let sp_obj = self.model.get_attr(attr::ObjVal)?;

    let mut ysum = Expr::default();
    let mut n_pairs = 0i32;

    for (&(t1, t2), c) in &self.cons.av_sync {
      trace!(?t1, ?t2, dual=?self.model.get_obj_attr(attr::Pi, c));
      if self.model.get_obj_attr(attr::Pi, c)?.abs() > 1e-6 || t2.ty == TaskType::DDepot {
        for a in cb.sets.avs() {
          if let Some(&y) = cb.mp_vars.y.get(&(a, t1, t2)) {
            ysum = ysum + y;
          }
        }
        n_pairs += 1;
      }
    }
    assert!(n_pairs > 0);
    let mut theta_sum = Expr::default();
    for (&t, var) in &self.vars {
      if self.model.get_obj_attr(attr::Obj, var)? > 0.001 {
        for a in cb.sets.avs() {
          if let Some(&theta) = cb.mp_vars.theta.get(&(a, t)) {
            theta_sum = theta_sum + theta;
          }
        }
      }
    }

    Ok(c!(sp_obj*(1 - n_pairs + ysum) <= theta_sum))
  }
}

