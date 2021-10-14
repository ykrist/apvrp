use grb::prelude::*;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use tracing::*;
use itertools::Itertools;
use std::collections::{HashSet, HashMap};
use std::hash::{Hash, BuildHasher};
use smallvec::SmallVec;
use anyhow::Context;
use std::path::Path;
use std::io::BufWriter;
use std::fs::File;
use daggylp::Weight;

use crate::*;
use crate::tasks::TaskId;
use crate::model::mp::MpVars;
use crate::solution::*;
use crate::model::cb::{CutType, Cb, CbError};
use crate::utils::{HashMapExt, IoContext};

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


pub struct TimingSubproblem<'a> {
  pub var_to_task: Map<Var, IdxTask>,
  pub task_to_var: Map<IdxTask, Var>,
  pub constraint_map: Map<SpConstr, Constr>,
  pub constraint_map_inv: Map<Constr, SpConstr>,
  pub obj_tasks: SmallVec<[IdxTask; NUM_AV_UB]>,
  pub model: Model,
  pub lu: &'a Lookups,
}

impl<'a> TimingSubproblem<'a> {
  pub fn from_mp_sol(lu: &'a Lookups, sol: &MpSolution) -> Result<Self> {
    let _span = error_span!("constraints").entered();

    let mut model = ENV.with(|env| Model::with_env("subproblem", env))?;
    let mut constraint_map = Map::default();
    let mut task_to_var = map_with_capacity(sol.num_nondepot_tasks());

    for &pvt in sol.iter_sp_vars() {
      let t = lu.tasks.pvtask_to_task[&pvt].index();
      let var = add_ctsvar!(model, name: &format!("T[{:?}]", t))?;
      task_to_var.insert(t, var);

      let lb = pvt.subproblem_lb();
      let ub = pvt.subproblem_ub();

      trace!(
        lb,
        ub,
        ?t, ?pvt, "bounds"
      );

      let grb_c = model.add_constr("", c!(task_to_var[&t] >= lb))?;
      constraint_map.insert(SpConstr::Lb(t, lb), grb_c);

      let grb_c = model.add_constr("", c!(task_to_var[&t] <= ub))?;
      constraint_map.insert(SpConstr::Ub(t, ub), grb_c);
    }

    for c in sol.iter_sp_x_edges() {
      let (pt1, pt2) = match c {
        XEdgeConstraint::Unloading(pt1, pt2) => {
          trace!(t1=?pt1, t2=?pt2, "unloading constraint");
          (pt1, pt2)
        }
        XEdgeConstraint::Loading(pt1, pt2) => {
          trace!(t1=?pt1, t2=?pt2, "loading constraint");
          (pt1, pt2)
        }
      };
      let d = pt1.tt + lu.data.srv_time[&pt1.end];
      let t1 = lu.tasks.pvtask_to_task[pt1].index();
      let t2 = lu.tasks.pvtask_to_task[pt2].index();


      let grb_c = model.add_constr("", c!(task_to_var[&t1] + d <= task_to_var[&t2]))?;
      constraint_map.insert(SpConstr::Delta(t1, t2, d), grb_c);
    }

    for (t1, t2) in sol.iter_sp_y_edges() {
      trace!(?t1, ?t2, "av_sync");
      let d = t1.tt + lu.data.travel_time[&(t1.end, t2.start)];
      let t1 = t1.index();
      let t2 = t2.index();
      let grb_c = model.add_constr("", c!(task_to_var[&t1] + d <= task_to_var[&t2]))?;
      constraint_map.insert(SpConstr::Delta(t1, t2, d), grb_c);
    }

    let obj_tasks = sol.sp_objective_tasks();
    model.set_objective(obj_tasks.iter().map(|t| task_to_var[t]).grb_sum(), Minimize)?;

    let var_to_task = utils::inverse_map(&task_to_var);
    let constraint_map_inv = utils::inverse_map(&constraint_map);
    Ok(TimingSubproblem {
      lu,
      model,
      task_to_var,
      var_to_task,
      constraint_map,
      constraint_map_inv,
      obj_tasks,
    })
  }


  pub fn build(lu: &'a Lookups, constraints: &Set<SpConstr>, obj_tasks: SmallVec<[IdxTask; NUM_AV_UB]>) -> Result<TimingSubproblem<'a>> {
    let _span = error_span!("sp_build").entered();

    let mut var_to_task = Map::default();

    let mut model = ENV.with(|env| Model::with_env("subproblem", env))?;

    for c in constraints {
      if let SpConstr::Lb(t, _) = c {
        #[cfg(debug_assertions)]
          let var = add_ctsvar!(model, name: &format!("T[{:?}]", t))?;
        #[cfg(not(debug_assertions))]
          let var = add_ctsvar!(model)?;
        var_to_task.insert(var, *t);
      }
    }

    let task_to_var: Map<_, _> = var_to_task.iter().map(|(&v, &t)| (t, v)).collect();
    let mut constraint_map = map_with_capacity(constraints.len());
    let mut constraint_map_inv = map_with_capacity(constraints.len());

    for c in constraints {
      let ineq = match c {
        SpConstr::Lb(t, lb) => c!(task_to_var[t] >= *lb),
        SpConstr::Ub(t, ub) => c!(task_to_var[t] <= *ub),
        SpConstr::Delta(t1, t2, d) => c!(task_to_var[t1] + *d <= task_to_var[t2]),
      };
      let grb_c = model.add_constr("", ineq)?;
      constraint_map.insert(*c, grb_c);
      constraint_map_inv.insert(grb_c, *c);
    }

    let obj = obj_tasks.iter().map(|t| task_to_var[t]).grb_sum();

    model.update()?;
    trace!(obj=?obj.with_names(&model));
    model.set_objective(obj, Minimize)?;

    Ok(TimingSubproblem {
      task_to_var,
      var_to_task,
      constraint_map,
      constraint_map_inv,
      obj_tasks,
      model,
      lu,
    })
  }

  pub fn iis_constraints<'b>(&'b self) -> Result<impl Iterator<Item=SpConstr> + 'b> {
    let model_cons = self.model.get_constrs()?;
    let iis_constr = self.model.get_obj_attr_batch(grb::prelude::attr::IISConstr, model_cons.iter().copied())?;
    Ok(iis_constr.into_iter().zip(model_cons).filter_map(move |(is_iis, grb_c)| {
      if is_iis > 0 {
        Some(self.constraint_map_inv[grb_c])
      } else {
        None
      }
    }))
  }

  pub fn mrs_constraints<'b>(&'b self) -> Result<impl Iterator<Item=SpConstr> + 'b> {
    let model_cons = self.model.get_constrs()?;
    let iis_constr = self.model.get_obj_attr_batch(grb::prelude::attr::Pi, model_cons.iter().copied())?;
    Ok(iis_constr.into_iter().zip(model_cons).filter_map(move |(dual, grb_c)| {
      if dual.abs() > 0.1 {
        Some(self.constraint_map_inv[grb_c])
      } else {
        None
      }
    }))
  }

  #[instrument(level = "error", name = "debug_iis", skip(self))]
  pub fn debug_infeasibility(&mut self) -> Result<()> {
    self.model.compute_iis()?;
    for cons in self.iis_constraints()? {
      error!(?cons);
    }
    Ok(())
  }
}

impl<'a> Subproblem<'a> for TimingSubproblem<'a> {
  type Optimal = ();
  type Infeasible = ();

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

  #[instrument(level = "trace", skip(self))]
  fn extract_and_remove_iis(&mut self, _: ()) -> Result<Iis> {
    self.model.compute_iis()?;

    let iis: Set<_> = self.iis_constraints()?.collect();
    trace!(?iis);

    match iis.iter().filter(|c| matches!(c, SpConstr::Lb(..))).count() {
      0 => Ok(Iis::Cycle(iis)),
      1 => Ok(Iis::Path(iis)),
      count => {
        error!(count, ?iis, "multiple lower bounds in IIS");
        panic!("bugalug")
      }
    }
  }

  fn add_optimality_cuts(&mut self, cb: &mut cb::Cb, _theta: &Map<(Avg, Task), Time>, _: ()) -> Result<()> {
    let sp_obj = self.model.get_attr(attr::ObjVal)?;
    let _s = trace_span!("optimality_cut", obj=%sp_obj).entered();
    let mut lhs = Expr::default();

    let mut num_edge_constraints = 0i32;
    let mut num_lbs = 0i32;
    for cons in self.mrs_constraints()? {
      match cons {
        SpConstr::Lb(t, lb) => {
          num_lbs += 1;
          cb.mp_vars.x_sum_similar_tasks_lb(self.lu, &t, lb).sum_into(&mut lhs)
        }
        SpConstr::Delta(t1, t2, _) => {
          num_edge_constraints += 1;
          cb.mp_vars.max_weight_edge_sum(
            self.lu,
            self.lu.tasks.by_index[&t1],
            self.lu.tasks.by_index[&t2],
          ).sum_into(&mut lhs);
        }
        SpConstr::Ub(..) => {
          error!(?cons, "UB constraint in MRS");
          panic!("bugalug")
        }
      }
      trace!(?cons, "MRS constraint");
    }

    self.obj_tasks.iter()
      .flat_map(|t| cb.mp_vars.y_sum_av(cb.lu, self.lu.tasks.by_index[t], cb.lu.tasks.ddepot))
      .sum_into(&mut lhs);

    let num_obj_tasks = self.obj_tasks.len() as i32;
    trace!(num_lbs, num_edge_constraints, num_obj_tasks);
    lhs = sp_obj * (1 - num_lbs - num_edge_constraints - num_obj_tasks + lhs);

    let rhs = self.lu.sets.avs()
      .cartesian_product(&self.obj_tasks)
      .filter_map(|(a, t)| cb.mp_vars.theta.get(&(a, self.lu.tasks.by_index[t])))
      .grb_sum();

    cb.enqueue_cut(c!(lhs <= rhs), CutType::LpOpt);
    Ok(())
  }
}