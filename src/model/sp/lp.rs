use anyhow::Context;
use daggylp::Weight;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use grb::prelude::*;
use itertools::Itertools;
use smallvec::SmallVec;
use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::hash::{BuildHasher, Hash};
use std::io::BufWriter;
use std::path::Path;
use tracing::*;

use crate::model::cb::{Cb, CbError, CutType};
use crate::model::mp::MpVars;
use crate::solution::*;
use crate::tasks::TaskId;
use crate::utils::{HashMapExt, IoContext};
use crate::*;

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
  pub obj_tasks: Map<IdxTask, Avg>,
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

      trace!(lb, ub, ?t, ?pvt, "bounds");

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
      let d = lu.av_task_travel_time(&t1, &t2);
      let t1 = t1.index();
      let t2 = t2.index();
      let grb_c = model.add_constr("", c!(task_to_var[&t1] + d <= task_to_var[&t2]))?;
      constraint_map.insert(SpConstr::Delta(t1, t2, d), grb_c);
    }

    let obj_tasks = sol.sp_objective_tasks();
    model.set_objective(obj_tasks.keys().map(|t| task_to_var[t]).grb_sum(), Minimize)?;

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

  pub fn build(
    lu: &'a Lookups,
    constraints: &Set<SpConstr>,
    obj_tasks: Map<IdxTask, Avg>,
  ) -> Result<TimingSubproblem<'a>> {
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

    let obj = obj_tasks.keys().map(|t| task_to_var[t]).grb_sum();

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

  pub fn iis_constraints<'b>(&'b self) -> Result<impl Iterator<Item = SpConstr> + 'b> {
    let model_cons = self.model.get_constrs()?;
    let iis_constr = self
      .model
      .get_obj_attr_batch(grb::prelude::attr::IISConstr, model_cons.iter().copied())?;
    Ok(
      iis_constr
        .into_iter()
        .zip(model_cons)
        .filter_map(move |(is_iis, grb_c)| {
          if is_iis > 0 {
            Some(self.constraint_map_inv[grb_c])
          } else {
            None
          }
        }),
    )
  }

  pub fn mrs_constraints<'b>(&'b self) -> Result<impl Iterator<Item = SpConstr> + 'b> {
    let model_cons = self.model.get_constrs()?;
    let iis_constr = self
      .model
      .get_obj_attr_batch(grb::prelude::attr::Pi, model_cons.iter().copied())?;
    Ok(
      iis_constr
        .into_iter()
        .zip(model_cons)
        .filter_map(move |(dual, grb_c)| {
          if dual.abs() > 0.1 {
            Some(self.constraint_map_inv[grb_c])
          } else {
            None
          }
        }),
    )
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
  fn calculate_theta(&mut self) -> Result<ThetaVals> {
    todo!()
  }

  fn solve(&mut self) -> Result<SpStatus> {
    self.model.optimize()?;
    match self.model.status()? {
      Status::Optimal => {
        let obj_val = self.model.get_attr(attr::ObjVal)?.round() as Time;
        Ok(SpStatus::Optimal(obj_val))
      }
      Status::Infeasible => Ok(SpStatus::Infeasible),
      other => {
        let err = CbError::Status(other);
        error!(status=?other, "{}", err);
        Err(err.into())
      }
    }
  }

  #[instrument(level = "trace", skip(self))]
  fn extract_and_remove_iis(&mut self) -> Result<Iis> {
    self.model.compute_iis()?;

    let iis: Set<_> = self.iis_constraints()?.collect();
    trace!(?iis);

    for c in &iis {
      let grb_c = self
        .constraint_map
        .remove(c)
        .expect("IIS constraint missing from constraint_map");
      self.constraint_map_inv.remove(&grb_c).unwrap();
      self
        .model
        .remove(grb_c)
        .context("removing IIS constraint from Gurobi model")?;
    }

    match iis.iter().filter(|c| matches!(c, SpConstr::Lb(..))).count() {
      0 => Ok(Iis::Cycle(iis)),
      1 => Ok(Iis::Path(iis)),
      count => {
        error!(count, ?iis, "multiple lower bounds in IIS");
        panic!("bugalug")
      }
    }
  }

  // Find the MRS paths
  fn visit_critical_paths(&mut self, _visitor: &mut CriticalPathVisitor) -> Result<()> {
    todo!()
  }
}
