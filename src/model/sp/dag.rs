use daggylp::{Var, Graph, SolveStatus, edge_storage::{AdjacencyList, ArrayVec}, Weight, InfKind, mrs::MrsTree};
use smallvec::SmallVec;

use crate::*;
use super::*;
use crate::solution::MpSolution;
use crate::model::cb;
use crate::utils::{iter_pairs, VarIterExt, CollectExt};


use tracing::*;
use grb::prelude::*;
use crate::model::cb::CutType;
use sawmill::InferenceModel;

pub struct GraphModel<'a> {
  pub lu: &'a Lookups,
  pub var_to_task: Map<Var, IdxTask>,
  /// ODepot var is *not* in this lookup.
  pub task_to_var: Map<IdxTask, Var>,
  pub edge_constraints: Map<(Var, Var), SpConstr>,
  pub ubs: Map<IdxTask, Time>,
  pub lbs: Map<IdxTask, Time>,
  pub model: Graph<AdjacencyList<ArrayVec<2>>>,
  pub inf_kind: Option<InfKind>,
  pub obj_tasks: Map<IdxTask, Avg>,
  // pub pv_req_assignments: Map<Req, Pv>,
}


impl<'a> GraphModel<'a> {
  fn optimality_cut_required(&self, theta: &Map<(Avg, Task), Time>, mrs: &MrsTree) -> bool {
    let mut theta_sum = 0;
    for t in mrs.obj_vars().map(|v| self.lu.tasks.by_index[&self.var_to_task[&v]]) {
      for a in self.lu.sets.av_groups() {
        if let Some(&theta_v) = theta.get(&(a, t)) {
          theta_sum += theta_v;
        }
      }
    }
    (theta_sum as Weight) < mrs.obj()
  }
}


impl<'a> GraphModel<'a> {
  #[tracing::instrument(level = "trace", skip(lu, sp_constraints, obj_tasks))]
  pub fn build(lu: &'a Lookups,
               sp_constraints: &Set<SpConstr>,
               obj_tasks: Map<IdxTask, Avg>,
  ) -> Self {
    trace!(?obj_tasks);
    let mut ubs: Map<IdxTask, Time> = Map::default();
    let mut lbs: Map<IdxTask, Time> = Map::default();

    for c in sp_constraints {
      match c {
        &SpConstr::Ub(t, ub) => { ubs.insert(t, ub); }
        &SpConstr::Lb(t, lb) => { lbs.insert(t, lb); }
        _ => {}
      }
    }

    debug_assert_eq!(ubs.len(), lbs.len());
    let mut model = Graph::new();
    let mut var_to_task = map_with_capacity(ubs.len());
    let mut task_to_var = map_with_capacity(ubs.len());

    for (&t, &ub) in &ubs {
      let lb = lbs[&t];
      let obj = if obj_tasks.contains_key(&t) { 1 } else { 0 };
      trace!(?t, lb, ub, obj, "add var");
      let var = model.add_var(obj, lb as Weight, ub as Weight);
      var_to_task.insert(var, t);
      task_to_var.insert(t, var);
    }

    let mut edge_constraints: Map<(Var, Var), SpConstr> = Map::default();
    let mut model = model.finish_nodes();

    for c in sp_constraints {
      match c {
        &c @ SpConstr::Delta(t1, t2, d) if t2 != IdxTask::DDepot => {
          let v1 = task_to_var[&t1];
          let v2 = task_to_var[&t2];
          trace!(?t1, ?t2, d, "add constr");
          model.add_constr(v1, d as Weight, v2);
          edge_constraints.insert((v1, v2), c);
        }
        _ => {}
      }
    }

    GraphModel {
      model: model.finish(),
      var_to_task,
      task_to_var,
      edge_constraints,
      lbs,
      ubs,
      obj_tasks,
      lu,
      inf_kind: None,
    }
  }

  pub fn solve_for_obj(&mut self) -> Result<Time> {
    match self.solve()? {
      SpStatus::Optimal(obj) => Ok(obj),
      SpStatus::Infeasible => anyhow::bail!("subproblem was infeasible")
    }
  }

  pub fn get_iis(&self, inf_kind: InfKind, graph_iis: &daggylp::Iis) -> Iis {
    match inf_kind {
      InfKind::Path => {
        let (lb_var, ub_var) = graph_iis.bounds().unwrap();
        let lb_t = self.var_to_task[&lb_var];
        let lb = self.lbs[&lb_t];

        let ub_t = self.var_to_task[&ub_var];
        let ub = self.ubs[&ub_t];

        let mut iis: Set<_> = graph_iis.iter_edge_constraints()
          .map(|e| self.edge_constraints[&e])
          .collect();

        iis.insert(SpConstr::Ub(ub_t, ub));
        iis.insert(SpConstr::Lb(lb_t, lb));

        Iis::Path(iis)
      }

      InfKind::Cycle => {
        let cycle = graph_iis.iter_edge_constraints()
          .map(|e| self.edge_constraints[&e])
          .collect();
        Iis::Cycle(cycle)
      }
    }
  }
}

impl<'a> Subproblem<'a> for GraphModel<'a> {
  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus> {
    let s = match self.model.solve() {
      SolveStatus::Infeasible(kind) => {
        self.inf_kind = Some(kind);
        SpStatus::Infeasible
      }
      SolveStatus::Optimal => {
        let obj = self.model.compute_obj()? as Time +
          self.obj_tasks.keys()
            .map(|t| self.lu.data.travel_time_to_ddepot(&self.lu.tasks.by_index[t]))
            .sum::<Time>();
        SpStatus::Optimal(obj)
      }
    };
    Ok(s)
  }

  fn calculate_theta(&mut self) -> Result<ThetaVals> {
    let theta = self.obj_tasks.iter()
      .map(|(t, &a)| {
        let task = self.lu.tasks.by_index[t];
        let val = self.model.get_solution(&self.task_to_var[t])? as Time
          + self.lu.data.travel_time_to_ddepot(&task);
        Ok(((a, task), val))
      })
      .collect_ok()?;
    Ok(theta)
  }

  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self) -> Result<Iis> {
    let graph_iis = self.model.compute_iis(true);
    self.model.remove_iis(&graph_iis);
    Ok(self.get_iis(self.inf_kind.expect("expected infeasible model"), &graph_iis))
  }

  // Find the MRS paths which need an optimality cut
  fn visit_mrs_paths(&mut self, visitor: &mut MrsPathVisitor) -> Result<()> {
    let mrs = self.model.compute_mrs();

    for mrs in mrs {
      if !mrs.has_obj_vars() {
        continue;
      }
      mrs.visit_paths(|p| {
        let last_var = p.last().unwrap();
        let last_task = self.lu.tasks.by_index[&self.var_to_task[last_var]];
        let mrs_objective = self.model.get_solution(last_var).unwrap() as Time + self.lu.data.travel_time_to_ddepot(&last_task);
        let tasks: Vec<_> = p.iter().map(|v| self.lu.tasks.by_index[&self.var_to_task[v]]).collect();
        trace!(?tasks, mrs_objective);
        if visitor.optimality_cut_required(last_task, mrs_objective) {
          let mut edge_constraints: Vec<_> = iter_pairs(p).map(|(&v1, &v2)| self.edge_constraints[&(v1, v2)]).collect();
          let last_task = tasks.last().unwrap();
          edge_constraints.push(
            SpConstr::Delta(last_task.index(), IdxTask::DDepot, self.lu.data.travel_time_to_ddepot(last_task))
          );

          visitor.add_optimality_cut(MrsPath {
            objective: mrs_objective,
            lower_bound: self.lbs[&tasks[0].index()],
            tasks,
            edge_constraints,
          });
        }
      })
    }
    Ok(())
  }
}