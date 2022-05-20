use daggylp::{
  edge_storage::{AdjacencyList, ArrayVec},
  mrs::MrsTree,
  Graph, InfKind, SolveStatus, Var, Weight,
};
use smallvec::SmallVec;

use super::*;
use crate::model::cb;
use crate::solution::MpSolution;
use crate::utils::{iter_pairs, CollectExt, VarIterExt};
use crate::*;

use crate::model::cb::CutType;
use grb::prelude::*;
use sawmill::InferenceModel;
use std::cmp::max;
use tracing::*;

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
    for t in mrs
      .obj_vars()
      .map(|v| self.lu.tasks.by_index[&self.var_to_task[&v]])
    {
      for a in self.lu.sets.av_groups() {
        if let Some(&theta_v) = theta.get(&(a, t)) {
          theta_sum += theta_v;
        }
      }
    }
    (theta_sum as Weight) < mrs.obj()
  }

  fn try_reduce_cycle_iis(&self, graph_iis: &daggylp::Iis) -> Iis {
    let cycle_len = graph_iis.len();
    let mut edge_constraints = Vec::with_capacity(cycle_len * 2 - 1);
    edge_constraints.extend(
      graph_iis
        .iter_edge_constraints()
        .map(|c| self.edge_constraints[&c].unwrap_delta()),
    );

    debug_assert_eq!(edge_constraints.len(), cycle_len);
    debug_assert_eq!(
      edge_constraints.first().unwrap().0,
      edge_constraints.last().unwrap().1
    );

    edge_constraints.extend_from_within(..edge_constraints.len() - 1);

    for chain_length in 3..(cycle_len - 1) {
      for chain in edge_constraints.windows(chain_length) {
        let first_task = chain.first().unwrap().0;
        let last_task = chain.last().unwrap().1;
        debug_assert_ne!(first_task, last_task);

        let mut time = self.lbs[&first_task];

        for (_, t, d) in chain {
          time = max(time + d, self.lbs[t]);
        }

        if time > self.ubs[&last_task] {
          let mut iis = set_with_capacity(chain.len() + 2);
          iis.extend(chain.iter().map(|&(t1, t2, d)| SpConstr::Delta(t1, t2, d)));
          iis.insert(SpConstr::Lb(first_task, self.lbs[&first_task]));
          iis.insert(SpConstr::Ub(last_task, self.ubs[&last_task]));
          debug!(path=?chain, cycle=?edge_constraints[..cycle_len], "Found path IIS within cycle IIS");
          return Iis::Path(iis);
        }
      }
    }

    let iis = edge_constraints
      .into_iter()
      .take(cycle_len)
      .map(|(t1, t2, d)| SpConstr::Delta(t1, t2, d))
      .collect();
    Iis::Cycle(iis)
  }

  pub fn pv_task_to_var(&self, t: &PvTask) -> Option<Var> {
    self
      .task_to_var
      .get(&self.lu.tasks.pvtask_to_task[t].index())
      .copied()
  }

  #[tracing::instrument(level = "trace", skip(lu, s))]
  pub fn build_from_solution(lu: &'a Lookups, s: &MpSolution) -> Self {
    assert!(s.pv_cycles.is_empty() && s.av_cycles.is_empty());

    let obj_tasks = s.sp_objective_tasks();
    trace!(?obj_tasks);

    let mut ubs: Map<IdxTask, Time> = Map::default();
    let mut lbs: Map<IdxTask, Time> = Map::default();
    let mut model = Graph::new();
    let mut var_to_task = map_with_capacity(ubs.len());
    let mut task_to_var = map_with_capacity(ubs.len());

    for route in s.pv_routes.iter() {
      for t in route.0.iter() {
        let av_t = lu.tasks.pvtask_to_task[t].index();
        let lb = t.subproblem_lb();
        let ub = t.subproblem_ub();

        let none = lbs.insert(av_t, lb);
        debug_assert_eq!(none, None);

        let none = ubs.insert(av_t, ub);
        debug_assert_eq!(none, None);

        let obj = if obj_tasks.contains_key(&av_t) { 1 } else { 0 };
        trace!(?t, lb, ub, obj, "add var");

        let var = model.add_var(obj, lb as Weight, ub as Weight);

        var_to_task.insert(var, av_t);
        let none = task_to_var.insert(av_t, var);
        debug_assert_eq!(none, None);
      }
    }
    let mut edge_constraints: Map<(Var, Var), SpConstr> = Map::default();
    let mut model = model.finish_nodes();

    for route in &s.pv_routes {
      for (k, t) in route.0.iter().enumerate() {
        if t.ty == TaskType::Request {
          let av_t = lu.tasks.pvtask_to_task[t].index();
          let prev = &route.0[k - 1];
          let prev_t = lu.tasks.pvtask_to_task[prev].index();
          let next = &route.0[k + 1];
          let next_t = lu.tasks.pvtask_to_task[next].index();

          debug_assert_matches!(prev.ty, TaskType::Transfer | TaskType::Start);
          debug_assert_matches!(next.ty, TaskType::Transfer | TaskType::End);

          let var = task_to_var[&av_t];
          let prev_var = task_to_var[&prev_t];
          let next_var = task_to_var[&next_t];

          let d = prev.tt + lu.data.srv_time[&prev.end];
          model.add_constr(prev_var, d as Weight, var);
          let none = edge_constraints.insert((prev_var, var), SpConstr::Delta(prev_t, av_t, d));
          debug_assert_eq!(none, None);

          let d = t.tt + lu.data.srv_time[&t.end];
          model.add_constr(var, d as Weight, next_var);
          let none = edge_constraints.insert((var, next_var), SpConstr::Delta(av_t, next_t, d));
          debug_assert_eq!(none, None);
        }
      }
    }

    for route in &s.av_routes {
      for (t1, t2) in route.iter_edges() {
        if t1.is_depot() || t2.is_depot() {
          continue;
        }
        let v1 = task_to_var[&t1.index()];
        let v2 = task_to_var[&t2.index()];

        if edge_constraints.contains_key(&(v1, v2)) {
          continue;
        }

        let d = lu.av_task_travel_time(t1, t2);
        edge_constraints.insert((v1, v2), SpConstr::Delta(t1.index(), t2.index(), d));
        model.add_constr(v1, d as Weight, v2);
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

  #[tracing::instrument(level = "trace", skip(lu, sp_constraints, obj_tasks))]
  pub fn build(
    lu: &'a Lookups,
    sp_constraints: &Set<SpConstr>,
    obj_tasks: Map<IdxTask, Avg>,
  ) -> Self {
    trace!(?obj_tasks);
    let mut ubs: Map<IdxTask, Time> = Map::default();
    let mut lbs: Map<IdxTask, Time> = Map::default();

    for c in sp_constraints {
      match c {
        &SpConstr::Ub(t, ub) => {
          ubs.insert(t, ub);
        }
        &SpConstr::Lb(t, lb) => {
          lbs.insert(t, lb);
        }
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
    match self.solve() {
      SpStatus::Optimal(obj) => Ok(obj),
      SpStatus::Infeasible => anyhow::bail!("subproblem was infeasible"),
    }
  }

  // Solve the subproblem and return status
  pub fn solve(&mut self) -> SpStatus {
    match self.model.solve() {
      SolveStatus::Infeasible(kind) => {
        self.inf_kind = Some(kind);
        SpStatus::Infeasible
      }
      SolveStatus::Optimal => {
        let obj = self.model.compute_obj().unwrap() as Time
          + self
            .obj_tasks
            .keys()
            .map(|t| {
              self
                .lu
                .data
                .travel_time_to_ddepot(&self.lu.tasks.by_index[t])
            })
            .sum::<Time>();
        SpStatus::Optimal(obj)
      }
    }
  }

  pub(super) fn calculate_theta(&mut self) -> Result<ThetaVals> {
    let theta = self
      .obj_tasks
      .iter()
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
  pub fn extract_and_remove_iis(&mut self) -> Iis {
    let graph_iis = self.model.compute_iis(true);
    self.model.remove_iis(&graph_iis);
    match self.inf_kind.expect("expected infeasible model") {
      InfKind::Path => {
        let (lb_var, ub_var) = graph_iis.bounds().unwrap();
        let lb_t = self.var_to_task[&lb_var];
        let lb = self.lbs[&lb_t];

        let ub_t = self.var_to_task[&ub_var];
        let ub = self.ubs[&ub_t];

        let mut iis: Set<_> = graph_iis
          .iter_edge_constraints()
          .map(|e| self.edge_constraints[&e])
          .collect();

        iis.insert(SpConstr::Ub(ub_t, ub));
        iis.insert(SpConstr::Lb(lb_t, lb));

        Iis::Path(iis)
      }

      InfKind::Cycle => self.try_reduce_cycle_iis(&graph_iis),
    }
  }
}
