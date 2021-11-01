use daggylp::{Var, Graph, SolveStatus, edge_storage::{AdjacencyList, ArrayVec}, Weight, InfKind, mrs::MrsTree};
use smallvec::SmallVec;

use crate::*;
use super::*;
use crate::solution::MpSolution;
use crate::model::cb;
use crate::utils::{iter_pairs, VarIterExt};


use tracing::*;
use grb::prelude::*;
use crate::model::cb::CutType;
use sawmill::InferenceModel;

pub struct GraphModel<'a> {
  pub second_last_tasks: SmallVec<[IdxTask; NUM_AV_UB]>,
  pub lu: &'a Lookups,
  pub var_to_task: Map<Var, IdxTask>,
  /// ODepot var is *not* in this lookup.
  pub task_to_var: Map<IdxTask, Var>,
  pub edge_constraints: Map<(Var, Var), SpConstr>,
  pub ubs: Map<IdxTask, Time>,
  pub lbs: Map<IdxTask, Time>,
  pub model: Graph<AdjacencyList<ArrayVec<2>>>,

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

  fn big_grey_umgak(&mut self, cb: &mut cb::Cb, active_vars: &Set<MpVar> , inf_model: &InferenceModel<MpVar, SpConstr>) {
    let mrs = self.model.compute_mrs();


    fn hebbe(lbs: &[Time], delta: &[Time]) -> Time {
      debug_assert_eq!(lbs.len(), delta.len());
      let mut time = lbs[0] + delta[0];
      for (&lb, &d) in lbs[1..].iter().zip(&delta[1..]) {
        time = std::cmp::max(time, lb) + d;
      }
      time
    }

    let cb = |path: &[Var]| {
      let mut tasks = Vec::new();
      let mut implied_lbs = Vec::new();
      let mut edge_weights = Vec::new();
      let mut edge_constraint_order = map_with_capacity(path.len());

      for v in path {
        let t = self.var_to_task[v];
        implied_lbs.push(self.lu.tasks.by_index[&t].t_release);
        tasks.push(t);
      }
      let _s = info_span!("mrs_path", path=?tasks).entered();

      let lb_task = tasks[0];
      let lb = self.lbs[&lb_task];
      let mut mrs_objective  = lb;

      for (k, (&v1, &v2)) in iter_pairs(path).enumerate() {
        let c = self.edge_constraints[&(v1, v2)];
        let (_, _, d) = c.unwrap_delta();
        mrs_objective += d;
        edge_weights.push(d);
        edge_constraint_order.insert(c, k);
      }
      let last_lil_bit = self.lu.av_task_travel_time(&self.lu.tasks.by_index[tasks.last().unwrap()], &self.lu.tasks.ddepot);
      edge_weights.push(last_lil_bit);
      mrs_objective += last_lil_bit;

      let mut constraint_set: Set<_> = edge_constraint_order.keys().copied().collect();
      constraint_set.insert(SpConstr::Lb(lb_task, lb));
      let cover = inf_model.cover(active_vars, &constraint_set);

      for (var, constrs) in cover {
        let start_idx = if let Some(idx) = constrs.iter().map(|c| edge_constraint_order[c]).max() {
          idx + 1
        } else {
          debug_assert_eq!(constrs.len(), 1);
          0
        };
        let new_obj = hebbe(&implied_lbs[start_idx..], &edge_weights[start_idx..]);
        let coeff = new_obj - mrs_objective;
        error!(?var, ?constrs, new_obj, coeff)
      }
    };

    for mrs in mrs {
      mrs.visit_paths(cb);
    }
    todo!()
  }
}


impl<'a> GraphModel<'a> {
  #[tracing::instrument(level = "trace", skip(lu, sp_constraints, obj_tasks))]
  pub fn build(lu: &'a Lookups, sp_constraints: &Set<SpConstr>, obj_tasks: SmallVec<[IdxTask; NUM_AV_UB]>) -> Self {
    let mut ubs: Map<IdxTask, Time> = Map::default();
    let mut lbs: Map<IdxTask, Time> = Map::default();

    for c in sp_constraints {
      match c {
        &SpConstr::Ub(t, ub) => { ubs.insert(t, ub); },
        &SpConstr::Lb(t, lb) => { lbs.insert(t, lb); },
        _ => {}
      }
    }

    debug_assert_eq!(ubs.len(), lbs.len());
    let mut model = Graph::new();
    let mut var_to_task = map_with_capacity(ubs.len());
    let mut task_to_var = map_with_capacity(ubs.len());

    for (&t, &ub) in &ubs {
      let lb = lbs[&t];
      let obj = if obj_tasks.contains(&t) { 1 } else { 0 };
      trace!(?t, lb, ub, obj, "add var");
      let var = model.add_var(obj, lb as Weight, ub as Weight);
      var_to_task.insert(var, t);
      task_to_var.insert(t, var);
    }

    let mut edge_constraints : Map<(Var, Var), SpConstr> = Map::default();
    let mut model = model.finish_nodes();

    for c in sp_constraints {
      match c {
        &c @ SpConstr::Delta(t1, t2, d) => {
          let v1 = task_to_var[&t1];
          let v2 = task_to_var[&t2];
          trace!(?t1, ?t2, d, "add constr");
          model.add_constr(v1, d as Weight, v2);
          edge_constraints.insert((v1, v2), c);
        },
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
      second_last_tasks: obj_tasks,
      lu,
    }
  }

  pub fn solve_for_obj(&mut self) -> Result<Time> {
    match self.solve()? {
      SpStatus::Optimal(obj, _) => Ok(obj),
      SpStatus::Infeasible(kind) => anyhow::bail!("subproblem was infeasible, inf kind {:?}", kind)
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
  // Additional information obtained when solving the subproblem to optimality
  type Optimal = ();
  // Additional information obtained when proving the subproblem to be infeasible
  type Infeasible = InfKind;

  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus<Self::Optimal, Self::Infeasible>> {
    let s = match self.model.solve() {
      SolveStatus::Infeasible(kind) => SpStatus::Infeasible(kind),
      SolveStatus::Optimal => SpStatus::Optimal(self.model.compute_obj()? as Time, ())
    };
    Ok(s)
  }
  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self, i: Self::Infeasible) -> Result<Iis> {
    let graph_iis = self.model.compute_iis(true);
    self.model.remove_iis(&graph_iis);
    Ok(self.get_iis(i, &graph_iis))
  }

  fn add_optimality_cuts(&mut self, cb: &mut cb::Cb, theta: &Map<(Avg, Task), Time>, _o: Self::Optimal) -> Result<()> {
    for mrs in self.model.compute_mrs() {
      if !self.optimality_cut_required(theta, &mrs) {
        continue;
      }

      let (obj, edges) = self.model.edge_sensitivity_analysis(&mrs);
      let mut cut = obj * cb.mp_vars.x_sum_similar_tasks_lb(
        self.lu,
        &self.var_to_task[&mrs.root()],
        mrs.root_lb() as Time)
        .grb_sum();

      for ((v1, v2), old_obj, new_obj) in edges {
        let t1 = self.var_to_task[&v1];
        let t2 = self.var_to_task[&v2];
        // summand for (t1, t2)
        // = (x(t2) - y(t1,t2))*new_obj - (1 - y(t1, t2))*old_obj
        // = x(t2)*new_obj  + y(t1, t2)*(old_obj - new_obj) - old_obj
        for xvar in cb.mp_vars.x_sum_similar_tasks_lb(self.lu, &t2, self.lbs[&t2]) {
          cut += xvar * new_obj;
        }

        for var in cb.mp_vars.max_weight_edge_sum(self.lu, self.lu.tasks.by_index[&t1], self.lu.tasks.by_index[&t2]) {
          cut += (old_obj - new_obj) * var;
        }

        cut += -old_obj;
      }

      for (var, t) in mrs.obj_vars().map(|v| (v, self.lu.tasks.by_index[&self.var_to_task[&v]])) {
        // Conditional objective terms
        // I(x,y) = sum(Y[a, t, ddepot] for a in A)
        // each term is - (1 - I(x, y) ) * c(t) * optimal_time(t) where c(t) = 1
        // = I(x, y) * optimal_time(t) - optimal_time(t)
        let time = self.model.get_solution(&var)?;
        cb.mp_vars.y_sum_av(self.lu, t, self.lu.tasks.ddepot).map(|y| time * y).sum_into(&mut cut);
        cut += -time;


        // Theta-terms for this task
        for a in self.lu.sets.av_groups() {
          if let Some(&theta) = cb.mp_vars.theta.get(&(a, t)) {
            cut += -1 * theta;
          }
        }
      }

      cb.enqueue_cut(c!(cut <= 0), CutType::LpOpt);
    }
    Ok(())
  }
}