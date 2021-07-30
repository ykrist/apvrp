use daggylp::{Var, Graph, SolveStatus, edge_storage::{AdjacencyList, ArrayVec}, Weight, InfKind, mrs::MrsTree};
use smallvec::SmallVec;

use crate::*;
use super::*;
use crate::solution::Solution;
use crate::model::cb;
use crate::utils::{iter_pairs, VarIterExt};


use tracing::*;
use grb::prelude::*;
use crate::model::cb::CutType;

pub struct GraphModel<'a> {
  pub second_last_tasks: SmallVec<[Task; NUM_AV_UB]>,
  pub lu: &'a Lookups,
  pub theta_val: &'a Map<(Avg, Task), Time>, // FIXME make this an argument to add_optimality_cuts()
  pub vars: Map<Task, Var>,
  /// ODepot var is *not* in this lookup.
  pub var_to_task: Map<Var, PvTask>,
  pub edges: Map<(Var, Var), (Task, Task)>,
  pub model: Graph<AdjacencyList<ArrayVec<2>>>,
}

impl<'a> GraphModel<'a> {
  fn optimality_cut_required(&self, mrs: &MrsTree) -> bool {
    let mut theta_sum = 0;
    for t in mrs.obj_vars().map(|v| self.lu.tasks.pvtask_to_task[&self.var_to_task[&v]]) {
      for a in self.lu.sets.avs() {
        if let Some(&theta_v) = self.theta_val.get(&(a, t)) {
          theta_sum += theta_v;
        }
      }
    }
    (theta_sum as Weight) < mrs.obj()
  }
}


impl<'a> GraphModel<'a> {
  #[tracing::instrument(level = "trace", skip(lu, sol, theta_val))]
  pub fn build(lu: &'a Lookups, sol: &'a Solution, theta_val: &'a Map<(Avg, Task), Time>) -> Result<Self> {
    let mut model = Graph::new();
    let mut vars = map_with_capacity(lu.data.n_req as usize * 2);
    let mut var_to_task = map_with_capacity(lu.data.n_req as usize * 2);

    let second_last_tasks: SmallVec<_> = sol.av_routes.iter()
      .map(|r| *r.theta_task())
      .collect();

    let task_objective = {
      let obj_tasks = second_last_tasks.as_slice();
      move |t: &Task| -> Weight {
        // loop over all AVs here, probably better than allocating on the heap.
        if obj_tasks.contains(t) { 1 } else { 0 }
      }
    };

    for &pt in sol.iter_sp_vars() {
      let t = lu.tasks.pvtask_to_task[&pt];
      let var = model.add_var(task_objective(&t), pt.t_release as Weight, (pt.t_deadline - pt.tt) as Weight);
      let prev = var_to_task.insert(var, pt);
      debug_assert_eq!(prev, None, "Duplicate variable in subproblem");
      let prev = vars.insert(t, var);
      debug_assert_eq!(prev, None);
    }

    let mut constraints = map_with_capacity(vars.len() * 2);
    let mut model = model.finish_nodes();

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
      let t1 = lu.tasks.pvtask_to_task[pt1];
      let t2 = lu.tasks.pvtask_to_task[pt2];
      let d = t1.tt + lu.data.srv_time[&t1.end];
      let v1 = vars[&t1];
      let v2 = vars[&t2];
      model.add_constr(v1, d as Weight, v2);
      constraints.insert((v1, v2), (t1, t2));
    }

    for (&t1, &t2) in sol.iter_sp_y_edges() {
      trace!(?t1, ?t2);
      let v1 = vars[&t1];
      let v2 = vars[&t2];

      constraints.entry((v1, v2)).or_insert_with(|| {
        trace!(?t1, ?t2, "AV travel  time constraint");
        let d = t1.tt + lu.data.travel_time[&(t1.end, t2.start)];
        model.add_constr(v1, d as Weight, v2);
        (t1, t2)
      });
    }

    Ok(GraphModel {
      second_last_tasks,
      lu,
      theta_val,
      vars,
      var_to_task,
      edges: constraints,
      model: model.finish(),
    })
  }

  pub fn solve_for_obj(&mut self) -> Result<Time> {
    match self.solve()? {
      SpStatus::Optimal(obj, _) => Ok(obj),
      SpStatus::Infeasible(kind) => anyhow::bail!("subproblem was infeasible, inf kind {:?}", kind)
    }
  }
}

impl<'a> Subproblem<'a> for GraphModel<'a> {
  // Additional information obtained when solving the subproblem to optimality
  type Optimal = ();
  // Additional information obtained when proving the subproblem to be infeasible
  type Infeasible = InfKind;
  // A group of constraint sets representing one or more IIS
  type IisConstraintSets = std::iter::Once<Iis>;

  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus<Self::Optimal, Self::Infeasible>> {
    let s = match self.model.solve() {
      SolveStatus::Infeasible(kind) => SpStatus::Infeasible(kind),
      SolveStatus::Optimal => SpStatus::Optimal(self.model.compute_obj()? as Time, ())
    };
    Ok(s)
  }
  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self, i: Self::Infeasible) -> Result<Self::IisConstraintSets> {
    let graph_iis = self.model.compute_iis(true);
    let iis = match i {
      InfKind::Path => {
        let (lb_var, ub_var) = graph_iis.bounds().unwrap();
        let lb_pt = self.var_to_task[&lb_var];
        let ub_pt = self.var_to_task[&ub_var];
        let path = graph_iis.iter_edge_vars()
          .map(|v| self.lu.tasks.pvtask_to_task[&self.var_to_task[&v]])
          .collect();

        Iis::Path(PathIis {
          ub: ub_pt,
          lb: lb_pt,
          path,
        })
      }

      InfKind::Cycle => {
        let cycle = graph_iis.iter_edge_vars()
          .map(|v| self.lu.tasks.pvtask_to_task[&self.var_to_task[&v]])
          .collect();
        Iis::Cycle(cycle)
      }
    };
    self.model.remove_iis(&graph_iis);
    Ok(std::iter::once(iis))
  }

  fn add_optimality_cuts(&mut self, cb: &mut cb::Cb, _o: Self::Optimal) -> Result<()> {
    for mrs in self.model.compute_mrs() {
      if !self.optimality_cut_required(&mrs) {
        continue;
      }

      let (obj, edges) = self.model.edge_sensitivity_analysis(&mrs);
      let mut cut = cb.mp_vars.x_sum_similar_tasks(self.lu, self.var_to_task[&mrs.root()]).grb_sum() * obj;

      for ((v1, v2), old_obj, new_obj) in edges {
        let (t1, t2) = self.edges[&(v1, v2)];
        // summand for (t1, t2)
        // = (x(t2) - y(t1,t2))*new_obj - (1 - y(t1, t2))*old_obj
        // = x(t2)*new_obj  + y(t1, t2)*(old_obj - new_obj) - old_obj
        for xvar in cb.mp_vars.x_sum_similar_tasks(self.lu, self.var_to_task[&v2]) {
          cut += xvar * new_obj;
        }

        for var in cb.mp_vars.max_weight_edge_sum(self.lu, t1, t2) {
          cut += (old_obj - new_obj) * var;
        }

        cut += -old_obj;
      }

      // Now need to add the conditional-objective terms
      for &t in &self.second_last_tasks {
        // I(x,y) = sum(Y[a, t, ddepot] for a in A)

        // each term is - (1 - I(x, y) ) * c(t) * optimal_time(t) where c(t) = 1
        // = I(x, y) * optimal_time(t) - optimal_time(t)
        let time = self.model.get_solution(&self.vars[&t])?;
        cb.mp_vars.y_sum_av(self.lu, t, self.lu.tasks.ddepot).map(|y| time * y).sum_into(&mut cut);
        cut += -time;
      }

      // Theta-terms
      for t in mrs.obj_vars().map(|v| self.lu.tasks.pvtask_to_task[&self.var_to_task[&v]]) {
        for a in self.lu.sets.avs() {
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