use daggylp::{Var, Graph, SolveStatus, edge_storage::{AdjacencyList, ArrayVec}, Weight, InfKind};

use crate::*;
use crate::model::sp::{Iis, Subproblem, SpStatus, iter_edge_constraints, XEdgeConstraint};
use crate::solution::Solution;
use crate::model::cb;
use crate::utils::iter_pairs;


use tracing::*;
use grb::prelude::*;
use crate::model::cb::CutType;

struct GraphModel<'a> {
  lu: &'a Lookups,
  vars: Map<Task, Var>,
  var_to_task: Map<Var, PvTask>,
  edges: Map<(Var, Var), (Task, Task)>,
  model: Graph<AdjacencyList<ArrayVec<2>>>,
}

impl<'a> Subproblem<'a> for GraphModel<'a> {
  // Additional information obtained when solving the subproblem to optimality
  type Optimal = ();
  // Additional information obtained when proving the subproblem to be infeasible
  type Infeasible = InfKind;
  // A group of constraint sets representing one or more IIS
  type IisConstraintSets = std::iter::Once<Iis>;

  #[tracing::instrument(level="trace", skip(lu, sol))]
  fn build(lu: &'a Lookups, sol: &'a Solution) -> Result<Self> {
    let mut model = Graph::new();
    let mut vars = map_with_capacity(lu.data.n_req  as usize * 2);
    let mut var_to_task = map_with_capacity(lu.data.n_req  as usize * 2);

    for (_, pv_route) in &sol.pv_routes {
      for &pt in pv_route {
        let t = lu.tasks.pvtask_to_task[&pt];

        // loop over all AVs here, probably better than allocating.
        let obj = if sol.av_routes.iter().any(|(_, av_route)| av_route[av_route.len()-2] == t) {
          1
        } else {
          0
        };

        let var = model.add_var(obj, pt.t_release as Weight, pt.t_deadline as Weight);
        vars.insert(t, var);
        var_to_task.insert(var, pt);
      }
    }

    let mut constraints = map_with_capacity(vars.len() * 2);
    let mut model = model.finish_nodes();
    for (_, pv_route) in &sol.pv_routes {
      for c in iter_edge_constraints(pv_route) {
        let (pt1, pt2) = match c {
          XEdgeConstraint::Unloading(pt1, pt2) =>  {
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
    }

    for (_, av_route) in &sol.av_routes {
      for (t1, t2) in iter_pairs(av_route) {
        let v1 = vars[&t1];
        let v2 = vars[&t2];

        constraints.entry((v1, v2))
          .or_insert_with(|| {
            trace!(?t1, ?t2, "AV travel time constraint");
            let d = t1.tt + lu.data.travel_time[&(t1.end, t2.start)];
            model.add_constr(v1, d as Weight, v2);
            (*t1, *t2)
          });
      }
    }
    Ok(GraphModel {
      lu,
      vars,
      var_to_task,
      edges: constraints,
      model: model.finish(),
    })
  }

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
    let iis = self.model.compute_iis(true);
    let iis = match i {
      InfKind::Path => {
        let (lb_var, ub_var) = iis.bounds().unwrap();
        let lb_pt = self.var_to_task[&lb_var];
        let ub_pt = self.var_to_task[&ub_var];
        let path = iis.iter_edge_vars()
          .map(|v| self.lu.tasks.pvtask_to_task[&self.var_to_task[&v]])
          .collect();

        Iis::Path {
          ub: ub_pt,
          lb: lb_pt,
          path,
        }
      },

      InfKind::Cycle => {
        let cycle = iis.iter_edge_vars()
          .map(|v| self.lu.tasks.pvtask_to_task[&self.var_to_task[&v]])
          .collect();
        Iis::Cycle(cycle)
      },
    };

    Ok(std::iter::once(iis))
  }

  fn add_optimality_cuts(&mut self, cb: &mut cb::Cb, o: Self::Optimal) -> Result<()> {
    for mrs in self.model.compute_mrs() {
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