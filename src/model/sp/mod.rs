use crate::*;
use crate::solution::Solution;
use super::{cb, cb::CutType};
use smallvec::SmallVec;
use grb::constr::IneqExpr;
use grb::prelude::*;
use crate::logging::*;
use crate::model::{EdgeConstrKind, edge_constr_kind};

use crate::utils::{iter_pairs, VarIterExt, PeekableExt};
pub mod lp;
pub mod dag;

#[derive(Debug, Clone)]
pub enum Iis {
  /// Includes first task LB, last task UB, tasks must be in time-order
  Path {
    lb: PvTask,
    ub: PvTask,
    path: SmallVec<[Task; 10]>,
  },
  /// Tasks should be ordered according to the cycle.  First task should NOT be the same as the last task.
  Cycle(SmallVec<[Task; 10]>),
}

#[derive(Debug)]
pub enum SpStatus<O, I> {
  Optimal(Time, O),
  Infeasible(I),
}

impl<O, I> SpStatus<O, I> {
  fn is_optimal(&self) -> bool {
    matches!(self, SpStatus::Optimal(..))
  }
}

pub fn solve_subproblem_and_add_cuts<'a, S: Subproblem<'a>>(cb: &'a mut cb::Cb, sol: &'a Solution, estimate: Time) -> Result<()> {
  let mut subproblem = S::build(cb.lu, sol)?;
  loop {
    match subproblem.solve()? {
      SpStatus::Optimal(sp_obj, o) => {
        if estimate < sp_obj {
          subproblem.add_optimality_cuts(cb, o)?;
        }
        break;
      }
      SpStatus::Infeasible(i) => {
        for iis in subproblem.extract_and_remove_iis(i)?.into_iter() {
          let cut = match iis {
            Iis::Path { lb, ub, path } =>
              build_path_infeasiblity_cut(cb, lb, ub, &path),
            Iis::Cycle(cycle) => build_cyclic_infeasiblity_cut(cb, &cycle),
          };
          cb.enqueue_cut(cut, CutType::LpFeas); // FIXME cuttype should be changed
        }
      }
    }
  }
  Ok(())
}

pub trait Subproblem<'a>: Sized {
  // Additional information obtained when solving the subproblem to optimality
  type Optimal;
  // Additional information obtained when proving the subproblem to be infeasible
  type Infeasible;
  // A group of constraint sets representing one or more IIS
  type IisConstraintSets: IntoIterator<Item=Iis>;

  fn build(lu: &'a Lookups, sol: &'a Solution) -> Result<Self>;

  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus<Self::Optimal, Self::Infeasible>>;
  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self, i: Self::Infeasible) -> Result<Self::IisConstraintSets>;
  // Find and remove one or more MRS when optimal
  fn add_optimality_cuts(&mut self, cb: &mut cb::Cb, o: Self::Optimal) -> Result<()>;
}

#[instrument(level="debug", skip(cb))]
pub fn build_path_infeasiblity_cut(cb: &cb::Cb, lb_task: PvTask, ub_task: PvTask, path: &[Task]) -> IneqExpr {
  let n = path.len() - 1;
  debug_assert_eq!((lb_task.start, lb_task.end), (path[0].start, path[0].end));
  debug_assert_eq!((ub_task.start, ub_task.end), (path[n].start, path[n].end));
  debug_assert!(path.len() > 1);

  let mut lhs = Expr::default();
  let mut rhs = 0;

  let first_edge = (path[0], path[1]);
  let last_edge = (path[n-1], path[n]);
  let mut x_edges = Set::default();

  match edge_constr_kind(&first_edge.0, &first_edge.1) {
    EdgeConstrKind::AvTravelTime => {
      rhs += 1;
      cb.mp_vars.y_sum_av(cb.lu, first_edge.0, first_edge.1).sum_into(&mut lhs);
    }
    EdgeConstrKind::Unloading => {
      x_edges.insert(first_edge.1);
    }
    EdgeConstrKind::Loading => {
      rhs += 1;
      cb.mp_vars.x_sum_similar_tasks(cb.lu, lb_task).sum_into(&mut lhs);
    }
  }

  match edge_constr_kind(&last_edge.0, &last_edge.1) {
    EdgeConstrKind::AvTravelTime => {
      rhs += 1;
      cb.mp_vars.y_sum_av(cb.lu, last_edge.0, last_edge.1).sum_into(&mut lhs);
    }
    EdgeConstrKind::Loading => {
      x_edges.insert(last_edge.0);
    }
    EdgeConstrKind::Unloading => {
      rhs += 1;
      cb.mp_vars.x_sum_similar_tasks(cb.lu, ub_task).sum_into(&mut lhs);
    }
  }

  let edge_nodes = &path[1..n];

  for (t1, t2) in iter_pairs(edge_nodes) {
    match edge_constr_kind(t1, t2) {
      EdgeConstrKind::AvTravelTime => {
        rhs += 1;
        cb.mp_vars.y_sum_av(cb.lu, *t1, *t2).sum_into(&mut lhs)
      },
      EdgeConstrKind::Loading => {
        x_edges.insert(*t1);
      },
      EdgeConstrKind::Unloading => {
        x_edges.insert(*t2);
      }
    }
  }

  rhs += x_edges.len();
  x_edges.iter()
    .flat_map(|t| &cb.lu.tasks.task_to_pvtasks[t])
    .map(|t| cb.mp_vars.x[t])
    .sum_into(&mut lhs);

  trace!(rhs, iis_size=path.len()-1, "generate cut");
  debug_assert!(rhs > 1);
  rhs -= 1;
  c!( lhs <= rhs)
}

pub fn build_cyclic_infeasiblity_cut(cb: &cb::Cb, cycle: &[Task]) -> IneqExpr {
  debug_assert!(cycle.len() > 2, "cycle too short: {:?}", cycle);
  let lhs = cycle.iter().copied()
    .tuple_windows()
    .chain(std::iter::once((cycle[cycle.len()-1], cycle[0])))
    .flat_map(|(t1, t2)| cb.mp_vars.max_weight_edge_sum(cb.lu, t1, t2))
    .grb_sum();
  c!(lhs <= cycle.len() - 1)
}

#[derive(Copy, Clone, Debug)]
enum XEdgeConstraint<'a> {
  Loading(&'a PvTask, &'a PvTask),
  Unloading(&'a PvTask, &'a PvTask),
}

fn iter_edge_constraints<'a>(pv_route: &'a [PvTask]) -> impl Iterator<Item=XEdgeConstraint<'a>> + 'a {
  iter_pairs(pv_route).filter_map(|(t1, t2)| {
    if matches!(&t2.ty, TaskType::Request) {
      debug_assert_matches!(&t1.ty, TaskType::Start | TaskType::Transfer);
      Some(XEdgeConstraint::Loading(t1, t2))
    } else if matches!(&t1.ty, TaskType::Request) {
      debug_assert_matches!(&t2.ty, TaskType::End | TaskType::Transfer);
      Some(XEdgeConstraint::Unloading(t1, t2))
    } else {
      None
    }
  })
}
