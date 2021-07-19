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

#[derive(Debug, Clone, Hash, Eq, PartialEq)]
pub struct PathIis {
  lb: PvTask,
  ub: PvTask,
  path: SmallVec<[Task; 10]>,
}

#[derive(Debug, Clone)]
pub enum Iis {
  /// Includes first task LB, last task UB, tasks must be in time-order
  Path(PathIis),
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

pub trait Subproblem<'a>: Sized {
  // Additional information obtained when solving the subproblem to optimality
  type Optimal;
  // Additional information obtained when proving the subproblem to be infeasible
  type Infeasible;
  // A group of constraint sets representing one or more IIS
  type IisConstraintSets: IntoIterator<Item=Iis>;

  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus<Self::Optimal, Self::Infeasible>>;
  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self, i: Self::Infeasible) -> Result<Self::IisConstraintSets>;
  // Find and remove one or more MRS when optimal
  fn add_optimality_cuts(&mut self, cb: &mut cb::Cb, o: Self::Optimal) -> Result<()>;

  fn solve_subproblem_and_add_cuts(&mut self, cb: &mut cb::Cb, estimate: Time) -> Result<()> {
    loop {
      match self.solve()? {
        SpStatus::Optimal(sp_obj, o) => {
          if estimate < sp_obj {
            self.add_optimality_cuts(cb, o)?;
          }
          break;
        }
        SpStatus::Infeasible(i) => {
          for iis in self.extract_and_remove_iis(i)?.into_iter() {
            trace!(?iis);
            let cut = match iis {
              Iis::Path(iis)=> {
                #[cfg(debug_assertions)] {
                  cb.infeasibilities.path_iis(&iis);
                }
                build_path_infeasiblity_cut(cb, &iis)
              }
              Iis::Cycle(cycle) => {
                #[cfg(debug_assertions)] {
                  cb.infeasibilities.av_cycle(&cycle);
                }
                build_cyclic_infeasiblity_cut(cb, &cycle)
              },
            };

            cb.enqueue_cut(cut, CutType::LpFeas); // FIXME cuttype should be changed
          }
        }
      }
    }
    Ok(())
  }
}

#[instrument(level="debug", skip(cb))]
pub fn build_path_infeasiblity_cut(cb: &cb::Cb, iis: &PathIis) -> IneqExpr {
  let PathIis { lb: lb_task, ub: ub_task, path } = iis;
  let n = path.len() - 1;
  debug_assert_eq!((lb_task.start, lb_task.end), (path[0].start, path[0].end));
  debug_assert_eq!((ub_task.start, ub_task.end), (path[n].start, path[n].end));
  debug_assert!(path.len() > 1);

  let mut lhs = Expr::default();
  let mut rhs = 1; // 2 bounds - 1

  cb.mp_vars.x_sum_similar_tasks(cb.lu, *lb_task).sum_into(&mut lhs);
  cb.mp_vars.x_sum_similar_tasks(cb.lu, *ub_task).sum_into(&mut lhs);

  let mut edge_nodes = path.as_slice();

  if matches!(lb_task.ty, TaskType::Transfer) && path[0].end == path[1].start {
    edge_nodes = &edge_nodes[1..];
  }
  if matches!(ub_task.ty, TaskType::Transfer) && path[n-1].end == path[n].start {
    edge_nodes = &edge_nodes[..edge_nodes.len()];
  }

  let mut x_edges = Set::default();

  for (t1, t2) in iter_pairs(edge_nodes) {
    let _s = trace_span!("edge_constraint", ?t1, ?t2).entered();
    match edge_constr_kind(t1, t2) {
      EdgeConstrKind::AvTravelTime => {
        rhs += 1;
        cb.mp_vars.y_sum_av(cb.lu, *t1, *t2).sum_into(&mut lhs);
        trace!("AV TT");
      },
      EdgeConstrKind::Loading => {
        x_edges.insert(*t1);
        trace!("Loading");
      },
      EdgeConstrKind::Unloading => {
        x_edges.insert(*t2);
        trace!("Unloading");
      }
    }
  }

  rhs += x_edges.len();
  x_edges.iter()
    .flat_map(|t| &cb.lu.tasks.task_to_pvtasks[t])
    .map(|t| cb.mp_vars.x[t])
    .sum_into(&mut lhs);


  trace!(lhs=?lhs.with_names(&cb.var_names), iis_size=path.len()-1, "generate cut");
  debug_assert!(rhs > 1);
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
