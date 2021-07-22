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
                  assert_ne!(cycle.first(), cycle.last());
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

#[cfg(test)]
mod tests {
  use super::*;
  use std::path::Path;
  use crate::experiment::ApvrpExp;
  use slurm_harray::Experiment;
  use crate::solution::iter_solution_log;
  use crate::utils::IoContext;
  use anyhow::Context;
  use crate::test::*;

  fn compare_one(lu: &Lookups, sol: &Solution) -> Result<()> {
    let dummy_theta = Default::default();
    let mut dagmodel = dag::GraphModel::build(lu, sol, &dummy_theta)?;
    let mut lpmodel = lp::TimingSubproblem::build(lu, sol)?;

    let dag_result = dagmodel.solve()?;
    let lp_result = lpmodel.solve()?;

    match (dag_result, lp_result) {
      (SpStatus::Infeasible(_), SpStatus::Infeasible(_)) => {},
      (SpStatus::Optimal(dag_obj, _), SpStatus::Optimal(lp_obj,_)) => {
        if dag_obj != lp_obj {
          anyhow::bail!("objective mismatch: DAG = {} != {} = LP", dag_obj, lp_obj)
        }
      },
      (dag_result, lp_result) => {
        if !lp_result.is_optimal() {
          lpmodel.debug_infeasibility()?;
        }
        let output = test_output_dir().join("LP.txt");
        lpmodel.write_debug_graph(&output)?;
        dagmodel.model.write_debug( test_output_dir().join("DAG.txt"))?;
        anyhow::bail!("status mismatch:\nDAG = {:?}\n LP = {:?}", dag_result, lp_result)
      }
    }
    Ok(())
  }

  fn compare_for_instance(index_file: impl AsRef<Path>) -> Result<()> {
    let exp = ApvrpExp::from_index_file(&index_file).read_context(&index_file)?;
    println!("{:?}", &exp.inputs);
    let lu = Lookups::load_data_and_build(exp.inputs.index)?;
    let sol_log = index_file.as_ref().with_file_name(&exp.outputs.solution_log);
    for (k, solution) in iter_solution_log(&sol_log)?.enumerate() {
      let sol = solution.to_solution(&lu);
      compare_one(&lu, &sol).with_context(|| format!("Failed for solution #{} of {:?}", k, &sol_log))?
    }
    Ok(())
  }

  #[test]
  fn compare_subproblem_methods() -> Result<()> {
    let _g = crate::logging::init_test_logging(None::<&str>);
    let mut patt = crate::test::test_data_dir().join("subproblems").into_os_string().into_string().unwrap();
    patt.push_str("/*index.json");
    for p in glob::glob(&patt)? {
      compare_for_instance(p?)?
    }
    Ok(())
  }

}