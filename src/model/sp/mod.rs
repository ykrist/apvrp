use crate::*;
use crate::solution::Solution;
use super::{cb, cb::CutType};
use smallvec::SmallVec;
use grb::constr::IneqExpr;
use grb::prelude::*;
use crate::logging::*;
use crate::model::{EdgeConstrKind, edge_constr_kind};

use crate::utils::{iter_pairs, VarIterExt, PeekableExt};
use daggylp::Weight;

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

  fn solve_subproblem_and_add_cuts(&mut self, cb: &mut cb::Cb, estimate: Time) -> Result<Option<Time>> {
    // TODO: stop early once the IIS covers start getting too big
    //  Can use cb.params, will need find the size of the cover before constructing the constraint

    for solve in 0.. {
      match self.solve()? {
        SpStatus::Optimal(sp_obj, o) => {
          if estimate < sp_obj {
            self.add_optimality_cuts(cb, o)?;
          }
          if solve == 0 {
            return Ok(Some(sp_obj))
          } else {
            return Ok(None)
          }
        }

        SpStatus::Infeasible(i) => {
          for iis in self.extract_and_remove_iis(i)?.into_iter() {
            trace!(?iis);
            match iis {
              Iis::Path(iis) => {
                #[cfg(debug_assertions)] {
                  cb.infeasibilities.path_iis(&iis);
                }
                cb.enqueue_cut(build_path_infeasiblity_cut(cb, &iis), CutType::LpPath);
              }
              Iis::Cycle(cycle) => {
                #[cfg(debug_assertions)] {
                  assert_ne!(cycle.first(), cycle.last());
                  cb.infeasibilities.av_cycle(&cycle);
                }
                cb.enqueue_cut(build_cyclic_infeasiblity_cut(cb, &cycle), CutType::LpCycle);
              }
            }
          }
        }
      }
    }

    unreachable!()
  }
}

#[instrument(level = "debug", skip(cb))]
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

  if matches!(lb_task.ty, TaskType::Transfer| TaskType::Start) && path[0].end == path[1].start {
    edge_nodes = &edge_nodes[1..];
    trace!(?edge_nodes, "LB task responsible for first edge");
  }

  if matches!(ub_task.ty, TaskType::Transfer | TaskType::End) && path[n - 1].end == path[n].start {
    edge_nodes = &edge_nodes[..edge_nodes.len()-1];
    trace!(?edge_nodes, "UB task responsible for last edge");
  }

  let mut x_edges = Set::default();

  for (t1, t2) in iter_pairs(edge_nodes) {
    let _s = trace_span!("edge_constraint", ?t1, ?t2).entered();
    match edge_constr_kind(t1, t2) {
      EdgeConstrKind::AvTravelTime => {
        rhs += 1;
        cb.mp_vars.y_sum_av(cb.lu, *t1, *t2).sum_into(&mut lhs);
        trace!("AV TT");
      }
      EdgeConstrKind::Loading => {
        x_edges.insert(*t1);
        trace!("Loading");
      }
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


  trace!(lhs=?lhs.with_names(&cb.var_names), iis_size=path.len()+1, "generate cut");
  c!( lhs <= rhs)
}

pub fn build_cyclic_infeasiblity_cut(cb: &cb::Cb, cycle: &[Task]) -> IneqExpr {
  debug_assert_ne!(cycle.first(), cycle.last());
  debug_assert!(cycle.len() > 1, "cycle too short: {:?}", cycle);

  let mut x_edges = set_with_capacity(cycle.len());
  let mut rhs = 0u32;

  let mut lhs = Expr::default();

  for (t1, t2) in iter_cycle(cycle) {
    match edge_constr_kind(t1, t2) {
      EdgeConstrKind::AvTravelTime => {
        rhs += 1;
        cb.mp_vars.y_sum_av(cb.lu, *t1, *t2).sum_into(&mut lhs);
        trace!("AV TT");
      }
      EdgeConstrKind::Loading => {
        x_edges.insert(*t1);
        trace!("Loading");
      }
      EdgeConstrKind::Unloading => {
        x_edges.insert(*t2);
        trace!("Unloading");
      }
    }
  }

  rhs += x_edges.len() as u32;
  rhs -= 1;

  for t in x_edges {
    cb.mp_vars.x_sum_all_task(cb.lu, t).sum_into(&mut lhs);
  }

  c!(lhs <= rhs)
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

impl Solution {
  fn iter_sp_vars<'a>(&'a self) -> impl Iterator<Item=&'a PvTask> + 'a {
    self.pv_cycles.iter().flat_map(|c| c.unique_tasks())
      .chain(self.pv_routes.iter().flat_map(|r| r.iter()))
  }

  fn iter_sp_x_edges<'a>(&'a self) -> impl Iterator<Item=XEdgeConstraint> + 'a {
    // Need to pass closures to flat_map for deref coercion to work
    self.pv_routes.iter()
      .flat_map(|r| iter_edge_constraints(r))
      .chain(self.pv_cycles.iter().flat_map(|c| iter_edge_constraints(c)))
  }

  fn iter_sp_y_edges<'a>(&'a self) -> impl Iterator<Item=(&'a Task, &'a Task)> + 'a {
    self.av_routes.iter().flat_map(|r| iter_pairs(r.without_depots()))
      .chain(self.av_cycles.iter().flat_map(|c| c.iter_edges()))
  }
}

use ShorthandTask::*;

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
pub enum Constraint {
  Lb(ShorthandTask, Time),
  Ub(ShorthandTask, Time),
  Delta(ShorthandTask, ShorthandTask, Time),
}

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
pub enum MpVar {
  X(Pv, ShorthandTask),
  Y(Av, ShorthandTask, ShorthandTask),
}

pub fn build_inference_graph(lu: &Lookups) -> sawmill::InferenceModel<MpVar, Constraint> {
  let mut model = sawmill::InferenceModel::<MpVar, Constraint>::build();
  let mut included_tasks = Set::default();

  for (pt, t) in lu.tasks.pvtask_to_task.iter() {
    let s = t.shorthand();
    let p = pt.p;
    let x = MpVar::X(p, s);

    match s {
      Transfer(r1, r2) => {
        model.add_domain_implication(x, MpVar::X(p, Request(r1)));
        model.add_domain_implication(x, MpVar::X(p, Request(r2)));
      }
      Direct(_) | Request(_) => {}
      End(p, r) | Start(p, r) => {
        model.add_domain_implication(x, MpVar::X(p, Request(r)))
      }
      ODepot | DDepot => unreachable!(),
    }

    model.add_implication(x, Constraint::Lb(s, t.t_release));
    model.add_implication(x, Constraint::Ub(s, t.t_deadline));

    match s {
      Transfer(r, _) | End(_, r) => {
        model.add_implication(x, Constraint::Delta(
          Request(r),
          s,
          lu.data.travel_time[&(Loc::ReqP(r), Loc::ReqD(r))] + lu.data.srv_time[&Loc::ReqD(r)],
        ));
      }
      _ => {}
    }

    match s {
      Transfer(_, r) | Start(_, r) => {
        model.add_implication(x, Constraint::Delta(
          s,
          Request(r),
          t.tt + lu.data.srv_time[&Loc::ReqP(r)],
        ));
      }
      _ => {}
    }
    included_tasks.insert(s);
  }

  for (&av, av_tasks) in &lu.tasks.compat_with_av {
    for &task1 in av_tasks {
      let t1 = task1.shorthand();
      for &task2 in &lu.tasks.succ[&task1] {
        let t2 = task2.shorthand();
        if av_tasks.contains(&task2) && included_tasks.contains(&t1) && included_tasks.contains(&t2) {
          let y = MpVar::Y(av, t1, t2);
          model.add_implication(y, Constraint::Delta(
            t1,
            t2,
            lu.data.travel_time[&(task1.end, task2.start)],
          ));

          let mut p1 = task1.infer_pv();
          let mut p2 = task2.infer_pv();

          if task1.end == task2.start {
            p1 = p1.or(p2);
            p2 = p2.or(p1);
            assert_eq!(p1, p2)
          }

          if let Some(p1) = p1 {
            model.add_domain_implication(y, MpVar::X(p1, t1))
          }
          if let Some(p2) = p2 {
            model.add_domain_implication(y, MpVar::X(p2, t2))
          }
        }
      }
    }
  }

  let constraints = model.constraints().clone();
  for &c1 in &constraints {
    for &c2 in &constraints {
      if c1 != c2 {
        match (c1, c2) {
          (Constraint::Lb(t1, lb1), Constraint::Lb(t2, lb2))
          if t1 == t2 && lb1 >= lb2 => {
            model.add_constraint_domination(c1, c2);
          },
          (Constraint::Ub(t1, ub1), Constraint::Ub(t2, ub2))
          if t1 == t2 && ub1 <= ub2 => {
            model.add_constraint_domination(c1, c2);
          },
          (Constraint::Delta(t1, s1, d1), Constraint::Delta(t2, s2, d2))
          if t1 == t2 && s1 == s2 && d1 >= d2 => {
            model.add_constraint_domination(c1, c2)
          }
          _ => {},
        }
      }
    }
  }

  model.finish()
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
      (SpStatus::Infeasible(_), SpStatus::Infeasible(_)) => {}
      (SpStatus::Optimal(dag_obj, _), SpStatus::Optimal(lp_obj, _)) => {
        if dag_obj != lp_obj {
          anyhow::bail!("objective mismatch: DAG = {} != {} = LP", dag_obj, lp_obj)
        }
      }
      (dag_result, lp_result) => {
        if !lp_result.is_optimal() {
          lpmodel.debug_infeasibility()?;
        }
        let output = test_output_dir().join("LP.txt");
        lpmodel.write_debug_graph(&output)?;
        dagmodel.model.write_debug(test_output_dir().join("DAG.txt"))?;
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
    crate::logging::init_test_logging();
    let mut patt = crate::test::test_data_dir().join("subproblems").into_os_string().into_string().unwrap();
    patt.push_str("/*index.json");
    for p in glob::glob(&patt)? {
      compare_for_instance(p?)?
    }
    Ok(())
  }
}

