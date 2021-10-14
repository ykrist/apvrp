use crate::*;
use crate::solution::MpSolution;
use super::{cb, cb::CutType};
use smallvec::SmallVec;
use grb::constr::IneqExpr;
use grb::prelude::*;
use crate::logging::*;
use crate::model::{EdgeConstrKind, edge_constr_kind};
use super::mp::MpVar;
use serde::{Deserialize, Serialize};
use IdxTask::*;

use crate::utils::{iter_pairs, VarIterExt, PeekableExt};
use daggylp::Weight;
use sawmill::lift::LiftedCover;

pub mod lp;
pub mod dag;

#[derive(Debug, Clone)]
pub enum Iis {
  /// Includes first task LB, last task UB, tasks must be in time-order
  Path(Set<SpConstr>),
  /// Tasks should be ordered according to the cycle.  First task should NOT be the same as the last task.
  Cycle(Set<SpConstr>),
}

impl Iis {
  pub fn constraints<'a>(&'a self) -> &'a Set<SpConstr> {
    match self {
      Iis::Cycle(s) | Iis::Path(s) => s,
    }
  }

  pub fn cut_type(&self) -> CutType {
    match self {
      Iis::Cycle(_) => CutType::LpCycle,
      Iis::Path(_) => CutType::LpPath,
    }
  }
}


#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Serialize, Deserialize, PartialOrd, Ord)]
pub enum SpConstr {
  Lb(IdxTask, Time),
  Ub(IdxTask, Time),
  Delta(IdxTask, IdxTask, Time),
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

  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus<Self::Optimal, Self::Infeasible>>;
  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self, i: Self::Infeasible) -> Result<Iis>;
  // Find and remove one or more MRS when optimal
  fn add_optimality_cuts(&mut self, cb: &mut cb::Cb, theta: &Map<(Avg, Task), Time>, o: Self::Optimal) -> Result<()>;

  fn solve_subproblem_and_add_cuts(&mut self, cb: &mut cb::Cb, active_mpvars: &Set<MpVar>, theta: &Map<(Avg, Task), Time>, estimate: Time) -> Result<Option<Time>> {
    // TODO: stop early once the IIS covers start getting too big
    //  Can use cb.params, will need find the size of the cover before constructing the constraint

    for solve in 0.. {
      match self.solve()? {
        SpStatus::Optimal(sp_obj, o) => {
          if estimate < sp_obj {
            self.add_optimality_cuts(cb, theta, o)?;
          }
          if solve == 0 {
            return Ok(Some(sp_obj));
          } else {
            return Ok(None);
          }
        }

        SpStatus::Infeasible(i) => {
          let iis = self.extract_and_remove_iis(i)?;
          trace!(?iis);
          #[cfg(debug_assertions)] {
            cb.infeasibilities.lp_iis(&iis);
          }
          let cover = cb.inference_model.cover(active_mpvars, iis.constraints());
          let lifted_cover = cb.inference_model.lift_cover(&cover, CoverLift{ lookups: cb.lu });
          cb.enqueue_cut(build_cut_from_lifted_cover(&cb, &lifted_cover), iis.cut_type());
        }
      }
    }

    unreachable!()
  }
}

pub fn build_cut_from_lifted_cover(cb: &cb::Cb, lifted_cover: &LiftedCover<MpVar, SpConstr>) -> IneqExpr {
  let rhs = lifted_cover.len() - 1;
  let mut lhs = Expr::default();
  for (vars, _) in lifted_cover {
    vars.iter().map(|v| match v {
      &MpVar::X(p, t) => cb.mp_vars.x[&cb.lu.tasks.by_index_pv[&IdxPvTask::from((p, t))]],
      MpVar::Y(av, t1, t2) => {
        let t1 = cb.lu.tasks.by_index[t1];
        let t2 = cb.lu.tasks.by_index[t2];
        cb.mp_vars.y[&(*av, t1, t2)]
      },
    })
      .sum_into(&mut lhs)
  }
  trace!(lhs=?lhs.with_names(&cb.var_names), iis_size=lifted_cover.len(), "generate auto cut");
  c!( lhs <= rhs)
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

impl MpSolution {
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


#[instrument(level = "info", skip(lu))]
pub fn build_inference_graph(lu: &Lookups) -> sawmill::InferenceModel<MpVar, SpConstr> {
  let mut model = sawmill::InferenceModel::<MpVar, SpConstr>::build();
  let mut included_tasks = Set::default();

  for t in lu.tasks.pvtask_to_task.keys() {
    let s = t.index().into();
    let p = t.p;
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

    model.add_implication(x, SpConstr::Lb(s, t.t_release));
    model.add_implication(x, SpConstr::Ub(s, t.t_deadline - t.tt)); // FIXME bug?

    match s {
      Transfer(r, _) | End(_, r) => {
        model.add_implication(x, SpConstr::Delta(
          Request(r),
          s,
          lu.data.travel_time[&(Loc::ReqP(r), Loc::ReqD(r))] + lu.data.srv_time[&Loc::ReqD(r)],
        ));
      }
      _ => {}
    }

    match s {
      Transfer(_, r) | Start(_, r) => {
        model.add_implication(x, SpConstr::Delta(
          s,
          Request(r),
          t.tt + lu.data.srv_time[&Loc::ReqP(r)],
        ));
      }
      _ => {}
    }
    included_tasks.insert(s);
  }
  info!("finished X-variable implications");
  for (&av, av_tasks) in &lu.tasks.compat_with_av {
    for &task1 in av_tasks {
      let t1 = task1.index();
      for &task2 in &lu.tasks.succ[&task1] {
        let t2 = task2.index();
        if av_tasks.contains(&task2) && included_tasks.contains(&t1) && included_tasks.contains(&t2) {
          let y = MpVar::Y(av, t1, t2);
          model.add_implication(y, SpConstr::Delta(
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
  info!("finished Y-variable implications");
  let constraints = model.constraints().clone();
  let mut groups: Map<_, Vec<_>> = Map::default();

  #[derive(Copy, Clone, Debug, Hash, Eq, PartialEq)]
  enum ConstraintGroup {
    Lb(IdxTask),
    Ub(IdxTask),
    Delta(IdxTask, IdxTask),
  }

  for &c in &constraints {
    let (g, time) = match c {
      SpConstr::Ub(t, v) => (ConstraintGroup::Ub(t), v),
      SpConstr::Lb(t, v) => (ConstraintGroup::Lb(t), v),
      SpConstr::Delta(t1, t2, v) => (ConstraintGroup::Delta(t1, t2), v),
    };
    groups.entry(g).or_default().push((time, c));
  }


  info!(total_subproblem_constraints = constraints.len());
  for (group, mut elems) in groups {
    let _s = trace_span!("constr_dom", ?group).entered();

    match group {
      // Bigger time is stronger
      ConstraintGroup::Lb(_) | ConstraintGroup::Delta(..) => {
        elems.sort_unstable_by_key(|e| -e.0);
      }
      // Smaller time is stronger
      ConstraintGroup::Ub(_) => {
        elems.sort_unstable_by_key(|e| e.0);
      }
    }

    for ((_, stronger), (_, weaker)) in iter_pairs(&elems) {
      trace!(?stronger, ?weaker, "dominate");
      model.add_constraint_domination(*stronger, *weaker);
    }
  }

  info!("finished subproblem constraint domination");
  model.finish()
}


pub struct CoverLift<'a> {
  lookups: &'a Lookups,
}

impl<'a> CoverLift<'a> {
  pub fn new(lookups: &'a Lookups) -> Self {
    CoverLift { lookups }
  }
}

impl sawmill::lift::Lift<MpVar, SpConstr> for CoverLift<'_> {
  fn visit_candidate_siblings(&mut self, ctx: &mut sawmill::lift::Ctx<MpVar, SpConstr>, var: &MpVar, _cons: &Set<SpConstr>) {
    match var {
      &MpVar::X(p, t) => {
        for q in self.lookups.sets.pvs() {
          if p != q {
            ctx.add_checked(MpVar::X(q, t));
          }
        }
      }
      &MpVar::Y(a, t1, t2) => {
        for b in self.lookups.sets.avs() {
          if a != b {
            ctx.add_checked(MpVar::Y(b, t1, t2));
          }
        }
      }
    }
  }
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

  fn compare_one(lu: &Lookups, sol: &MpSolution) -> Result<()> {
    let dummy_theta = Default::default();
    let mut dagmodel = dag::GraphModel::build(lu, sol)?;
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

