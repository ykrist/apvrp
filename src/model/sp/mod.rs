use self::dag::GraphModel;

use super::mp::MpVar;
use super::{cb, cb::CutType};
use crate::experiment::OptimalityCutKind;
use crate::logging::*;
use crate::model::{edge_constr_kind, EdgeConstrKind};
use crate::solution::MpSolution;
use crate::*;
use grb::constr::IneqExpr;
use grb::prelude::*;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;
use IdxTask::*;

use crate::model::cb::ThetaVals;
use crate::utils::{iter_pairs, PeekableExt, VarIterExt};
use daggylp::Weight;
use sawmill::cover::Cover;
use sawmill::lift::LiftedCover;
use sawmill::InferenceModel;

pub mod cuts;
pub mod dag;
mod inference;
pub use inference::{build_inference_graph, CoverLift};

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

impl SpConstr {
  #[track_caller]
  #[inline(always)]
  pub fn unwrap_delta(self) -> (IdxTask, IdxTask, Time) {
    match self {
      SpConstr::Delta(t1, t2, d) => (t1, t2, d),
      _ => panic!("unwrap on a non-Delta constraint"),
    }
  }

  /// For a constraint o.t.f `x + d <= y`, returns `Some(d)`, otherwise `None`.
  #[inline(always)]
  pub fn d(self) -> Option<Time> {
    match self {
      SpConstr::Delta(_, _, d) => Some(d),
      _ => None,
    }
  }
}

#[derive(Debug)]
pub enum SpStatus {
  Optimal(Time),
  Infeasible,
}

impl SpStatus {
  fn is_optimal(&self) -> bool {
    matches!(self, SpStatus::Optimal(..))
  }
}

#[derive(Debug, Clone)]
pub struct MrsPath {
  objective: Time,
  lower_bound: Time,
  /// n tasks (excluding the DDepot task), in order
  tasks: Vec<Task>,
  /// n edge constraints (including the task-to-DDepot edge)
  edge_constraints: Vec<SpConstr>,
}

// pub struct CriticalPathVisitor<'a, 'b> {
//   theta_estimate: &'a Map<(Avg, Task), Time>,
//   task_to_avg: Map<IdxTask, Avg>,
//   active_mp_vars: &'a Set<MpVar>,
//   cb: &'a mut cb::Cb<'b>,
// }

// impl<'a, 'b> CriticalPathVisitor<'a, 'b> {
//   fn new(
//     cb: &'a mut cb::Cb<'b>,
//     active_mp_vars: &'a Set<MpVar>,
//     theta: &'a Map<(Avg, Task), Time>,
//   ) -> Self {
//     let task_to_avg = active_mp_vars
//       .iter()
//       .filter_map(|v| match v {
//         MpVar::Y(a, t, td) if td == &IdxTask::DDepot => Some((*t, *a)),
//         _ => None,
//       })
//       .collect();
//     trace!(?theta);
//     CriticalPathVisitor {
//       theta_estimate: theta,
//       active_mp_vars,
//       task_to_avg,
//       cb,
//     }
//   }

//   #[inline(always)]
//   fn optimality_cut_required(&self, t: Task, true_obj: Time) -> bool {
//     let a = self.active_vehicle(&t.index());
//     trace!(a = a.0, ?t, true_obj);
//     self.theta_estimate.get(&(a, t)).copied().unwrap_or(0) < true_obj
//   }

//   #[inline(always)]
//   pub fn active_vehicle(&self, t: &IdxTask) -> Avg {
//     trace!(?t);
//     self.task_to_avg[t]
//   }

//   pub fn add_optimality_cut(&mut self, mrs_path: MrsPath) {
//     #[inline]
//     fn subpath_objective(lbs: &[Time], delta: &[Time]) -> Time {
//       debug_assert_eq!(lbs.len(), delta.len());
//       let mut time = lbs[0] + delta[0];
//       for (&lb, &d) in lbs[1..].iter().zip(&delta[1..]) {
//         time = std::cmp::max(time, lb) + d;
//       }
//       time
//     }

//     debug_assert_eq!(mrs_path.tasks.len(), mrs_path.edge_constraints.len());

//     let last_task = *mrs_path.tasks.last().unwrap();
//     let a = self.active_vehicle(&last_task.index());

//     let implied_lbs: Vec<_> = mrs_path.tasks.iter().map(|t| t.t_release).collect();
//     let edge_weights: Vec<_> = mrs_path
//       .edge_constraints
//       .iter()
//       .map(|c| {
//         let (_, _, d) = c.unwrap_delta();
//         d
//       })
//       .collect();

//     let mut constraint_set: Set<_> = mrs_path.edge_constraints.iter().copied().collect();
//     constraint_set.insert(SpConstr::Lb(
//       mrs_path.tasks[0].index(),
//       mrs_path.lower_bound,
//     ));

//     let cover = self
//       .cb
//       .inference_model
//       .cover(self.active_mp_vars, &constraint_set);

//     let mut lhs = Expr::default();

//     for (var, constrs) in cover {
//       let grb_var = self.cb.mp_vars.get_grb_var(self.cb.lu, &var);
//       if constrs.contains(mrs_path.edge_constraints.last().unwrap()) {
//         lhs += grb_var * mrs_path.objective;
//         continue;
//       }
//       let start_idx = match constrs
//         .iter()
//         .filter_map(|c| mrs_path.edge_constraints.iter().rposition(|x| c == x))
//         .max()
//       {
//         None => {
//           debug_assert_eq!(constrs.len(), 1);
//           0
//         }
//         Some(k) => k + 1,
//       };

//       let new_obj = subpath_objective(&implied_lbs[start_idx..], &edge_weights[start_idx..]);
//       let coeff = new_obj - mrs_path.objective;
//       trace!(?var, ?constrs, new_obj, coeff);
//       debug_assert!(new_obj <= mrs_path.objective);
//       lhs += (1 - grb_var) * coeff;
//     }

//     self.cb.enqueue_cut(
//       c!(lhs <= self.cb.mp_vars.theta[&(a, last_task)]),
//       CutType::LpOpt,
//     );
//   }
// }

pub trait GenIisCut {
  fn cut(cb: &mut cb::Cb, iis: &Iis) -> Result<()>;
}

pub trait GenOptimalityCut {
  fn cut(
    cb: &mut cb::Cb,
    subproblem: &mut GraphModel,
    active_mp_vars: &Set<MpVar>,
    theta_est: &ThetaVals,
  );
}

pub fn solve_subproblem_and_add_cuts(
  sp: &mut GraphModel,
  cb: &mut cb::Cb,
  active_mpvars: &Set<MpVar>,
  theta: &Map<(Avg, Task), Time>,
  estimate: Time,
  opt_cut: OptimalityCutKind,
) -> Result<Option<Map<(Avg, Task), Time>>> {
  let mut iis_covers = Vec::new();
  let mut n_constraint_counts = SmallVec::<[u8; 15]>::new();
  let mut subproblems_solved = 0u32;

  let sp_obj = loop {
    let _s = debug_span!("solve_loop", iter = subproblems_solved).entered();
    subproblems_solved += 1;

    match sp.solve() {
      SpStatus::Optimal(sp_obj) => {
        break sp_obj;
      }

      SpStatus::Infeasible => {
        let iis = sp.extract_and_remove_iis();
        trace!(?iis);
        #[cfg(debug_assertions)]
        {
          cb.infeasibilities.lp_iis(&iis);
        }
        n_constraint_counts.push(iis.constraints().len() as u8);
        let cover = cb.inference_model.cover(active_mpvars, iis.constraints());
        iis_covers.push((cover, iis.cut_type()));
      }
    }
  };

  // Feasibility cuts
  cb.stats.subproblem_cover_sizes(
    iis_covers
      .iter()
      .zip(n_constraint_counts)
      .map(|((c, _), n_constraints)| (c.len() as u8, n_constraints)),
  );

  iis_covers.sort_unstable_by_key(|(c, _)| c.len());

  for (cover, cut_ty) in iis_covers {
    let lifted_cover = cb.inference_model.lift_cover(&cover, CoverLift::new(cb.lu));
    cb.enqueue_cut(build_cut_from_lifted_cover(&cb, &lifted_cover), cut_ty);
  }

  // Optimality cuts
  if estimate < sp_obj {
    match opt_cut {
      OptimalityCutKind::CriticalPath => cuts::CriticalPathCut::cut(cb, sp, active_mpvars, theta),
      OptimalityCutKind::MrsPath => cuts::MrsPathCut::cut(cb, sp, active_mpvars, theta),
      OptimalityCutKind::MrsTree => cuts::MrsTreeCut::cut(cb, sp, active_mpvars, theta),
      OptimalityCutKind::Mrs => cuts::MrsCut::cut(cb, sp, active_mpvars, theta),
    }
  } else {
    trace!(
      subproblems_solved,
      estimate,
      sp_obj,
      "no optimality cut needed"
    );
  }

  if subproblems_solved == 1 {
    let new_theta = sp.calculate_theta()?;
    trace!(?new_theta, sp_obj);
    Ok(Some(new_theta))
  } else {
    Ok(None)
  }
}

pub fn build_cut_from_lifted_cover(
  cb: &cb::Cb,
  lifted_cover: &LiftedCover<MpVar, SpConstr>,
) -> IneqExpr {
  let rhs = lifted_cover.len() - 1;
  let mut lhs = Expr::default();
  for (vars, _) in lifted_cover {
    vars
      .iter()
      .map(|v| match v {
        &MpVar::X(p, t) => cb.mp_vars.x[&cb.lu.tasks.by_index_pv[&IdxPvTask::from((p, t))]],
        MpVar::Y(av, t1, t2) => {
          let t1 = cb.lu.tasks.by_index[t1];
          let t2 = cb.lu.tasks.by_index[t2];
          cb.mp_vars.y[&(*av, t1, t2)]
        }
      })
      .sum_into(&mut lhs)
  }
  trace!(lhs=?lhs.with_names(&cb.var_names), iis_size=lifted_cover.len(), "generate auto cut");
  c!(lhs <= rhs)
}

#[derive(Copy, Clone, Debug)]
enum XEdgeConstraint<'a> {
  Loading(&'a PvTask, &'a PvTask),
  Unloading(&'a PvTask, &'a PvTask),
}

fn iter_edge_constraints<'a>(
  pv_route: &'a [PvTask],
) -> impl Iterator<Item = XEdgeConstraint<'a>> + 'a {
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
  fn iter_sp_vars<'a>(&'a self) -> impl Iterator<Item = &'a PvTask> + 'a {
    self
      .pv_cycles
      .iter()
      .flat_map(|c| c.unique_tasks())
      .chain(self.pv_routes.iter().flat_map(|r| r.iter()))
  }

  fn iter_sp_x_edges<'a>(&'a self) -> impl Iterator<Item = XEdgeConstraint> + 'a {
    // Need to pass closures to flat_map for deref coercion to work
    self
      .pv_routes
      .iter()
      .flat_map(|r| iter_edge_constraints(r))
      .chain(self.pv_cycles.iter().flat_map(|c| iter_edge_constraints(c)))
  }

  fn iter_sp_y_edges<'a>(&'a self) -> impl Iterator<Item = (&'a Task, &'a Task)> + 'a {
    self
      .av_routes
      .iter()
      .flat_map(|r| iter_pairs(r.without_depots()))
      .chain(self.av_cycles.iter().flat_map(|c| c.iter_edges()))
  }
}

impl Lookups {
  #[inline(always)]
  pub fn av_task_travel_time(&self, t1: &Task, t2: &Task) -> Time {
    t1.tt + self.data.travel_time[&(t1.end, t2.start)]
  }
}
