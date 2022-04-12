use self::dag::GraphModel;

use super::mp::MpVar;
use super::{cb, cb::CutType};
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

pub mod dag;
pub mod cuts;

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

pub struct CriticalPathVisitor<'a, 'b> {
  theta_estimate: &'a Map<(Avg, Task), Time>,
  task_to_avg: Map<IdxTask, Avg>,
  active_mp_vars: &'a Set<MpVar>,
  cb: &'a mut cb::Cb<'b>,
}

impl<'a, 'b> CriticalPathVisitor<'a, 'b> {
  fn new(
    cb: &'a mut cb::Cb<'b>,
    active_mp_vars: &'a Set<MpVar>,
    theta: &'a Map<(Avg, Task), Time>,
  ) -> Self {
    let task_to_avg = active_mp_vars
      .iter()
      .filter_map(|v| match v {
        MpVar::Y(a, t, td) if td == &IdxTask::DDepot => Some((*t, *a)),
        _ => None,
      })
      .collect();
    trace!(?theta);
    CriticalPathVisitor {
      theta_estimate: theta,
      active_mp_vars,
      task_to_avg,
      cb,
    }
  }

  #[inline(always)]
  fn optimality_cut_required(&self, t: Task, true_obj: Time) -> bool {
    let a = self.active_vehicle(&t.index());
    trace!(a = a.0, ?t, true_obj);
    self.theta_estimate.get(&(a, t)).copied().unwrap_or(0) < true_obj
  }

  #[inline(always)]
  pub fn active_vehicle(&self, t: &IdxTask) -> Avg {
    trace!(?t);
    self.task_to_avg[t]
  }

  pub fn add_optimality_cut(&mut self, mrs_path: MrsPath) {
    #[inline]
    fn subpath_objective(lbs: &[Time], delta: &[Time]) -> Time {
      debug_assert_eq!(lbs.len(), delta.len());
      let mut time = lbs[0] + delta[0];
      for (&lb, &d) in lbs[1..].iter().zip(&delta[1..]) {
        time = std::cmp::max(time, lb) + d;
      }
      time
    }

    debug_assert_eq!(mrs_path.tasks.len(), mrs_path.edge_constraints.len());

    let last_task = *mrs_path.tasks.last().unwrap();
    let a = self.active_vehicle(&last_task.index());

    let implied_lbs: Vec<_> = mrs_path.tasks.iter().map(|t| t.t_release).collect();
    let edge_weights: Vec<_> = mrs_path
      .edge_constraints
      .iter()
      .map(|c| {
        let (_, _, d) = c.unwrap_delta();
        d
      })
      .collect();

    let mut constraint_set: Set<_> = mrs_path.edge_constraints.iter().copied().collect();
    constraint_set.insert(SpConstr::Lb(
      mrs_path.tasks[0].index(),
      mrs_path.lower_bound,
    ));

    let cover = self
      .cb
      .inference_model
      .cover(self.active_mp_vars, &constraint_set);

    let mut lhs = Expr::default();

    for (var, constrs) in cover {
      let grb_var = self.cb.mp_vars.get_grb_var(self.cb.lu, &var);
      if constrs.contains(mrs_path.edge_constraints.last().unwrap()) {
        lhs += grb_var * mrs_path.objective;
        continue;
      }
      let start_idx = match constrs
        .iter()
        .filter_map(|c| mrs_path.edge_constraints.iter().rposition(|x| c == x))
        .max()
      {
        None => {
          debug_assert_eq!(constrs.len(), 1);
          0
        }
        Some(k) => k + 1,
      };

      let new_obj = subpath_objective(&implied_lbs[start_idx..], &edge_weights[start_idx..]);
      let coeff = new_obj - mrs_path.objective;
      trace!(?var, ?constrs, new_obj, coeff);
      debug_assert!(new_obj <= mrs_path.objective);
      lhs += (1 - grb_var) * coeff;
    }

    self.cb.enqueue_cut(
      c!(lhs <= self.cb.mp_vars.theta[&(a, last_task)]),
      CutType::LpOpt,
    );
  }
}

pub trait GenIisCut {
  fn cut(cb: &mut cb::Cb, iis: &Iis) -> Result<()>;
}

pub trait GenOptimalityCut {
  fn cut(cb: &mut cb::Cb, subproblem: &mut GraphModel, constr: &Set<SpConstr>) -> Result<()>;
}


pub trait Subproblem<'a>: Sized {
  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus>;

  fn calculate_theta(&mut self) -> Result<ThetaVals>;

  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self) -> Result<Iis>;

  // Find the MRS paths
  fn visit_critical_paths(&mut self, visitor: &mut CriticalPathVisitor) -> Result<()>;

  fn solve_subproblem_and_add_cuts(
    &mut self,
    cb: &mut cb::Cb,
    active_mpvars: &Set<MpVar>,
    theta: &Map<(Avg, Task), Time>,
    estimate: Time,
  ) -> Result<Option<Map<(Avg, Task), Time>>> {
    // TODO: stop early once the IIS covers start getting too big
    //  Can use cb.params, will need find the size of the cover before constructing the constraint

    let mut iis_covers = Vec::new();
    let mut n_constraint_counts = SmallVec::<[u8; 15]>::new();
    let mut subproblems_solved = 0u32;

    let sp_obj = loop {
      let _s = debug_span!("solve_loop", iter = subproblems_solved).entered();
      subproblems_solved += 1;

      match self.solve()? {
        SpStatus::Optimal(sp_obj) => {
          break sp_obj;
        }

        SpStatus::Infeasible => {
          let iis = self.extract_and_remove_iis()?;
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
      let lifted_cover = cb
        .inference_model
        .lift_cover(&cover, CoverLift { lookups: cb.lu });
      cb.enqueue_cut(build_cut_from_lifted_cover(&cb, &lifted_cover), cut_ty);
    }

    // Optimality cuts
    if estimate < sp_obj {
      trace!(?active_mpvars);
      let mut visitor = CriticalPathVisitor::new(cb, active_mpvars, theta);
      self.visit_critical_paths(&mut visitor)?;
    }

    if subproblems_solved == 1 {
      let new_theta = self.calculate_theta()?;
      trace!(?new_theta, sp_obj);
      Ok(Some(new_theta))
    } else {
      Ok(None)
    }
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

#[instrument(level = "info", skip(lu))]
pub fn build_inference_graph(lu: &Lookups) -> sawmill::InferenceModel<MpVar, SpConstr> {
  // TODO move this to a new submodule called "inference"
  // TODO use the non-Pv task deadlines/release times
  let mut model = sawmill::InferenceModel::<MpVar, SpConstr>::build();

  for pv_task in lu.tasks.pvtask_to_task.keys() {
    let t = pv_task.index().into();
    let p = pv_task.p;
    let x = MpVar::X(p, t);

    match t {
      Transfer(r1, r2) => {
        model.add_domain_implication(x, MpVar::X(p, Request(r1)));
        model.add_domain_implication(x, MpVar::X(p, Request(r2)));
      }
      Direct(_) | Request(_) => {}
      End(p, r) | Start(p, r) => model.add_domain_implication(x, MpVar::X(p, Request(r))),
      ODepot | DDepot => unreachable!(),
    }

    model.add_implication(x, SpConstr::Lb(t, pv_task.subproblem_lb()));
    model.add_implication(x, SpConstr::Ub(t, pv_task.subproblem_ub()));

    match t {
      Transfer(r, _) | End(_, r) => {
        model.add_implication(
          x,
          SpConstr::Delta(
            Request(r),
            t,
            lu.data.travel_time[&(Loc::ReqP(r), Loc::ReqD(r))] + lu.data.srv_time[&Loc::ReqD(r)],
          ),
        );
      }
      _ => {}
    }

    match t {
      Transfer(_, r) | Start(_, r) => {
        model.add_implication(
          x,
          SpConstr::Delta(t, Request(r), pv_task.tt + lu.data.srv_time[&Loc::ReqP(r)]),
        );
      }
      _ => {}
    }
  }

  info!("finished X-variable implications");

  for (av, task1, task2) in lu.iter_yvars() {
    if task1.is_depot() {
      continue;
    }

    let (t1, t2) = (task1.index(), task2.index());
    let y = MpVar::Y(av, t1, t2);
    model.add_implication(
      y,
      SpConstr::Delta(t1, t2, lu.av_task_travel_time(&task1, &task2)),
    );

    let mut p1 = task1.infer_pv();
    let mut p2 = task2.infer_pv();

    if task1.end == task2.start {
      p1 = p1.or(p2);
      p2 = p2.or(p1);
      debug_assert_eq!(p1, p2)
    }

    if let Some(p1) = p1 {
      model.add_domain_implication(y, MpVar::X(p1, t1))
    }
    if let Some(p2) = p2 {
      model.add_domain_implication(y, MpVar::X(p2, t2))
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

  let mut model = model.add_higher_order();

  for (av, t1, t2) in lu.iter_yvars() {
    let t1 = t1.index();
    let t2 = t2.index();
    let y_var = MpVar::Y(av, t1, t2);
    for t in [t1, t2] {
      if !t.is_depot() && t.infer_pv().is_none() {
        // only need to bother with tasks where P can't be inferred.
        let x_vars = lu.tasks.task_to_pvtasks[&lu.tasks.by_index[&t]]
          .iter()
          .map(|pt| MpVar::X(pt.p, t));
        model.add_implies_one_of(y_var, x_vars);
      }
    }
  }
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
  fn visit_candidate_siblings(
    &mut self,
    ctx: &mut sawmill::lift::Ctx<MpVar, SpConstr>,
    var: &MpVar,
    _cons: &Set<SpConstr>,
  ) {
    match var {
      &MpVar::X(p, t) => {
        for q in self.lookups.sets.pvs() {
          if p != q {
            ctx.add_checked(MpVar::X(q, t));
          }
        }
      }
      &MpVar::Y(a, t1, t2) => {
        for b in self.lookups.sets.av_groups() {
          if a != b {
            ctx.add_checked(MpVar::Y(b, t1, t2));
          }
        }
      }
    }
  }
}
