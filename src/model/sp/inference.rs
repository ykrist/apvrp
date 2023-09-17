use crate::tasks::IdxTask::*;
use crate::utils::*;
use crate::*;

#[instrument(level = "info", skip(lu))]
pub fn build_inference_graph(lu: &Lookups) -> sawmill::InferenceModel<MpVar, SpConstr> {
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
