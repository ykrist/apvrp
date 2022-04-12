use super::*;
use std::cell::Ref;

pub struct Ctx<'a, A, C> {
  model: &'a InferenceModel<A, C>,
  cover: &'a cover::Cover<A, C>,
  current_cover_component: Option<(A, Set<C>)>,
  siblings: Set<A>,
  lifted_cover: Vec<(Set<A>, Set<C>)>,
}


pub type LiftedCover<A, C> = Vec<(Set<A>, Set<C>)>;

impl<A: Clause, C: Constraint> InferenceModel<A, C> {
  pub fn lift_cover(&self, cover: &cover::Cover<A, C>, mut alg: impl Lift<A, C>) -> LiftedCover<A, C> {
    let mut ctx = Ctx {
      model: self,
      cover,
      current_cover_component: None,
      siblings: Set::default(),
      lifted_cover: Vec::default(),
    };

    for (a, cons) in cover.iter() {
      ctx.init_component(a.clone(), cons.clone());
      alg.visit_candidate_siblings(&mut ctx, a, cons);
      ctx.finish_component();
    }

    ctx.lifted_cover
  }
}

impl<'a, A: Clause, C: Constraint> Ctx<'a, A, C> {
  fn init_component(&mut self, current_clause: A, covered: Set<C>) {
    self.current_cover_component = Some((current_clause, covered));
  }

  fn finish_component(&mut self) {
    let (a, cons) = self.current_cover_component.take().unwrap();
    let mut clauses = std::mem::replace(&mut self.siblings, Set::default());
    clauses.insert(a);
    self.lifted_cover.push((clauses, cons));
  }

  /// Declares that `sibling` is a sibling
  pub fn add_checked(&mut self, sibling: A) {
    let sibling_imp = match self.model.implications.get(&sibling) {
      Some(imp) => imp,
      None => return,
    };
    let a = &self.current_cover_component.as_ref().unwrap().0;
    for c in &self.cover[a] {
      if !sibling_imp.contains(c) { return; }
    }
    self.add_unchecked(sibling)
  }

  pub fn add_unchecked(&mut self, sibling: A) {
    self.siblings.insert(sibling);
  }
}


pub trait Lift<A, C> {
  /// Given a clause, this method should call a `ctx.add_*()` method for each clause which is mutually exclusive with `a`.
  fn visit_candidate_siblings(&mut self, ctx: &mut Ctx<A, C>, a: &A, cons: &Set<C>);
}

impl<A, C, T> Lift<A, C> for &mut T
  where T: Lift<A, C>
{
  #[inline(always)]
  fn visit_candidate_siblings(&mut self, ctx: &mut Ctx<A, C>, a: &A, cons: &Set<C>) {
    T::visit_candidate_siblings(self, ctx, a, cons)
  }
}