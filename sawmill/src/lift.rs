use super::*;
use std::cell::Ref;

pub struct Ctx<'a, A, C> {
  model: &'a InferenceModel<A, C>,
  cover: &'a cover::Cover<A, C>,
  current_cover_component: Set<A>,
  lifted_cover: Vec<(Set<A>, Set<C>)>,
}


impl<A: Clause, C: Constraint> InferenceModel<A, C> {
  pub fn lift(&self, cover: &cover::Cover<A, C>, alg: ()) -> Vec<(Set<A>, Set<C>)> {
    todo!()
  }
}

pub type LiftedCover<A, C> = Vec<(Set<A>, Set<C>)>;


impl<A: Clause, C: Constraint> InferenceModel<A, C> {
  fn lift_cover(&self, cover: &cover::Cover<A, C>, alg: &mut impl Lift<A, C>) -> LiftedCover<A, C> {
    let mut ctx = Ctx {
      model: self,
      cover,
      current_cover_component: Set::default(),
      lifted_cover: Vec::default(),
    };

    for (a, cons) in cover.iter() {
      alg.visit_candidate_siblings(&mut ctx, a, cons);
    }

    ctx.lifted_cover
  }
}

impl<'a, A: Clause, C: Constraint> Ctx<'a, A, C> {
  /// Declares that `sibling` is a sibling
  fn add_checked(&mut self, clause: &A, sibling: A) {
    let sibling_imp = match self.model.implications.get(&sibling) {
      Some(s) => s,
      None => return,
    };

    for c in &self.cover[clause] {
      if !sibling_imp.contains(c) {
        return
      }
    }

    self.add_unchecked(clause, sibling)
  }

  fn add_unchecked(&mut self, clause: &A, sibling: A) {
    self.current_cover_component.insert(sibling);
  }
}




pub trait Lift<A, C> {
  /// Given a clause, this method should call a `ctx.add_*()` method for each clause which is mutually exclusive with `a`.
  fn visit_candidate_siblings(&mut self, ctx: &mut Ctx<A, C>, a: &A, cons: &Set<C>);
}
