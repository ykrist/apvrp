use crate::*;
use std::hash::Hasher;

pub struct Ctx<'a, A, C: Constraint> {
  pub(crate) active_clauses: &'a Set<A>,
  pub(crate) active_constraints: &'a Set<C>,
  pub(crate) model: &'a InferenceModel<A, C>
}

pub type Cover<A, C> = Map<A, Set<C>>;

impl<'a, A: Clause, C: Constraint> Ctx<'a, A, C> {
  pub fn active_clauses(&self) -> &Set<A> {
    self.active_clauses
  }

  pub fn active_constraints(&self) -> &Set<C> {
    self.active_constraints
  }

  pub fn implied_constraints(&self, a: &A) -> impl Iterator<Item=&C> {
    self.model.implications[a].iter()
  }

  pub fn active_implied_constraints(&self, a: &A) -> impl Iterator<Item=&C> {
    self.implied_constraints(a).filter(move |c| self.active_constraints.contains(c))
  }

  pub fn clauses_implying(&self, c: &C) -> impl Iterator<Item=&A> {
    tracing::trace!(?c);
    self.model.impliers[c].iter()
  }

  pub fn active_clauses_implying(&self, c: &C) -> impl Iterator<Item=&A> {
    self.clauses_implying(c).filter(move |a| self.active_clauses.contains(a))
  }
}

pub trait CoverAlgorithm<A, C: Constraint> {
  fn find_cover(ctx: &Ctx<A, C>) -> Cover<A, C>;
}

pub struct Greedy;

impl<A: Clause, C: Constraint> CoverAlgorithm<A, C> for Greedy {
  fn find_cover(ctx: &Ctx<A, C>) -> Cover<A, C> {
    let active_clauses = ctx.active_clauses();
    let active_constraints = ctx.active_constraints();

    let mut scores = Map::with_capacity_and_hasher(active_clauses.len(), Default::default());

    for c in active_constraints {
      for a in ctx.active_clauses_implying(c) {
        scores.entry(a).and_modify(|x| *x += 1).or_insert(1u32);
      }
    }
    println!("{:?}", scores);
    let mut uncovered_constr = active_constraints.clone();
    let mut cover = Map::default();

    while !uncovered_constr.is_empty() {
      let (&a, best_score) = scores.iter().max_by_key(|(a, x)| *x).unwrap();
      debug_assert_ne!(best_score, &0);
      scores.remove(a);

      let newly_covered: Set<_> = ctx.implied_constraints(a)
        .filter(|c| uncovered_constr.contains(c))
        .cloned()
        .collect();

      for c in &newly_covered {
        for ad in ctx.active_clauses_implying(c) {
          if let Some(cnt) = scores.get_mut(ad) {
            *cnt -= 1;
          }
        }
        uncovered_constr.remove(c);
      }
      cover.insert(a.clone(), newly_covered);
    }

    cover
  }
}