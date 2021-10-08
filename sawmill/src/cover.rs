use crate::*;
use std::hash::Hasher;
use tracing::*;

pub struct Ctx<'a, A, C: Constraint> {
  pub(crate) active_clauses: &'a Set<A>,
  pub(crate) active_constraints: &'a Set<C>,
  pub(crate) model: &'a InferenceModel<A, C>
}

pub type Cover<A, C> = Map<A, Set<C>>;

impl<'a, A: Clause, C: Constraint> Ctx<'a, A, C> {
  pub fn nearest_active_clause(&self, cons: C) -> (u64, &A) {
    let mut dist = 0u64;
    let mut queue = std::collections::VecDeque::new();
    let node = Node::Constr(cons);
    let mut n = &node;
    loop {
      dist += 1;
      if let Some(pred) = self.model.predecessors.get(n) {
        for p in pred {
          if let Node::Clause(a) = p {
            return (dist, a)
          }
        }
        queue.extend(pred)
      }
      n = queue.pop_front().expect("BFS queue empty: active constraint with no active predecessor");
    }
  }

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
    let _s = trace_span!("greedy_cover").entered();
    let active_clauses = ctx.active_clauses();
    let active_constraints = ctx.active_constraints();

    let mut uncovered_constr = active_constraints.clone();
    let mut cover = Map::default();

    // Preprocessing: Some constraints are only implied by one clause.  These can be pre-processed out, since the
    // associated clause must be in the cover.
    for c in active_constraints {
      if !uncovered_constr.contains(c) {
        continue
      }

      let clauses : Set<_> = ctx.active_clauses_implying(c).collect();
      debug_assert!(!clauses.is_empty());
      if clauses.len() == 1 {
        let a = clauses.into_iter().next().unwrap().clone();
        let cons : Set<_> = ctx.implied_constraints(&a)
          .filter(|c| uncovered_constr.contains(c))
          .cloned()
          .collect();
        for c in &cons {
          uncovered_constr.remove(c);
        }
        cover.insert(a, cons);
      }
    }
    trace!(?cover, "preprocessing done");

    let mut scores = Map::with_capacity_and_hasher(active_clauses.len(), Default::default());

    for c in &uncovered_constr {
      for a in ctx.active_clauses_implying(c) {
        scores.entry(a).and_modify(|x| *x += 1).or_insert(1u32);
      }
    }
    scores.retain(|a, scr| *scr > 1 );
    trace!(?scores, ?uncovered_constr);

    while !uncovered_constr.is_empty() {
      if let Some((&a, best_score)) = scores.iter().max_by_key(|(a, x)| *x) {
        debug_assert_ne!(*best_score, 0);
        if *best_score == 1 {
          break
        }
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
      } else {
        break
      }
    }


    // Post-processing - if all that is left is one-to-one coverings, use the logical distances instead.
    for c in uncovered_constr {
      let (_, a) = ctx.nearest_active_clause(c.clone());
      let mut cons = Set::with_capacity_and_hasher(
        1, Default::default()
      );
      cons.insert(c);
      cover.insert(a.clone(), cons);
    }

    cover
  }
}