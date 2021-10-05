#![allow(unused)]

mod unordered_pair;
mod viz;
pub mod cover;
pub mod lift;

use std::hash::{Hash, BuildHasher};
use fnv::{FnvHashMap as Map, FnvHashSet as Set};
use std::borrow::Borrow;
use crate::cover::Greedy;
use std::path::Path;
use std::io::Write;


pub trait Constraint: Clone + Hash + Eq + std::fmt::Debug {}

impl<T: Clone + Hash + Eq + std::fmt::Debug> Constraint for T {}

pub trait Clause: Hash + Eq + Clone + std::fmt::Debug {}

impl<T: Clone + Hash + Eq + std::fmt::Debug> Clause for T {}

#[derive(Hash, PartialEq, Eq, Clone, Debug)]
enum Node<A, C> {
  Clause(A),
  Constr(C),
}

#[derive(Clone, Default)]
pub struct InferenceModelBuilder<A, C: Constraint> {
  constraints: Set<C>,
  predecessors: Map<Node<A, C>, Set<Node<A, C>>>,
}


impl<A: Clause, C: Constraint> InferenceModelBuilder<A, C> {
  pub fn constraints(&self) -> &Set<C> { &self.constraints }

  pub fn add_domain_implication(&mut self, condn: A, result: A) {
    self.predecessors.entry(Node::Clause(result)).or_default().insert(Node::Clause(condn));
  }

  pub fn add_implication(&mut self, clause: A, constraint: C) {
    self.constraints.insert(constraint.clone());
    self.predecessors.entry(Node::Constr(constraint)).or_default().insert(Node::Clause(clause));
  }

  pub fn add_constraint_domination(&mut self, stronger: C, weaker: C) {
    self.constraints.insert(stronger.clone());
    self.constraints.insert(weaker.clone());
    self.predecessors.entry(Node::Constr(weaker)).or_default().insert(Node::Constr(stronger));
  }

  pub fn finish(self) -> InferenceModel<A, C> {
    #[tracing::instrument(skip(impliers, predecessors))]
    fn dfs_add<A, C>(
      impliers: &mut Map<C, Set<A>>,
      clauses: &mut Set<A>,
      in_progress: &mut Set<Node<A, C>>,
      current_node: &Node<A, C>,
      predecessors: &Map<Node<A, C>, Set<Node<A, C>>>)
      where
        A: Clause,
        C: Constraint,
    {
      if in_progress.contains(current_node) {
        panic!("cycles not supported")
      }

      match current_node {
        Node::Clause(k) => {
          if !clauses.insert(k.clone()) {
            tracing::trace!("previously explored clause");
            return;
          }
          tracing::trace!("new clause found");
        }
        Node::Constr(c) => {
          if let Some(cl) = impliers.get(c) {
            clauses.extend(cl.iter().cloned());
            tracing::trace!(prev_clauses=?cl, "previously explored constraint node");
            return;
          }
        }
      }

      in_progress.insert(current_node.clone());
      if let Some(pred) = predecessors.get(current_node) {
        for p in pred {
          dfs_add(impliers, clauses, in_progress, p, predecessors);
        }
      }
      in_progress.remove(current_node);
    }

    #[tracing::instrument(skip(dominated_constraints, successors))]
    fn dfs_build_dom_constraints<A, C>(
      dominated_constraints: &mut Map<C, Set<C>>,
      dominated: &mut Set<C>,
      current_node: &Node<A, C>,
      start_node: &Node<A, C>,
      successors: &Map<Node<A, C>, Set<Node<A, C>>>,
    )
      where
        A: Clause,
        C: Constraint,
    {
      match current_node {
        Node::Clause(_) => unreachable!(),
        Node::Constr(c) => {
          if let Some(cons) = dominated_constraints.get(c) {
            dominated.extend(cons.iter().cloned());
            return;
          }

          if current_node != start_node {
            dominated.insert(c.clone());
          }

          if let Some(succ) = successors.get(current_node) {
            for s in succ {
              dfs_build_dom_constraints(dominated_constraints, dominated, s, start_node, successors);
            }
          }
        }
      }
    }

    let InferenceModelBuilder { constraints, predecessors } = self;

    let successors = {
      let mut succ: Map<_, Set<_>> = Map::with_capacity_and_hasher(predecessors.len(), Default::default());
      for (n, pred) in &predecessors {
        for p in pred {
          succ.entry(p.clone()).or_default().insert(n.clone());
        }
      }
      succ
    };

    let mut impliers = Map::default();
    let mut stack = Set::default();

    for c in &constraints {
      let mut imp_c: Set<A> = Set::default();
      let nc = Node::Constr(c.clone());
      dfs_add(&mut impliers, &mut imp_c, &mut stack, &nc, &predecessors);
      let existing = impliers.insert(c.clone(), imp_c);
      debug_assert!(existing.is_none());
    }

    let mut implications: Map<A, Set<C>> = Map::default();

    for (c, clauses) in impliers.iter() {
      for k in clauses {
        implications.entry(k.clone()).or_default().insert(c.clone());
      }
    }

    let dominated_constraints = {
      let mut domcons = Map::default();
      let mut current = Set::default();
      for c in constraints {
        let n = &Node::Constr(c.clone());
        dfs_build_dom_constraints(
          &mut domcons,
          &mut current,
          &n,
          &n,
          &successors,
        );
        if !current.is_empty() {
          domcons.insert(c, current.clone());
          current.clear();
        }
      }
      domcons
    };

    InferenceModel {
      predecessors,
      successors,
      dominated_constraints,
      implications,
      impliers,
    }
  }
}


pub struct InferenceModel<A, C> {
  predecessors: Map<Node<A, C>, Set<Node<A, C>>>,
  successors: Map<Node<A, C>, Set<Node<A, C>>>,
  dominated_constraints: Map<C, Set<C>>,
  implications: Map<A, Set<C>>,
  impliers: Map<C, Set<A>>,
}


#[derive(Debug, Copy, Clone)]
pub struct Cover;

impl<A: Clause, C: Constraint> InferenceModel<A, C> {
  pub fn constraints<'a>(&'a self) -> impl Iterator<Item=&'a C> + 'a {
    self.impliers.keys()
  }

  pub fn clauses<'a>(&'a self) -> impl Iterator<Item=&'a A> + 'a {
    self.implications.keys()
  }

  pub fn build() -> InferenceModelBuilder<A, C> {
    InferenceModelBuilder {
      constraints: Default::default(),
      predecessors: Default::default(),
    }
  }

  fn cover_ctx<'a>(&'a self, active_clauses: &'a Set<A>, active_constraints: &'a Set<C>) -> cover::Ctx<'a, A, C> {
    // for a in active_clauses {
    //   dbg!(a);
    //   assert!(self.implications.contains_key(a))
    // }
    // for c in active_constraints {
    //   assert!(self.impliers.contains_key(c))
    // }
    cover::Ctx {
      active_clauses,
      active_constraints,
      model: self,
    }
  }

  pub fn cover<Algo: cover::CoverAlgorithm<A, C>>(&self, active_clauses: &Set<A>, constraints: &Set<C>) -> cover::Cover<A, C> {
    let ctx = self.cover_ctx(active_clauses, constraints);
    Algo::find_cover(&ctx)
  }

  pub fn cover_default(&self, active_clauses: &Set<A>, constraints: &Set<C>) -> cover::Cover<A, C> {
    self.cover::<cover::Greedy>(active_clauses, constraints)
  }

  pub fn implied_constraints(&self, active_clauses: &Set<A>) -> Set<C> {
    let mut constraints: Set<_> = Set::default();
    for c in active_clauses.iter().filter_map(|a| self.implications.get(a)).flatten() {
      if !constraints.contains(c) {
        constraints.insert(c.clone());
      }
    }

    for c in self.dominated_constraints(&constraints) {
      constraints.remove(&c);
    }
    constraints
  }

  fn dominated_constraints<'a>(&self, constraints: impl IntoIterator<Item=&'a C>) -> Set<C> where C: 'a {
      constraints.into_iter()
        .filter_map(|c| self.dominated_constraints.get(c))
        .flatten()
        .cloned()
        .collect()
  }


  pub fn viz(&self) -> viz::InferenceModelViz<A, C> {
    viz::InferenceModelViz::new(self)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
  struct Klause(u8);

  #[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
  struct Konstraint(u8);

  #[test]
  #[should_panic]
  fn adding_domain_cycle_panics() {
    let mut model = InferenceModel::build();

    model.add_domain_implication(Klause(0), Klause(1));
    model.add_domain_implication(Klause(1), Klause(0));
    model.add_implication(Klause(0), Konstraint(0));
    model.finish();
  }

  #[test]
  fn simple_graph() {
    let mut model = InferenceModel::build();
    model.add_domain_implication(Klause(1), Klause(0));
    model.add_domain_implication(Klause(3), Klause(2));
    model.add_implication(Klause(2), Konstraint(0));
    model.add_implication(Klause(0), Konstraint(1));
    model.add_constraint_domination(Konstraint(0), Konstraint(1));
    let model = model.finish();
  }

  #[test]
  #[should_panic]
  fn adding_constraint_cycle_panics() {
    let mut model = InferenceModel::build();
    model.add_constraint_domination(Konstraint(0), Konstraint(1));
    model.add_constraint_domination(Konstraint(1), Konstraint(0));
    model.add_implication(Klause(0), Konstraint(0));
    model.finish();
  }
}

