#![allow(unused)]
#![feature(hash_set_entry)]
#![feature(entry_insert)]

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
use tracing::{debug, debug_span, trace, instrument, trace_span};

pub trait Constraint: Clone + Hash + Eq + std::fmt::Debug {}

impl<T: Clone + Hash + Eq + std::fmt::Debug> Constraint for T {}

pub trait Clause: Hash + Eq + Clone + std::fmt::Debug {}

impl<T: Clone + Hash + Eq + std::fmt::Debug> Clause for T {}

#[derive(Hash, PartialEq, Eq, Clone, Debug)]
enum Node<A, C> {
  Clause(A),
  Constr(C),
}

impl<A, C> Node<A, C> {
  #[inline]
  #[track_caller]
  pub fn unwrap_clause(self) -> A {
    match self {
      Node::Clause(a) => a,
      Node::Constr(_) => panic!("called `Node::unwrap_clause()` on a `Constr` value"),
    }
  }

  #[inline]
  #[track_caller]
  pub fn unwrap_constraint(self) -> C {
    match self {
      Node::Constr(c) => c,
      Node::Clause(_) => panic!("called `Node::unwrap_constraint()` on a `Clause` value"),
    }
  }

  #[inline]
  pub fn constraint(self) -> Option<C> {
    match self {
      Node::Constr(c) => Some(c),
      Node::Clause(_) => None,
    }
  }

  #[inline]
  pub fn clause(self) -> Option<A> {
    match self {
      Node::Clause(c) => Some(c),
      Node::Constr(_) => None,
    }
  }

  #[inline]
  pub fn as_ref<'a>(&'a self) -> Node<&'a A, &'a C> {
    match *self {
      Node::Clause(ref a) => Node::Clause(a),
      Node::Constr(ref c ) => Node::Constr(c),
    }
  }
}

#[derive(Clone, Default)]
pub struct InferenceModelBuilder<A, C: Constraint> {
  clauses: Set<A>,
  constraints: Set<C>,
  predecessors: Map<Node<A, C>, Set<Node<A, C>>>,
}


impl<A: Clause, C: Constraint> InferenceModelBuilder<A, C> {
  pub fn constraints(&self) -> &Set<C> { &self.constraints }

  pub fn add_domain_implication(&mut self, condn: A, result: A) {
    self.clauses.get_or_insert_owned(&condn);
    self.clauses.get_or_insert_owned(&result);
    self.predecessors.entry(Node::Clause(result)).or_default().insert(Node::Clause(condn));
  }

  pub fn add_implication(&mut self, clause: A, constraint: C) {
    self.clauses.get_or_insert_owned(&clause);
    self.constraints.get_or_insert_owned(&constraint);
    self.predecessors.entry(Node::Constr(constraint)).or_default().insert(Node::Clause(clause));
  }

  pub fn add_constraint_domination(&mut self, stronger: C, weaker: C) {
    self.constraints.get_or_insert_owned(&stronger);
    self.constraints.get_or_insert_owned(&weaker);
    self.predecessors.entry(Node::Constr(weaker)).or_default().insert(Node::Constr(stronger));
  }

  #[instrument(level="debug", name="flatten_graph", skip(self))]
  pub fn finish(self) -> InferenceModel<A, C> {
    #[derive(Copy, Clone, Debug)]
    enum NodeState {
      Unvisited,
      Inprogress,
      Visited,
    }
    fn dfs_cycle_check<A, C>(
      current_node: &Node<A, C>,
      node_state: &mut Map<&Node<A, C>, NodeState>,
      successors: &Map<Node<A, C>, Set<Node<A, C>>>)
      where
        A: Clause,
        C: Constraint,
    {
      match node_state.get_mut(current_node).unwrap() {
        s @ NodeState::Unvisited => { *s = NodeState::Inprogress; },
        NodeState::Inprogress => panic!("cycles not yet supported"),
        NodeState::Visited => return
      }

      if let Some(succ) = successors.get(current_node) {
        for n in succ {
          dfs_cycle_check(n, node_state, successors)
        }
      }

      *node_state.get_mut(current_node).unwrap() = NodeState::Visited;
    }

    #[tracing::instrument(skip(implications, successors))]
    fn dfs_build_flat_implications<'a, A, C>(
      implications: &'a mut Map<Node<A, C>, Set<Node<A, C>>>,
      current_node: &Node<A, C>,
      successors: &Map<Node<A, C>, Set<Node<A, C>>>)
      where
        A: Clause,
        C: Constraint,
    {
      if implications.contains_key(current_node) {
        tracing::trace!(node=?current_node, "previously explored node");
        return
      }
      let mut imp = Set::default();
      if let Some(succ) = successors.get(current_node) {
        for n in succ {
          imp.insert(n.clone());
          dfs_build_flat_implications(implications, n, successors);
          imp.extend(implications[n].iter().cloned());
        }
      }
      implications.insert(current_node.clone(), imp);
    }


    let InferenceModelBuilder { clauses, constraints, predecessors } = self;
    let nodes : Set<_> = clauses.iter().map(|c| Node::Clause(c.clone()))
      .chain(constraints.iter().map(|c| Node::Constr(c.clone())))
      .collect();

    let successors = {
      let mut succ: Map<_, Set<_>> = Map::with_capacity_and_hasher(predecessors.len(), Default::default());
      for (n, pred) in &predecessors {
        for p in pred {
          succ.entry(p.clone()).or_default().insert(n.clone());
        }
      }
      succ
    };

    #[cfg(debug_assertions)] {// check for cycles
      let mut node_states = nodes.iter().map(|n| (n, NodeState::Unvisited)).collect();
      for n in &nodes {
        dfs_cycle_check(n, &mut node_states, &successors);
      }
      debug!("cycle check found no cycles")
    }


    let mut flat_successors = Map::with_capacity_and_hasher(nodes.len(), Default::default());

    for n in &nodes {
      let _s = trace_span!("dfs_search", node=?n).entered();
      dfs_build_flat_implications(&mut flat_successors,n, &successors);
      trace!("DFS complete");
    }

    let mut dominated_constraints = Map::default();
    let mut implications = Map::default();

    for (n, succ) in flat_successors {
      match n {
        Node::Clause(a) => {
          let cons : Set<_> = succ.into_iter().filter_map(Node::constraint).collect();
          implications.insert(a, cons);
        },
        Node::Constr(c) => {
          let cons : Set<_> = succ.into_iter().map(Node::unwrap_constraint).collect();
          dominated_constraints.insert(c, cons);
        }
      }
    }

    let mut impliers: Map<_, Set<_>> = Map::default();
    for (a, cons) in &implications {
      for c in cons {
        impliers.entry(c.clone())
          .or_default()
          .insert(a.clone());
      }
    }
    debug!("finished");
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

pub trait ImpliedConstraintsVistor<C> {
  fn visit(&mut self, constraint: &C);
}

impl<A: Clause, C: Constraint> InferenceModel<A, C> {
  pub fn constraints<'a>(&'a self) -> impl Iterator<Item=&'a C> + 'a {
    self.impliers.keys()
  }

  pub fn clauses<'a>(&'a self) -> impl Iterator<Item=&'a A> + 'a {
    self.implications.keys()
  }

  pub fn build() -> InferenceModelBuilder<A, C> {
    InferenceModelBuilder {
      clauses: Default::default(),
      constraints: Default::default(),
      predecessors: Default::default(),
    }
  }

  fn cover_ctx<'a>(&'a self, active_clauses: &'a Set<A>, active_constraints: &'a Set<C>) -> cover::Ctx<'a, A, C> {
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

  // pub fn visit_implied_constraints(&self, active_clauses: impl IntoIterator<Item=&C>, visitor: &mut impl ImpliedConstraintsVistor) {
  //   for c in active_clauses.into_iter().filter_map(|a| self.implications.get(a)).flatten() {
  //     visitor.visit(c);
  //   }
  // }

  pub fn implied_constraints(&self, active_clauses: &Set<A>) -> Set<C> {
    let mut constraints: Set<_> = Set::default();
    for c in active_clauses.iter().filter_map(|a| self.implications.get(a)).flatten() {
      if !constraints.contains(c) {
        constraints.insert(c.clone());
      }
    }
    constraints
  }

  pub fn remove_dominated_constraints(&self, cons: &mut Set<C>) {
    let implied = self.dominated_constraints(&*cons);
    cons.retain(|c| !implied.contains(c));
  }

  fn dominated_constraints<'a>(&self, constraints: impl IntoIterator<Item=&'a C>) -> Set<C> where C: 'a {
    // Assumption: graph doesn't contain cycles (currently checked during ) - otherwise we remove entire cycle of constraints
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

