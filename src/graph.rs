use crate::Map;
use std::fmt::Debug;
use std::hash::Hash;
use std::mem::take;
use std::ops::Sub;
use std::rc::Rc;
use tracing::{debug, error_span, info, trace};

#[inline]
fn min_weights<W: Copy + PartialOrd>(weights: &[W]) -> W {
  *weights
    .iter()
    .min_by(|a, b| a.partial_cmp(b).expect("unorderable (NaN?) weights found"))
    .unwrap()
}

pub trait DecomposableDigraph<N, A, W>: Sized
where
  N: Clone + Hash + Eq + Debug + 'static,
  A: Clone + 'static,
  W: Sub + Copy + PartialOrd,
{
  fn is_sink(&self, node: &N) -> bool;

  fn next_start(&self) -> Option<N>;

  fn next_outgoing_arc(&self, node: &N) -> (A, N, W);

  fn subtract_arc(&mut self, arc: &A, weight: W);

  fn decompose_paths_cycles(mut self) -> (Vec<(Vec<A>, W)>, Vec<(Vec<A>, W)>) {
    let _span = error_span!("decompose_paths_cycles").entered();

    let mut paths = Vec::new();
    let mut cycles = Vec::new();

    let mut visited_nodes = Map::default();
    let mut node_order = Vec::new();
    let mut arcs = Vec::with_capacity(1);
    let mut weights = Vec::with_capacity(1);

    while let Some(first_node) = self.next_start() {
      visited_nodes.clear();
      node_order.clear();
      arcs.clear();
      weights.clear();
      trace!(?first_node);
      node_order.push(first_node.clone());
      visited_nodes.insert(first_node, 0usize);
      let mut current_node = node_order.last().unwrap();

      loop {
        let (arc, next_node, weight) = self.next_outgoing_arc(current_node);
        trace!(node=?next_node);
        arcs.push(arc);
        weights.push(weight);

        if let Some(&cyc_idx) = visited_nodes.get(&next_node) {
          // cycle found
          debug!(?cyc_idx, ?node_order, "cycle found");
          // println!("cycle detected {:?}", &visited_nodes);
          let cycle = arcs.split_off(cyc_idx);
          let cycle_weight = min_weights(&weights[cyc_idx..]);
          weights.truncate(cyc_idx);
          for a in &cycle {
            self.subtract_arc(a, cycle_weight);
          }
          cycles.push((cycle, cycle_weight));

          if cyc_idx > 0 {
            // backtrack to just before the start of the cycle.
            for n in node_order.drain((cyc_idx + 1)..) {
              visited_nodes.remove(&n);
            }
            current_node = node_order.last().unwrap();
            continue;
          } else {
            // we may have remove all arcs here, so start again from scratch
            break;
          }
        } else if self.is_sink(&next_node) {
          // path complete
          debug!(?node_order, "path found");
          let path = arcs.split_off(0);
          let path_weight = min_weights(&weights);
          for a in &path {
            self.subtract_arc(a, path_weight);
          }
          paths.push((path, path_weight));
          break;
        }

        // move to next node
        visited_nodes.insert(next_node.clone(), node_order.len());
        node_order.push(next_node);
        let next_node = node_order.last().unwrap();
        current_node = next_node;
      }

      visited_nodes.clear();
    }

    (paths, cycles)
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn empty_graph() {
    struct EmptyGraph;

    impl DecomposableDigraph<(), (), f64> for EmptyGraph {
      fn is_sink(&self, _: &()) -> bool {
        unreachable!()
      }
      fn next_start(&self) -> Option<()> {
        None
      }
      fn next_outgoing_arc(&self, _node: &()) -> ((), (), f64) {
        unreachable!()
      }
      fn subtract_arc(&mut self, _arc: &(), _weight: f64) {
        unreachable!()
      }
    }

    let (paths, cycles) = EmptyGraph.decompose_paths_cycles();
    assert_eq!(paths.len(), 0);
    assert_eq!(cycles.len(), 0);
  }

  #[test]
  fn single_path() {
    struct Graph(Map<(usize, usize), f64>);

    impl DecomposableDigraph<usize, (usize, usize), f64> for Graph {
      fn is_sink(&self, node: &usize) -> bool {
        *node == 42
      }

      fn next_start(&self) -> Option<usize> {
        let mut fallback = None;
        for &(i, _) in self.0.keys() {
          if i == 0 {
            return Some(0);
          }
          fallback = Some(i)
        }
        fallback
      }

      fn next_outgoing_arc(&self, node: &usize) -> ((usize, usize), usize, f64) {
        for (&(i, j), &w) in &self.0 {
          if &i == node {
            return ((i, j), j, w);
          }
        }
        println!("{:?}", self.0);
        dbg!(node);
        unreachable!("graph should be connected")
      }

      fn subtract_arc(&mut self, arc: &(usize, usize), weight: f64) {
        let val = self.0.get_mut(arc).unwrap();
        *val -= weight;
        assert!(*val >= 0.0);
        if val.abs() < 1e-12 {
          self.0.remove(arc);
        }
      }
    }

    let graph = {
      let mut g = Map::default();
      g.insert((0, 1), 1.0);
      g.insert((1, 2), 1.0);
      g.insert((2, 42), 1.0);
      Graph(g)
    };

    let (paths, cycles) = graph.decompose_paths_cycles();
    assert_eq!(paths, vec![(vec![(0, 1), (1, 2), (2, 42)], 1.0)]);
    assert_eq!(cycles.len(), 0);
  }
}
