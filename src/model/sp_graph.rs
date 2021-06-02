use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use super::mp::MpVars;
use super::cb::{CbError};
use tracing::{error_span, debug, trace, error, debug_span, trace_span};
use itertools::Itertools;
use crate::solution::*;
use std::collections::{HashSet, HashMap, hash_map::Entry, BinaryHeap};
use std::hash::{Hash, BuildHasher};
use crate::utils::HashMapExt;
use smallvec::SmallVec;
use std::cmp::{PartialOrd, Eq, PartialEq, Ord, Ordering, max, Reverse};
use std::path::Path;
use std::fmt::Debug;
use std::fmt;
use super::*;


#[derive(Debug, Clone)]
pub struct Node {
  idx: usize,
  avg: Avg,
  task: Task,
  time: Time,
  active_pred: Option<usize>,
  #[cfg(debug_assertions)]
  forward_order: i32,
}

impl Node {
  pub fn new(idx: usize, avg: Avg, task: Task) -> Self {
    Node {
      idx,
      avg,
      task,
      time: task.t_release,
      active_pred: None,
      #[cfg(debug_assertions)]
      forward_order: -1,
    }
  }

  pub fn reset(&mut self) {
    self.time = self.time_lb();
    self.active_pred = None;
    #[cfg(debug_assertions)] {
      self.forward_order = -1;
    }
  }
}

impl Node {
  #[inline]
  pub fn time_ub(&self) -> Time { self.task.t_deadline - self.task.tt }
  #[inline]
  pub fn time_lb(&self) -> Time { self.task.t_release }
}

#[derive(Hash, Debug, Copy, Clone, Eq, PartialEq)]
pub enum EdgeKind {
  // Passive vehicle unloading time
  Unloading,
  // Passive vehicle loading time
  Loading,
  // Active vehicle travel time
  AVTravel,
}

#[derive(Hash, Debug, Copy, Clone, Eq, PartialEq)]
pub struct Edge {
  from: usize,
  to: usize,
  kind: EdgeKind,
  weight: Time,
}

type EdgeList = SmallVec<[Edge; 10]>;

#[derive(Clone, Debug)]
pub struct Graph {
  edges_from_node: Vec<SmallVec<[Edge; 2]>>,
  edges_to_node: Vec<SmallVec<[Edge; 2]>>,
  nodes: Vec<Node>,
  source_connected_nodes: usize,
  sources: Vec<usize>,
  last_task_nodes: Vec<usize>,
  last_task_tt_to_depot: Vec<Time>,
  av_ddepot: Task,
}

impl Graph {
  #[tracing::instrument(level = "debug", skip(data, tasks, sol))]
  pub fn build(data: &Data, tasks: &Tasks, sol: &Solution) -> Self {
    let mut nodes = Vec::with_capacity(sol.av_routes.iter().map(|(_, route)| route.len()).sum());
    let mut task_to_node_idx = map_with_capacity(nodes.len());
    let mut last_task_nodes = Vec::with_capacity(sol.av_routes.len());
    let mut last_task_tt_to_depot = Vec::with_capacity(sol.av_routes.len());

    for (avg, route) in &sol.av_routes {
      // trim the depots from the routes
      for &task in &route[1..route.len() - 1] {
        task_to_node_idx.insert(task, nodes.len());
        nodes.push(Node::new(nodes.len(), *avg, task));
      }

      let last_task: &Task = &nodes.last().unwrap().task;
      last_task_tt_to_depot.push(last_task.tt + data.travel_time[&(last_task.end, Loc::Ad)]);
      last_task_nodes.push(nodes.len() - 1); // last node we pushed was the task before the DDp node
    }

    trace!(?task_to_node_idx);
    let mut edges = Map::default();
    for (_, pv_route) in &sol.pv_routes {
      for (t1, t2) in pv_route.iter().tuple_windows() {
        if !(t1.ty == TaskType::Request || t2.ty == TaskType::Request) {
          continue;
        }

        let from = task_to_node_idx[t1];
        let to = task_to_node_idx[t2];
        let weight = t1.tt + data.srv_time[&t1.end];

        let kind = if t2.ty == TaskType::Request {
          // Constraints 3c)
          EdgeKind::Loading
        } else {
          debug_assert_eq!(t1.ty, TaskType::Request);
          // Constraints 3d)
          EdgeKind::Unloading
        };

        let edge = Edge { from, to, weight, kind };
        trace!(weight=?edge.weight, kind=?edge.kind, from=?t1, to=?t2, "add edge");
        edges.insert((from, to), edge);
      }
    }

    for (avg, route) in &sol.av_routes {
      for (t1, t2) in route[1..route.len() - 1].iter().tuple_windows() {
        let from = task_to_node_idx[t1];
        let to = task_to_node_idx[t2];
        if let Entry::Vacant(entry) = edges.entry((from, to)) {
          // Constraints 3b)
          let edge = entry.insert(Edge {
            from,
            to,
            kind: EdgeKind::AVTravel,
            weight: t1.tt + data.travel_time[&(t1.end, t2.start)],
          });
          trace!(weight=?edge.weight, kind=?edge.kind, from=?t1, to=?t2, "add edge");
        }
      }
    }

    let mut edges_from_node = vec![SmallVec::<[Edge; 2]>::new(); nodes.len()];
    let mut edges_to_node = vec![SmallVec::<[Edge; 2]>::new(); nodes.len()];

    for ((from, to), edge) in edges {
      edges_from_node[from].push(edge.clone());
      edges_to_node[to].push(edge);
    }

    let mut sources = Vec::with_capacity(sol.av_routes.len());

    for n in &nodes {
      if edges_to_node[n.idx].is_empty() {
        sources.push(n.idx);
      }
    }
    debug_assert_ne!(sources.len(), 0);

    let connected_nodes = nodes.len();

    trace!(?edges_from_node, ?edges_to_node, ?nodes, ?sources, ?last_task_nodes, "built");
    Graph {
      edges_from_node,
      edges_to_node,
      nodes,
      source_connected_nodes: connected_nodes,
      sources,
      last_task_nodes,
      last_task_tt_to_depot,
      av_ddepot: tasks.ddepot,
    }
  }


  fn remove_edge(&mut self, e: &Edge) {
    self.edges_to_node[e.to].retain(|g| g.from != e.from);
    self.edges_from_node[e.from].retain(|g| g.to != e.to);
  }

  fn edge_active(&self, e: &Edge) -> bool {
    &(self.nodes[e.to].active_pred) == &Some(e.from)
  }

  pub fn save_as_dot(&self, path: impl AsRef<Path>) -> Result<()> {
    let mut file = std::fs::File::create(path).map(std::io::BufWriter::new)?;
    dot::render(self, &mut file)?;
    Ok(())
  }

  pub fn save_as_svg(&self, path: impl AsRef<Path>) -> Result<()> {
    use std::process::{Command, Stdio};
    use std::io::Write;
    let mut dot_contents = Vec::with_capacity(1000);
    dot::render(self, &mut dot_contents)?;

    let mut gv = std::process::Command::new("dot")
      .arg(format!("-o{}", path.as_ref().as_os_str().to_str().expect("printable filename")))
      .arg("-Grankdir=LR")
      .arg("-Tsvg")
      .stdin(Stdio::piped())
      .stdout(Stdio::inherit())
      .stderr(Stdio::inherit())
      .spawn()?;

    gv.stdin.take().unwrap().write_all(&dot_contents)?;
    gv.wait()?;
    Ok(())
  }

  fn get_edge(&self, from: usize, to: usize) -> &Edge {
    for e in &self.edges_from_node[from] {
      if e.to == to {
        return e;
      }
    }
    unreachable!()
  }

  #[tracing::instrument(level = "debug", skip(self))]
  fn find_cycles(&self) -> Vec<EdgeList> {
    trace!("eeee");
    #[derive(Copy, Clone, Debug)]
    enum NodeState {
      // We have explored all elementary paths from this node
      Removed,
      // The node is in the current elementary path
      CurrentPath,
      // The node is not in the current elementary path and is not Removed.
      Present,
    }

    fn fmt_blocked_neighbours(bn: &[SmallVec<[usize;2]>]) -> String {
      use std::fmt::Write;
      let mut s = String::from("{");

      for (i, neigh) in bn.iter().enumerate() {
        if neigh.len() > 0 {
          write!(s, "{} ->{:?}, ", i, neigh).unwrap();
        }
      }
      s.push('}');
      s
    }

    let mut cycles = Vec::new();
    let mut node_state = vec![NodeState::Present; self.nodes.len()];
    let mut blocked_neighbours = vec![SmallVec::<[usize; 2]>::new(); self.nodes.len()];
    let mut current_path = Vec::with_capacity(self.nodes.len());

    for root in 0..self.nodes.len() {
      current_path.push(root);
      node_state[root] = NodeState::CurrentPath;
      let mut i = root;

      'outer: loop {
        let _s = trace_span!("loop", i, ?current_path).entered();
        // trace!(?current_path, bn=%fmt_blocked_neighbours(&blocked_neighbours));

        // try to extend the current path to a new unvisited path
        for j in self.edges_from_node[i].iter().map(|e| e.to) {
          if matches!(node_state[j], NodeState::Present) && !blocked_neighbours[i].contains(&j) {
            current_path.push(j);
            node_state[j] = NodeState::CurrentPath;
            trace!(from=i, to=j, "extend");
            i = j;
            continue 'outer;
          } else {
            trace!(j, state=?node_state[j], bni=?blocked_neighbours[i], "forbidden");
          }
        }

        // cannot extend path further subject to the above three conditions
        if self.edges_from_node[i].iter().any(|e| e.to == root) {
          trace!("cycle found!");
          let c = current_path.iter()
            .copied()
            .tuple_windows()
            .chain(std::iter::once((i, root)))
            .map(|(from, to)| *self.get_edge(from, to))
            .collect();
          cycles.push(c);
        }

        if current_path.len() > 1 {
          // shorten the current path (stack pop)
          let j = current_path.pop().unwrap();
          i = *current_path.last().unwrap();
          node_state[j] = NodeState::Present;

          blocked_neighbours[j].clear();
          blocked_neighbours[i].push(j);
          trace!(popped=j, bn=%fmt_blocked_neighbours(&blocked_neighbours), "pop");

        } else {
          // advance the root node
          node_state[i] = NodeState::Removed;
          current_path.clear();
          break;
        }
      }
    }
    trace!(?cycles);
    cycles
  }

  #[tracing::instrument(level = "debug", skip(self))]
  fn find_cycle(&self) -> EdgeList {
    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    enum NodeState {
      NotVisited,
      Current,
      Visited,
    }
    use NodeState::*;

    #[tracing::instrument(level = "trace", skip(graph, node_state))]
    fn dfs(graph: &Graph, node_state: &mut Vec<NodeState>, node: usize, order: usize) -> Option<EdgeList> {
      match node_state[node] {
        NotVisited => node_state[node] = Current,
        Visited => return None,
        Current => {
          // Re-construct the cycle
          let mut edges = EdgeList::new();
          let mut n = node;
          loop {
            let edge = *graph.edges_from_node[n].iter()
              .filter(|e| node_state[e.to] == Current)
              .next()
              .expect("should have at least one Current succ");

            n = edge.to;
            edges.push(edge);

            if n == node { break; }
          }
          trace!(?edges, "cycle found");
          return Some(edges);
        }
      }

      for e in &graph.edges_from_node[node] {
        let cycle = dfs(graph, node_state, e.to, order + 1);
        if cycle.is_some() {
          return cycle;
        }
      }

      node_state[node] = Visited;
      None
    }

    let mut node_state = vec![NotVisited; self.nodes.len()];

    for &n in &self.sources {
      let cycle = dfs(self, &mut node_state, n, 0);
      if let Some(c) = cycle {
        return c;
      }
    }
    unreachable!()
  }

  #[tracing::instrument(level = "debug", skip(self))]
  // For each infeasible UB, returns the smallest IIS containing that UB, if one exists.
  fn backlabel_min_iis(&self, ub_violated_nodes: &[usize]) -> Vec<EdgeList> {
    // Which predecessor to move to build the IIS path
    type Action = usize;

    #[tracing::instrument(level = "trace", skip(graph, cache))]
    fn backlabel_dp(graph: &Graph, cache: &mut Map<(usize, Time), Option<(u16, Action)>>, node: usize, time: Time) -> Option<(u16, Action)> {
      // Base cases
      let n = &graph.nodes[node];
      if time > n.time_ub() {
        trace!("no iis");
        return None;
      } else if time < n.time_lb() {
        trace!("found iis");
        return Some((0, node));
      }

      // Cached Non-base cases
      if let Some(val) = cache.get(&(node, time)) {
        trace!(?val, "cache hit");
        return *val;
      }

      // Compute non-base case
      let mut best_val = u16::MAX;
      let mut best_action = None;

      for edge in &graph.edges_to_node[node] {
        if let Some((mut val, _)) = backlabel_dp(graph, cache, edge.from, time - edge.weight) {
          val += 1; // add the cost from this edge
          if val < best_val {
            best_val = val;
            best_action = Some(edge.from);
          }
        }
      }

      let best_val_action = best_action.map(|a| (best_val, a));
      cache.insert((node, time), best_val_action);
      trace!(?best_val_action, "cache insert");
      best_val_action
    }

    let mut cache = map_with_capacity(self.nodes.len() * 2);

    let iises : Vec<_> = ub_violated_nodes.iter()
      .filter_map(|&node| {
        let time = self.nodes[node].time_ub();
        backlabel_dp(self, &mut cache, node, time).map(|(path_len, mut pred_node)| {
          let mut iis = EdgeList::new();
          let mut time = time;
          let mut node = node;

          while node != pred_node {
            trace!(pred_node, node);
            let e = *self.get_edge(pred_node, node);
            time -= e.weight;
            iis.push(e);
            node = pred_node;
            pred_node = backlabel_dp(self, &mut cache, node, time).unwrap().1;
          }
          iis
        })
      })
      .collect();

    trace!(?iises);
    debug_assert_ne!(iises.len(), 0);
    iises
  }

  #[tracing::instrument(level = "debug", skip(self))]
  fn backlabel_mrs(&self, last_task_nodes: &[usize]) -> Vec<EdgeList> {
    let mut edge_list = EdgeList::new();
    let mut backlabelled = if last_task_nodes.len() > 1 {
      Some(vec![false; self.nodes.len()])
    } else {
      None
    };

    for &start_node in last_task_nodes {
      // Reconstruct the path from this node
      let mut node = &self.nodes[start_node];

      while let Some(pred_idx) = node.active_pred {
        if let Some(backlabelled) = &mut backlabelled {
          // finish early, another start_node has caused us to
          // label this path from here on.  Can only happen if we have multiple start nodes
          let bl = backlabelled.get_mut(node.idx).unwrap();
          if *bl { break; } else { *bl = true }
        }

        // find the red edge arriving at this node
        let edge = 'outer: loop {
          for e in &self.edges_to_node[node.idx] {
            if e.from == pred_idx {
              break 'outer *e;
            }
          }
          unreachable!()
        };

        // push and move back along red edge
        edge_list.push(edge);
        node = &self.nodes[pred_idx];
      }
    }

    // TODO: disaggregate the MRS
    vec![edge_list]
  }

  /// Check the if the supplied non-source node should become a new source nodes
  /// or marked as isolated
  fn reset(&mut self) {
    self.sources.clear();
    self.source_connected_nodes = self.nodes.len();

    for node in &mut self.nodes {
      if self.edges_to_node[node.idx].len() == 0 {
        if self.edges_from_node[node.idx].len() == 0 {
          self.source_connected_nodes -= 1; // node is unreachable from sources
        } else {
          self.sources.push(node.idx);
        }
      }
      node.reset();
    }
  }

  fn get_cyclic_iis_constraints(&self, edges: &[Edge]) -> SpConstraints {
    edges.iter().map(|e| self.get_sp_constraint(e)).collect()
  }

  fn get_path_iis_constraints(&self, edges: &[Edge]) -> SpConstraints {
    // edges are returned in reverse order, so edge.last().unwrap().from is the first node along the IIS
    // path and `edges[0].to` is the node indexed by `ub_violate_node`.
    let mut iis: SpConstraints = edges.iter().map(|e| self.get_sp_constraint(e)).collect();
    iis.push(SpConstr::Ub(self.nodes[edges.first().unwrap().to].task));
    iis.push(SpConstr::Lb(self.nodes[edges.last().unwrap().from].task));
    iis
  }

  fn get_mrs_constraints(&self, edges: &[Edge]) -> SpConstraints {
    let mut mrs : SpConstraints = edges.iter().map(|e| self.get_sp_constraint(e)).collect();
    mrs.extend(self.last_task_nodes.iter()
      .map(|&n| SpConstr::AvTravelTime(self.nodes[n].task, self.av_ddepot)));
    mrs
  }

  fn get_sp_constraint(&self, edge: &Edge) -> SpConstr {
    match edge.kind {
      EdgeKind::AVTravel => SpConstr::AvTravelTime(self.nodes[edge.from].task, self.nodes[edge.to].task),
      EdgeKind::Loading => SpConstr::Loading(self.nodes[edge.from].task),
      EdgeKind::Unloading => SpConstr::Unloading(self.nodes[edge.to].task),
    }
  }
}

impl<'a> dot::GraphWalk<'a, Node, Edge> for Graph {
  fn nodes(&'a self) -> dot::Nodes<'a, Node> {
    let nodes: Vec<_> = self.nodes.iter()
      .filter(|n| !(self.edges_from_node[n.idx].is_empty() && self.edges_to_node[n.idx].is_empty()))
      .cloned()
      .collect();
    nodes.into()
  }

  fn edges(&'a self) -> dot::Edges<'a, Edge> {
    let edges: Vec<_> = self.edges_to_node.iter()
      .flat_map(|es| es.iter())
      .cloned()
      .collect();
    dot::Edges::Owned(edges)
  }

  fn source(&'a self, edge: &Edge) -> Node {
    self.nodes[edge.from].clone()
  }

  fn target(&'a self, edge: &Edge) -> Node {
    self.nodes[edge.to].clone()
  }
}

impl<'a> dot::Labeller<'a, Node, Edge> for Graph {
  fn graph_id(&'a self) -> dot::Id<'a> { dot::Id::new("solution").unwrap() }

  fn node_id(&'a self, n: &Node) -> dot::Id<'a> { dot::Id::new(format!("N{}", n.idx)).unwrap() }

  fn node_label(&'a self, n: &Node) -> dot::LabelText<'a> {
    #[cfg(not(debug_assertions))]
      let label = format!(
      "{:?} ({})\nt={}\n[{}, {}]",
      &n.task, n.idx, n.time, n.time_lb(), n.time_ub());

    #[cfg(debug_assertions)]
      let label = format!(
      "{:?} ({})\nt={}, ord={}\n[{}, {}]",
      &n.task, n.idx, n.time, n.forward_order, n.time_lb(), n.time_ub());

    dot::LabelText::escaped(label)
  }

  fn edge_label(&'a self, e: &Edge) -> dot::LabelText<'a> {
    dot::LabelText::escaped(format!("{:?}\n{:?}", &e.weight, e.kind))
  }

  fn edge_color(&'a self, e: &Edge) -> Option<dot::LabelText<'a>> {
    if self.edge_active(e) {
      Some(dot::LabelText::label("red"))
    } else {
      Some(dot::LabelText::label("black"))
    }
    // None
  }

  #[cfg(debug_assertions)]
  fn node_color(&'a self, n: &Node) -> Option<dot::LabelText<'a>> {
    let color = if n.time > n.time_ub() {
      "red"
    } else if n.forward_order >= 0 {
      "royalblue"
    } else {
      "grey"
    };
    Some(dot::LabelText::LabelStr(color.into()))
  }

  fn kind(&self) -> dot::Kind { dot::Kind::Digraph }
}


#[derive(Debug, Clone)]
pub enum InfKind {
  Cycle,
  Time(Vec<usize>),
}

impl SpSolve for Graph {
  type OptInfo = ();
  type InfInfo = InfKind;
  type IisConstraintSets = Vec<SpConstraints>;
  type MrsConstraintSets = Vec<SpConstraints>;

  fn solve(&mut self) -> Result<SpStatus<Self::OptInfo, Self::InfInfo>> {
    let mut queue = SmallVec::<[usize; NUM_AV_UB * 2]>::new();
    queue.extend_from_slice(&self.sources);
    trace!(?queue, "init");
    let mut num_visited_pred = vec![0usize; self.nodes.len()];
    let mut violated_ub_nodes = Vec::new();
    let nodes = &mut self.nodes;
    let edges_from_node = &self.edges_from_node;


    let mut nodes_processed = 0;

    while let Some(node_idx) = queue.pop() {
      let current_node = &mut nodes[node_idx];
      let _s = trace_span!("process_node", node=node_idx, time=current_node.time, task=?current_node.task).entered();

      #[cfg(debug_assertions)] {
        current_node.forward_order = nodes_processed as i32;
      }
      nodes_processed += 1;
      let time = current_node.time;

      for edge in &edges_from_node[node_idx] {
        let next_node: &mut Node = &mut nodes[edge.to];
        let new_time = time + edge.weight;

        if next_node.time < new_time {
          // update neighbouring label's time
          next_node.time = new_time;
          next_node.active_pred = Some(node_idx);
          // check feasibility
          if new_time > next_node.time_ub() {
            violated_ub_nodes.push(next_node.idx);
            debug!(new_node = next_node.idx, time = next_node.time, ub = next_node.time_ub(), "infeasibility found");
          } else {
            trace!(new_node = next_node.idx, time = next_node.time, "update time");
          }
        }

        // If the next node has had all its predecessors processed, add it to the queue
        let visited_pred = &mut num_visited_pred[edge.to];
        *visited_pred += 1;
        if *visited_pred == self.edges_to_node[edge.to].len() {
          queue.push(edge.to);
          trace!(?queue, new_node=next_node.idx, time=next_node.time, task=?next_node.task, "push node");
        }
      }
    }

    if nodes_processed < self.source_connected_nodes {
      Ok(SpStatus::Infeasible(InfKind::Cycle))
    } else if !violated_ub_nodes.is_empty() {
      Ok(SpStatus::Infeasible(InfKind::Time(violated_ub_nodes)))
    } else {
      let obj = self.last_task_nodes.iter()
        .zip(&self.last_task_tt_to_depot)
        .map(|(&n, &tt)| self.nodes[n].time + tt)
        .sum();

      debug!(obj, "subproblem optimal");
      Ok(SpStatus::Optimal(obj, ()))
    }
  }

  fn extract_and_remove_iis(&mut self, inf: Self::InfInfo) -> Result<Self::IisConstraintSets> {
    let mut edges_to_remove = Set::default();
    let iis_sets = match inf {
      InfKind::Cycle => {
        self.find_cycles().into_iter()
          .map(|cycle| {
            let iis = self.get_cyclic_iis_constraints(&cycle);
            edges_to_remove.extend(cycle);
            iis
          })
          .collect()
      }
      InfKind::Time(ub_violated_nodes) => {
        self.backlabel_min_iis(&ub_violated_nodes).into_iter()
          .map(|path| {
            let iis = self.get_path_iis_constraints(&path);
            edges_to_remove.extend(path);
            iis
          })
          .collect()
      }
    };
    for e in edges_to_remove {
      self.remove_edge(&e);
    }
    self.reset();
    Ok(iis_sets)
  }

  fn extract_mrs(&self, _: ()) -> Result<Self::MrsConstraintSets> {
    let mrs_sets = self.backlabel_mrs(&self.last_task_nodes).into_iter()
      .map(|edges| self.get_mrs_constraints(&edges))
      .collect();
    Ok(mrs_sets)
  }
}


#[cfg(test)]
mod tests {
  use tracing::*;
  use super::*;
  use super::super::sp_lp::TimingSubproblem;
  use grb::Env;
  use std::fmt::Debug;

  struct TestData {
    data: Data,
    sets: Sets,
    tasks: Tasks,
    solutions: Vec<Solution>,
  }

  fn load(idx: usize) -> Result<TestData> {
    let data = dataset(0.5)
      .load_instance(idx)
      .map(preprocess::full_pipeline)?;

    let sets = Sets::new(&data);

    // `pv_req_t_start` is the earliest time we can *depart* from request r's pickup with passive vehicle p
    let pv_req_t_start = schedule::earliest_departures(&data);
    let tasks = Tasks::generate(&data, &sets, &pv_req_t_start);

    let path = crate::utils::test_data_file(&format!("{}-sollog.ndjson", idx));
    let solutions = std::fs::read_to_string(path).unwrap();

    let solutions: Result<Vec<_>> = serde_json::de::Deserializer::from_str(&solutions)
      .into_iter::<SerialisableSolution>()
      .map(|s| Ok(Solution::from_serialisable(&tasks, &s?)))
      .collect();

    Ok(TestData { data, sets, tasks, solutions: solutions? })
  }

  fn compare_graph_algo_with_lp_model(td: &TestData, sidx: usize, env: &Env, output_debug_files: bool) -> Result<()> {
    let sol = &td.solutions[sidx];
    let mut graph = Graph::build(&td.data, &td.tasks, sol);
    let mut lp = TimingSubproblem::build(env, &td.data, &td.tasks, sol)?;

    fn print_constr_info<'a, K: 'a + Debug>(model: &grb::Model, cgroup: &str, constraints: impl IntoIterator<Item=(&'a K, &'a Constr)>) {
      for (key, c) in constraints {
        if model.get_obj_attr(attr::IISConstr, c).unwrap() > 0 {
          let rhs = model.get_obj_attr(attr::RHS, c).unwrap();
          trace!(%cgroup, constr=?key, ?rhs, "IIS constr")
        }
      }
    }

    fn print_lp_iis(lp: &mut TimingSubproblem) {
      lp.model.compute_iis().unwrap();
      print_constr_info(&lp.model, "loading", &lp.cons.loading);
      print_constr_info(&lp.model, "unloading", &lp.cons.unloading);
      print_constr_info(&lp.model, "ub", &lp.cons.ub);
      print_constr_info(&lp.model, "lb", &lp.cons.lb);
      print_constr_info(&lp.model, "av_sync", &lp.cons.av_sync);
    }
    fn print_lp_soln(lp: &TimingSubproblem) {
      for (task, t) in get_var_values(&lp.model, &lp.vars).unwrap() {
        trace!(?task, %t, "LP solution");
      }
    }

    for iter in 0.. {
      let _s = trace_span!("solve_loop", iter).entered();
      let graph_result = graph.solve()?;
      let lp_result = lp.solve()?;

      if output_debug_files {
        match &graph_result {
          SpStatus::Infeasible(..) => graph.save_as_svg(format!("debug-inf-{}.svg", iter))?,
          SpStatus::Optimal(..) => graph.save_as_svg(format!("debug-opt-{}.svg", iter))?,
        };

        lp.model.write(&format!("sp-dbg-{}.lp", iter))?;
      }

      match (graph_result, lp_result) {
        (SpStatus::Optimal(g_obj, _), SpStatus::Optimal(lp_obj, _)) => {
          assert_eq!(g_obj, lp_obj);

          let g_mrs = &mut graph.extract_mrs(())?[0];
          g_mrs.sort();
          let lp_mrs = &mut graph.extract_mrs(())?[0];
          lp_mrs.sort();
          assert_eq!(g_mrs, lp_mrs);
          debug!(obj=g_obj, mrs=?g_mrs, "Obj and MRS are the same!");
          break;
        }

        (SpStatus::Infeasible(nidx), SpStatus::Infeasible(_)) => {
          print_lp_iis(&mut lp);
          let mut g_iis = graph.extract_and_remove_iis(nidx)?;
          let mut lp_iis = lp.extract_and_remove_iis(())?.next().unwrap();
          assert!(g_iis.iter().map(|iis| iis.len()).min().unwrap() <= lp_iis.len());

          if g_iis.len() == 1 {
            let mut g_iis = g_iis.pop().unwrap();
            g_iis.sort();
            lp_iis.sort();

            if g_iis == lp_iis {
              debug!(iis=?g_iis, "IIS are the same!");
              continue;
            }
          }

          warn!(?g_iis, ?lp_iis, "IIS divergence");
          break;
        }

        (a, b @ SpStatus::Infeasible(_)) => {
          print_lp_iis(&mut lp);
          panic!("mismatch!\ngraph algo gave {:?}\nLP gave {:?}", a, b)
        }

        (a, b @ SpStatus::Optimal(..)) => {
          panic!("mismatch!\ngraph algo gave {:?}\nLP gave {:?}", a, b)
        }
      }
    }
    Ok(())
  }

  fn get_sp_env() -> Result<Env> {
    let mut env = Env::empty()?;
    env.set(param::OutputFlag, 0)?;
    Ok(env.start()?)
  }

  #[test]
  fn solve_all() -> Result<()> {
    let _g = crate::logging::init_test_logging(None::<&str>);
    let env = get_sp_env()?;


    for di in 0..60 {
      let _s = error_span!("solve_all", data_idx=di).entered();
      let td = load(di)?;
      for si in 0..td.solutions.len() {
        let _s = error_span!("sol", sol_idx=si).entered();
        info!("comparing...");
        compare_graph_algo_with_lp_model(&td, si, &env, false)?;
      }
    }
    Ok(())
  }


  #[test]
  fn solve_one() -> Result<()> {
    let _g = crate::logging::init_test_logging(None::<&str>);
    let env = get_sp_env()?;

    let td = load(42)?;
    compare_graph_algo_with_lp_model(&td, 18, &env, true)?;
    Ok(())
  }
}