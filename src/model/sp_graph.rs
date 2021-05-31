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

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
enum LabelState {
  Unvisited,
  Queue,
  Processed,
}

#[derive(Debug, Clone)]
pub struct Node {
  idx: usize,
  avg: Avg,
  task: Task,
  time: Time,
  active_pred: Option<usize>,
  label_state: LabelState,
  #[cfg(debug_assertions)]
  processed: bool,
}

impl Node {
  pub fn new(idx: usize, avg: Avg, task: Task) -> Self {
    Node {
      idx,
      avg,
      task,
      time: task.t_release,
      active_pred: None,
      label_state: LabelState::Unvisited,
      #[cfg(debug_assertions)]
      processed: false,
    }
  }

  pub fn reset(&mut self) {
    self.time = self.time_lb();
    self.active_pred = None;
    self.label_state = LabelState::Unvisited;
    #[cfg(debug_assertions)] {
      self.processed = false;
    }

  }
}

impl Node {
  #[inline]
  pub fn time_ub(&self) -> Time { self.task.t_deadline - self.task.tt }
  #[inline]
  pub fn time_lb(&self) -> Time { self.task.t_release }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum EdgeKind {
  // Passive vehicle unloading time
  Unloading,
  // Passive vehicle loading time
  Loading,
  // Active vehicle travel time
  AVTravel,
}

#[derive(Debug, Copy, Clone)]
pub struct Edge {
  from: usize,
  to: usize,
  kind: EdgeKind,
  weight: Time,
}

#[derive(Clone, Debug)]
pub struct Graph {
  edges_from_node: Vec<SmallVec<[Edge; 2]>>,
  edges_to_node: Vec<SmallVec<[Edge; 2]>>,
  nodes: Vec<Node>,
  sources: Vec<usize>,
  sinks: Vec<usize>,
}

impl Graph {
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
    dot::LabelText::escaped(format!(
      "{:?} ({})\nt={}\n[{}, {}]",
      &n.task, n.idx, n.time, n.time_lb(), n.time_ub())
    )
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
  }

  #[cfg(debug_assertions)]
  fn node_color(&'a self, n: &Node) -> Option<dot::LabelText<'a>> {
    if n.processed {
      Some(dot::LabelText::label("royalblue"))
    } else {
      Some(dot::LabelText::label("grey"))
    }
  }

  fn kind(&self) -> dot::Kind { dot::Kind::Digraph }
}


#[tracing::instrument(level = "debug", skip(data, tasks, sol))]
pub fn build_graph(data: &Data, tasks: &Tasks, sol: &Solution) -> Graph {
  let mut nodes = Vec::with_capacity(sol.av_routes.iter().map(|(_, route)| route.len()).sum());
  let mut task_to_node_idx = map_with_capacity(nodes.len());
  let mut sources = Vec::with_capacity(sol.av_routes.len());
  let mut sinks = sources.clone();


  for (avg, route) in &sol.av_routes {
    sources.push(nodes.len());
    // trim the depots from the routes
    for &task in &route[1..route.len() - 1] {
      task_to_node_idx.insert(task, nodes.len());
      nodes.push(Node::new(nodes.len(), *avg, task));
    }
    sinks.push(nodes.len() - 1); // last node we pushed was a DDp node
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

  trace!(?edges_from_node, ?edges_to_node, ?nodes, ?sources, ?sinks, "built");
  Graph {
    edges_from_node,
    edges_to_node,
    nodes,
    sources,
    sinks,
  }
}

#[derive(Debug, Clone)]
pub enum SubproblemStatus {
  Infeasible(usize),
  Optimal(Map<(Avg, Task), Time>),
}

#[derive(Debug, Copy, Clone)]
pub struct NodePriority {
  node_idx: usize,
  time: Time,
}

impl Ord for NodePriority {
  fn cmp(&self, other: &Self) -> Ordering {
    self.time.cmp(&other.time)
  }
}

impl PartialOrd for NodePriority {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}

impl PartialEq for NodePriority {
  fn eq(&self, other: &Self) -> bool {
    self.time == other.time
  }
}

impl Eq for NodePriority {}

/// A bad priority queue where every operation is O(n).  It is, however, cache-friendly and branch-prediction friendly.
/// Because only a few nodes in the network are "active" nodes and in the queue, n is usually small (2-10)
/// (node, priority) pairs are stored in a list with gaps.
#[derive(Clone)]
struct StupidQueue {
  items: Vec<Option<NodePriority>>
}

impl Debug for StupidQueue {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    let mut list = f.debug_list();
    for i in &self.items {
      match i {
        Some(np) => list.entry(&(np.node_idx, np.time)),
        none => list.entry(none),
      };
    }
    list.finish()
  }
}

impl StupidQueue {
  pub fn initialise(init_cap: usize, iter: impl Iterator<Item=NodePriority>) -> Self {
    let mut items = Vec::with_capacity(init_cap);
    items.extend(iter.map(Some));
    StupidQueue { items }
  }

  pub fn update_priority(&mut self, np: NodePriority) -> bool {
    let mut free_slot = None;
    for (k, other) in self.items.iter_mut().enumerate() {
      if let Some(other) = other {
        if other.node_idx == np.node_idx {
          *other = np;
          return false;
        }
      } else if free_slot.is_none() {
        free_slot = Some(k);
      }
    }
    if let Some(k) = free_slot {
      self.items[k] = Some(np);
    } else {
      self.items.push(Some(np));
    }
    true
  }

  fn pop_min(&mut self) -> Option<NodePriority> {
    let min = self.items.iter()
      .enumerate()
      .filter_map(|(k, item)|
        item.as_ref().map(move |np| (k, np.time))
      )
      .min_by_key(|pair| pair.1);

    match min {
      Some((k, _)) => self.items[k].take(),
      None => None,
    }
  }
}

#[tracing::instrument(level = "debug", skip(graph))]
pub fn forward_label(graph: &mut Graph) -> SubproblemStatus {
  let mut queue = StupidQueue::initialise(
    graph.sources.len() * 2,
    graph.sources.iter().map(|&n| NodePriority { node_idx: n, time: graph.nodes[n].time }),
  );

  let nodes = &mut graph.nodes;
  let edges_from_node = &graph.edges_from_node;

  while let Some(NodePriority { node_idx, time }) = queue.pop_min() {
    let current_node = &mut nodes[node_idx];
    trace!(?queue, node_idx, time, task=?current_node.task, lb=current_node.time_lb(), "process node");
    #[cfg(debug_assertions)] {
      current_node.processed = true;
    }

    for edge in &edges_from_node[node_idx] {
      let next_node = &mut nodes[edge.to];
      let new_time = time + edge.weight;

      if new_time > next_node.time {
        // update neighbouring label's time
        next_node.time = new_time;
        next_node.active_pred = Some(node_idx);
        // check feasibility
        if next_node.time > next_node.time_ub() {
          debug!(node_idx = next_node.idx, time = next_node.time, ub = next_node.time_ub(), "infeasibility found");
          return SubproblemStatus::Infeasible(next_node.idx);
        }
      }

      // mark neighbour to be processed
      next_node.label_state = LabelState::Queue;
      queue.update_priority(NodePriority { node_idx: next_node.idx, time: next_node.time });
      trace!(?queue, node_idx=next_node.idx, time=next_node.time, task=?next_node.task, "push node");
    }
  }

  let theta_val = graph.sinks.iter()
    .map(|&idx| {
      let n = &graph.nodes[idx];
      ((n.avg, n.task), n.time)
    })
    .collect();
  debug!(?theta_val, "subproblem optimal");
  SubproblemStatus::Optimal(theta_val)
}

type BlEdgeList<'a> = SmallVec<[Edge; 10]>;

#[tracing::instrument(level = "debug", skip(graph, start_node), fields(start_node))]
fn extract_critical_paths(graph: &Graph, start_node: usize) -> BlEdgeList {
  // Reconstruct the path
  let mut node = &graph.nodes[start_node];
  let mut edges = BlEdgeList::new();

  while let Some(pred_idx) = node.active_pred {
    let edge = 'outer: loop {
      for e in &graph.edges_to_node[node.idx] {
        if e.from == pred_idx {
          break 'outer *e;
        }
      }
      unreachable!()
    };
    edges.push(edge);
    node = &graph.nodes[pred_idx];
  }
  edges
}

fn remove_path(graph: &mut Graph, edges: &[Edge]) {
  for e in edges {
    graph.remove_edge(&e);
  }

  let affected_nodes = std::iter::once(edges[0].from)
    .chain(edges.iter().map(|e| e.to));

  for n in affected_nodes {
    if graph.edges_to_node[n].len() == 0 && graph.edges_from_node[n].len() > 0 {
      graph.sources.push(n);
    }
  }
}


#[derive(Debug, Copy, Clone)]
enum IisMember {
  AvTravelTime(Task, Task),
  Loading(Task),
  Unloading(Task),
  Ub(Task),
  Lb(Task),
}

fn extract_and_remove_iis(graph: &mut Graph, ub_violate_node: usize) -> Vec<IisMember> {
  let edges = extract_critical_paths(graph, ub_violate_node);
  remove_path(graph, &edges);
  for n in graph.nodes.iter_mut() {
    n.reset();
  }

  let mut iis = Vec::with_capacity(edges.len());
  for edge in &edges {
    let member = match edge.kind {
      EdgeKind::AVTravel => IisMember::AvTravelTime(graph.nodes[edge.from].task, graph.nodes[edge.to].task),
      EdgeKind::Loading => IisMember::Loading(graph.nodes[edge.from].task),
      EdgeKind::Unloading => IisMember::Unloading(graph.nodes[edge.to].task),
    };
    iis.push(member)
  }
  iis.push(IisMember::Lb(graph.nodes[edges.first().unwrap().from].task));
  iis.push(IisMember::Ub(graph.nodes[edges.last().unwrap().to].task));
  iis
}


#[cfg(test)]
mod tests {
  use tracing::*;
  use super::*;
  use super::super::sp_lp::TimingSubproblem;
  use grb::Env;
  use std::fmt::Debug;

  fn load(idx: usize) -> Result<(Data, Sets, Tasks, Vec<Solution>)> {
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

    Ok((data, sets, tasks, solutions?))
  }

  fn compare_graph_algo_with_lp_model(data: &Data, tasks: &Tasks, sol: &Solution, env: &Env) -> Result<()> {
    let mut graph = build_graph(data, tasks, sol);
    let graph_result = forward_label(&mut graph);
    println!("{:?}", &graph_result);
    let mut lp = TimingSubproblem::build(env, data, tasks, sol)?;
    lp.model.optimize()?;

    fn print_constr_info<'a, K: 'a + Debug>(model: &grb::Model, cgroup: &str, constraints: impl IntoIterator<Item=(&'a K, &'a Constr)>) {
      for (key, c) in constraints {
        if model.get_obj_attr(attr::IISConstr, c).unwrap() > 0 {
          let rhs = model.get_obj_attr(attr::RHS, c).unwrap();
          error!(%cgroup, constr=?key, ?rhs, "IIS constr")
        }
      }
    }

    fn print_iis(lp: &mut TimingSubproblem) {
      lp.model.compute_iis().unwrap();
      print_constr_info(&lp.model, "loading", &lp.cons.loading);
      print_constr_info(&lp.model, "unloading", &lp.cons.unloading);
      print_constr_info(&lp.model, "ub", &lp.cons.ub);
      print_constr_info(&lp.model, "lb", &lp.cons.lb);
      print_constr_info(&lp.model, "av_sync", &lp.cons.av_sync);
    }

    match &graph_result {
      SubproblemStatus::Infeasible(_) => graph.save_as_svg("debug-infeasible.svg")?,
      SubproblemStatus::Optimal(_) => graph.save_as_svg("debug-optimal.svg")?,
    };

    match (graph_result, lp.model.status()?) {
      (SubproblemStatus::Infeasible(node), Status::Infeasible) => {
        print_iis(&mut lp);
        let iis = extract_and_remove_iis(&mut graph, node);
        println!("{:?}", iis);
        let result = forward_label(&mut graph);
        dbg!(result);
        graph.save_as_svg("debug-2nd.svg")?;
        println!("infeasible");
        panic!();
      }

      (SubproblemStatus::Optimal(_), Status::Optimal) => {
        println!("optimal");
        // panic!()
      }

      (a, b) => {
        if b == Status::Infeasible {
          print_iis(&mut lp);
        }

        graph.save_as_svg("debug.svg")?;
        panic!("mismatch!\ngraph algo gave {:?}\nLP gave {:?}", a, b)
      }
    }
    Ok(())
  }


  #[test]
  fn solve() -> Result<()> {
    let _g = crate::logging::init_test_logging(None::<&str>);
    let _s = trace_span!("test").entered();
    let (data, sets, tasks, solutions) = load(0)?;
    let env = Env::new("gurobi.log")?;
    for (n, sol) in solutions.iter().skip(5).enumerate() {
      info!(n, "running on solution");
      compare_graph_algo_with_lp_model(&data, &tasks, sol, &env)?;
    }
    Ok(())
  }
}