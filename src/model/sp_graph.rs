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
  #[cfg(debug_assertions)]
  forward_labelled: bool,
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
      forward_labelled: false,
    }
  }

  pub fn reset(&mut self) {
    self.time = self.time_lb();
    self.active_pred = None;
    #[cfg(debug_assertions)] {
      self.forward_labelled = false;
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

type BlEdgeList<'a> = SmallVec<[Edge; 10]>;

#[derive(Clone, Debug)]
pub struct Graph {
  edges_from_node: Vec<SmallVec<[Edge; 2]>>,
  edges_to_node: Vec<SmallVec<[Edge; 2]>>,
  nodes: Vec<Node>,
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
    let mut sources = Vec::with_capacity(sol.av_routes.len());
    let mut last_task_nodes = sources.clone();
    let mut last_task_tt_to_depot = Vec::with_capacity(sol.av_routes.len());

    for (avg, route) in &sol.av_routes {
      sources.push(nodes.len());
      // trim the depots from the routes
      for &task in &route[1..route.len() - 1] {
        task_to_node_idx.insert(task, nodes.len());
        nodes.push(Node::new(nodes.len(), *avg, task));
      }

      {
        let last_task: &Task = &nodes.last().unwrap().task;
        last_task_tt_to_depot.push( last_task.tt + data.travel_time[&(last_task.end, Loc::Ad)])
      }
      last_task_nodes.push(nodes.len() - 1); // last node we pushed was the one before the DDp node

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

    trace!(?edges_from_node, ?edges_to_node, ?nodes, ?sources, ?last_task_nodes, "built");
    Graph {
      edges_from_node,
      edges_to_node,
      nodes,
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

  #[tracing::instrument(level = "debug", skip(self))]
  fn backlabel_iis(&self, ub_violate: usize) -> BlEdgeList {
    let mut edge_list = BlEdgeList::new();
    // Reconstruct the path from this node
    let mut node = &self.nodes[ub_violate];

    while let Some(pred_idx) = node.active_pred {
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
    edge_list
  }

  #[tracing::instrument(level = "debug", skip(self))]
  fn backlabel_mrs(&self, last_task_nodes: &[usize]) -> BlEdgeList {
    let mut edge_list = BlEdgeList::new();
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
          if *bl { break }
          else { *bl = true }
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
    edge_list
  }

  fn get_sp_constraint(&self, edge: &Edge) -> SpConstr {
    match edge.kind {
      EdgeKind::AVTravel => SpConstr::AvTravelTime(self.nodes[edge.from].task, self.nodes[edge.to].task),
      EdgeKind::Loading => SpConstr::Loading(self.nodes[edge.from].task),
      EdgeKind::Unloading => SpConstr::Unloading(self.nodes[edge.to].task),
    }
  }

  fn remove_edges_and_clean(&mut self, edges: &[Edge]) {
    for e in edges {
      self.remove_edge(&e);
    }

    let affected_nodes = std::iter::once(edges[0].from)
      .chain(edges.iter().map(|e| e.to));

    for n in affected_nodes {
      if self.edges_to_node[n].len() == 0 && self.edges_from_node[n].len() > 0 {
        self.sources.push(n);
      }
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
    if n.forward_labelled {
      Some(dot::LabelText::label("royalblue"))
    } else {
      Some(dot::LabelText::label("grey"))
    }
  }

  fn kind(&self) -> dot::Kind { dot::Kind::Digraph }
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

impl SpSolve for Graph {
  type OptInfo = ();
  /// Node index at which the time window is violated.
  type InfInfo = usize;

  fn solve(&mut self) -> Result<SpStatus<(), usize>> {
    let mut queue = StupidQueue::initialise(
      self.sources.len() * 2,
      self.sources.iter().map(|&n| NodePriority { node_idx: n, time: self.nodes[n].time }),
    );

    let nodes = &mut self.nodes;
    let edges_from_node = &self.edges_from_node;

    while let Some(NodePriority { node_idx, time }) = queue.pop_min() {
      let current_node = &mut nodes[node_idx];
      trace!(?queue, node_idx, time, task=?current_node.task, lb=current_node.time_lb(), "process node");
      #[cfg(debug_assertions)] {
        current_node.forward_labelled = true;
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
            return Ok(SpStatus::Infeasible(next_node.idx));
          }
        }

        // mark neighbour to be processed
        queue.update_priority(NodePriority { node_idx: next_node.idx, time: next_node.time });
        trace!(?queue, node_idx=next_node.idx, time=next_node.time, task=?next_node.task, "push node");
      }
    }

    let obj = self.last_task_nodes.iter()
      .zip(&self.last_task_tt_to_depot)
      .map(|(&n, &tt)| self.nodes[n].time + tt)
      .sum();

    debug!(obj, "subproblem optimal");
    Ok(SpStatus::Optimal(obj, ()))
  }

  fn extract_and_remove_iis(&mut self, ub_violate_node: usize) -> Result<SpConstraints> {
    let edges = self.backlabel_iis(ub_violate_node);
    self.remove_edges_and_clean(&edges);

    for n in self.nodes.iter_mut() {
      n.reset();
    }

    let mut iis = SpConstraints::with_capacity(edges.len() + 2);
    for edge in &edges {
      iis.push(self.get_sp_constraint(edge))
    }

    iis.push(SpConstr::Ub(self.nodes[edges.first().unwrap().to].task));
    iis.push(SpConstr::Lb(self.nodes[edges.last().unwrap().from].task));
    Ok(iis)
  }

  fn extract_mrs(&self, _: ()) -> Result<MrsInfo> {
    let edges = self.backlabel_mrs(&self.last_task_nodes);
    let mut mrs = SmallVec::with_capacity(edges.len() + self.last_task_nodes.len());

    for e in &edges {
      mrs.push(self.get_sp_constraint(e));
    }

    for &n in &self.last_task_nodes {
      mrs.push(SpConstr::AvTravelTime(self.nodes[n].task, self.av_ddepot))
    }

    Ok(smallvec::smallvec![mrs])
  }
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
    let mut graph = Graph::build(data, tasks, sol);
    let mut lp = TimingSubproblem::build(env, data, tasks, sol)?;

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

    let mut g_iises = HashSet::<SpConstraints>::default();
    let mut lp_iises = HashSet::<SpConstraints>::default();

    loop {
      let graph_result = graph.solve()?;
      let lp_result = lp.solve()?;

      match &graph_result {
        SpStatus::Infeasible(..) => graph.save_as_svg("debug-infeasible.svg")?,
        SpStatus::Optimal(..) => graph.save_as_svg("debug-optimal.svg")?,
      };

      match (graph_result, lp_result) {
        (SpStatus::Optimal(g_obj, _), SpStatus::Optimal(lp_obj, _)) => {
          assert_eq!(g_obj, lp_obj);

          let g_mrs = &mut graph.extract_mrs(())?[0];
          g_mrs.sort();
          let lp_mrs = &mut graph.extract_mrs(())?[0];
          lp_mrs.sort();
          assert_eq!(g_mrs, lp_mrs);
          break;
        }

        (SpStatus::Infeasible(nidx), SpStatus::Infeasible(_)) => {
          let mut g_iis = graph.extract_and_remove_iis(nidx)?;
          g_iis.sort();
          let mut lp_iis = lp.extract_and_remove_iis(())?;
          lp_iis.sort();
          // assert_eq!(g_iis, lp_iis);
          g_iises.insert(g_iis);
          lp_iises.insert(lp_iis);
        }

        (a, b) => {
          panic!("mismatch!\ngraph algo gave {:?}\nLP gave {:?}", a, b)
        }
    }


    }
    Ok(())
  }



  #[test]
  fn solve() -> Result<()> {
    let _g = crate::logging::init_test_logging(None::<&str>);
    for di in 0..60 {
      let (data, sets, tasks, solutions) = load(di)?;
      let env = Env::new("gurobi.log")?;
      for (si, sol) in solutions.iter().enumerate() {
        let _s = info_span!("solve_test", data_idx=di, sol_idx=si).entered();
        compare_graph_algo_with_lp_model(&data, &tasks, sol, &env)?;
      }
    }
    Ok(())
  }
}