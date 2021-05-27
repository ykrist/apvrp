use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use super::mp::MpVars;
use super::cb::{CbError};
use tracing::{error_span, debug, trace, error, debug_span};
use itertools::Itertools;
use crate::solution::*;
use std::collections::{HashSet, HashMap, hash_map::Entry, BinaryHeap};
use std::hash::{Hash, BuildHasher};
use crate::utils::HashMapExt;
use smallvec::SmallVec;
use std::cmp::{PartialOrd, Eq, PartialEq, Ord, Ordering, max};
use std::path::Path;

#[derive(Debug, Clone)]
pub struct Node {
  idx: usize,
  avg: Avg,
  task: Task,
  time: Time,
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

#[derive(Debug, Clone)]
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
  fn nodes(&'a self) -> dot::Nodes<'a, Node> { dot::Nodes::Borrowed(&self.nodes) }

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
    // dot::LabelText::label(format!("{:?}", &n.task))
    dot::LabelText::escaped(format!(
      "{:?}\nt={}\n[{}, {}]",
      &n.task, n.time, n.time_lb(), n.time_ub())
    )
  }

  fn edge_label(&'a self, e: &Edge) -> dot::LabelText<'a> {
    // dot::LabelText::label(format!("{:?}", &e.weight))
    dot::LabelText::escaped(format!("{:?}\n{:?}", &e.weight, e.kind))
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
      nodes.push(Node {
        idx: nodes.len(),
        avg: *avg,
        task,
        time: task.t_release,
      })
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
    other.time.cmp(&self.time) // flip the ordering
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

#[tracing::instrument(level = "debug", skip(graph))]
pub fn forward_label(graph: &mut Graph) -> SubproblemStatus {
  let mut queue = BinaryHeap::with_capacity(graph.nodes.len());
  for &n in &graph.sources {
    queue.push(NodePriority { node_idx: n, time: graph.nodes[n].time });
  }
  let nodes = &mut graph.nodes;
  let edges_from_node = &graph.edges_from_node;

  while let Some(NodePriority { node_idx, time }) = queue.pop() {
    trace!(?queue, node_idx, time, task=?nodes[node_idx].task, lb=nodes[node_idx].time_lb(), "process node");
    for edge in &edges_from_node[node_idx] {
      let node = &mut nodes[edge.to];
      node.time = max(node.time, time + edge.weight); // TODO maybe be clever here?
      if node.time > node.time_ub() {
        debug!(node_idx=node.idx, time=node.time, ub=node.time_ub(), "infeasibility found");
        return SubproblemStatus::Infeasible(node.idx);
      }
      queue.push(NodePriority { node_idx: node.idx, time: node.time });
      trace!(?queue, node_idx=node.idx, time=node.time, task=?node.task, "push node");
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

#[cfg(test)]
mod tests {
  use tracing::*;
  use super::*;
  use super::super::sp::TimingSubproblem;
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


    match (graph_result, lp.model.status()?) {
      (SubproblemStatus::Infeasible(_), Status::Infeasible) => {
        print_iis(&mut lp);
        graph.save_as_svg("debug.svg")?;
        println!("infeasible");
        panic!();
      },

      (SubproblemStatus::Optimal(_), Status::Optimal) => {
        println!("optimal")
      },

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
    let (data, sets, tasks, solutions) = load(0)?;
    let env = Env::new("gurobi.log")?;
    for (n, sol) in solutions.iter().skip(1).enumerate() {
      info!(n, "running on solution");
      compare_graph_algo_with_lp_model(&data, &tasks, sol, &env)?;
    }
    Ok(())
  }
}