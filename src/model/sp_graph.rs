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
use dot::LabelText;

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
        last_task_tt_to_depot.push(last_task.tt + data.travel_time[&(last_task.end, Loc::Ad)])
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

  fn get_edge(&self, from: usize, to: usize) -> &Edge {
    for e in &self.edges_from_node[from] {
      if e.to == to {
        return e
      }
    }
    unreachable!()
  }

  #[tracing::instrument(level = "debug", skip(self))]
  fn backlabel_min_iis(&self, ub_violated_nodes: &[usize]) -> BlEdgeList {

    type Action = usize;
    use std::collections::hash_map::Entry;

    #[tracing::instrument(level="trace", skip(graph, cache))]
    fn backlabel_dp(graph: &Graph, cache: &mut Map<(usize, Time), Option<(u16, Action)>>, node: usize, time: Time) -> Option<(u16, Action)> {
      // Base cases
      let n = &graph.nodes[node];
      if time > n.time_ub() {
        trace!("no iis");
        return None
      } else if time < n.time_lb() {
        trace!("found iis");
        return Some((0, node))
      }

      // Cached Non-base cases
      if let Some(val) = cache.get(&(node, time)) {
        trace!(?val, "cache hit");
        return *val
      }

      // Compute non-base case
      let mut best_val = u16::MAX;
      let mut best_action = None;

      for edge in &graph.edges_to_node[node] {
        if let Some((mut val, _)) = backlabel_dp(graph, cache, edge.from, time - edge.weight) {
          val += 1;
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


    let mut cache = map_with_capacity(self.nodes.len()*2);
    let mut edge_list = BlEdgeList::new();

    let (_, mut pred_node, mut node, mut time) = ub_violated_nodes.iter()
      .filter_map(|&node| {
        let time = self.nodes[node].time_ub();
        backlabel_dp(self, &mut cache, node, time)
          .map(|(path_len, pred)| (path_len, pred, node, time))
        }
      )
      .min_by_key(|(path_len, ..)| *path_len)
      .expect("should find at least one IIS");

    while node != pred_node {
      trace!(pred_node, node);
      let e = *self.get_edge(pred_node, node);
      time -= e.weight;
      edge_list.push(e);
      node = pred_node;
      pred_node = backlabel_dp(self, &mut cache, node, time).unwrap().1;
    }

    trace!(?edge_list);
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
    edge_list
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
    // if self.edge_active(e) {
    //   Some(dot::LabelText::label("red"))
    // } else {
    //   Some(dot::LabelText::label("black"))
    // }
    None
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
    Some(LabelText::LabelStr(color.into()))
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

impl SpSolve for Graph {
  type OptInfo = ();
  /// Node index at which the time window is violated.
  type InfInfo = Vec<usize>;

  fn solve(&mut self) -> Result<SpStatus<Self::OptInfo, Self::InfInfo>> {
    let mut queue = SmallVec::<[usize; NUM_AV_UB * 2]>::new();
    queue.extend_from_slice(&self.sources);
    trace!(?queue, "init");
    let mut num_visited_pred = vec![0usize; self.nodes.len()];
    let mut violated_ub_nodes =  Vec::new();
    let nodes = &mut self.nodes;
    let edges_from_node = &self.edges_from_node;

    #[cfg(debug_assertions)]
      let mut order = 0;


    while let Some(node_idx) = queue.pop() {
      let current_node = &mut nodes[node_idx];
      trace!(?queue, node_idx, time=current_node.time, task=?current_node.task, lb=current_node.time_lb(), "pop node");
      #[cfg(debug_assertions)] {
        current_node.forward_order = order;
        order += 1;
      }
      let time = current_node.time;

      for edge in &edges_from_node[node_idx] {
        let visited_pred = &mut num_visited_pred[edge.to];
        let next_node = &mut nodes[edge.to];
        let new_time = time + edge.weight;

        if next_node.time < new_time {
          // update neighbouring label's time
          next_node.time = new_time;
          next_node.active_pred = Some(node_idx);
          // check feasibility
          if next_node.time > next_node.time_ub() {
            violated_ub_nodes.push(edge.to);
            debug!(node_idx = next_node.idx, time = next_node.time, ub = next_node.time_ub(), "infeasibility found");
            // return Ok(SpStatus::Infeasible(next_node.idx));
          }
        }

        // If the next node has had all its predecessors processed, add it to the queue
        *visited_pred += 1;
        if *visited_pred == self.edges_to_node[edge.to].len() {
          queue.push(edge.to);
          trace!(?queue, node_idx=next_node.idx, time=next_node.time, task=?next_node.task, "push node");
        }
      }
    }

    let obj = self.last_task_nodes.iter()
      .zip(&self.last_task_tt_to_depot)
      .map(|(&n, &tt)| self.nodes[n].time + tt)
      .sum();

    if violated_ub_nodes.is_empty() {
      debug!(obj, "subproblem optimal");
      Ok(SpStatus::Optimal(obj, ()))
    } else {
      Ok(SpStatus::Infeasible(violated_ub_nodes))
    }

  }

  fn extract_and_remove_iis(&mut self, ub_violate_nodes: Vec<usize>) -> Result<SpConstraints> {
    // edges are returned in reverse order, so edge.last().unwrap().from is the first node along the IIS
    // path and `edges[0].to` is the node indexed by `ub_violate_node`.
    let edges = self.backlabel_min_iis(&ub_violate_nodes);

    for e in &edges {
      self.remove_edge(e);
    }

    { // update source nodes
      let iis_start_node = edges.last().expect("IIS should contain at least one non-bound constraint").from;
      let iis_start_node_is_source = self.sources.contains(&iis_start_node);
      let edges_to_node = &self.edges_to_node;
      let edges_from_node = &self.edges_from_node;

      let new_source_nodes = edges.iter().map(|e| e.to)
        // if the start node of the IIS path is the source node, don't add it again
        .chain(std::iter::once(iis_start_node).take((!iis_start_node_is_source) as usize))
        // new source nodes should not be isolated
        .filter(|&n| edges_to_node[n].len() == 0 && edges_from_node[n].len() > 0);

      self.sources.extend(new_source_nodes);
    }

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

  fn compare_graph_algo_with_lp_model(td: &TestData, sidx: usize, env: &Env) -> Result<()> {
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
      // graph.backlabel_min_iis(20);
      let lp_result = lp.solve()?;

      match &graph_result {
        SpStatus::Infeasible(..) => graph.save_as_svg(format!("debug-inf-{}.svg", iter))?,
        SpStatus::Optimal(..) => graph.save_as_svg(format!("debug-opt-{}.svg", iter))?,
      };

      // lp.model.write(&format!("sp-dbg-{}.lp", iter))?;

      match (graph_result, lp_result) {
        (SpStatus::Optimal(g_obj, _), SpStatus::Optimal(lp_obj, _)) => {
          // print_lp_soln(&lp);
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
          print_lp_iis(&mut lp);
          let mut lp_iis = lp.extract_and_remove_iis(())?;
          lp_iis.sort();

          if g_iis != lp_iis {
            warn!(?g_iis, ?lp_iis, "IIS divergence");
            break;
          }
        }

        (a, b) => {
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


    for di in 40..60 {
      let _s = error_span!("solve_all", data_idx=di).entered();
      let td = load(di)?;
      for si in 0..td.solutions.len() {
        let _s = error_span!("sol", sol_idx=si).entered();
        info!("comparing...");
        compare_graph_algo_with_lp_model(&td, si, &env)?;
      }
    }
    Ok(())
  }


  #[test]
  fn solve_one() -> Result<()> {
    let _g = crate::logging::init_test_logging(None::<&str>);
    let env = get_sp_env()?;

    let td = load(40)?;
    compare_graph_algo_with_lp_model(&td, 12, &env)?;
    Ok(())
  }
}