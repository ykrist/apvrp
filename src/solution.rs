use crate::*;
use crate::model::sp::{lp::TimingSubproblem, Subproblem};
use serde::{Serialize, Deserialize};
use std::fs::read_to_string;
use std::path::Path;
use anyhow::Context;
use tracing::trace;
use crate::model::mp::{MpVars, TaskModelMaster, ObjWeights};
use grb::callback::MIPSolCtx;
use grb::prelude::*;
use std::hash::Hash;
use crate::graph::DecomposableDigraph;
use crate::utils::IoContext;
use std::assert_matches::*;


/// A Passive vehicle route, beginning at a PV origin depot and ending at a PV destination depot
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct PvRoute(Vec<PvTask>);

impl PvRoute {
  fn new(path: Vec<PvTask>) -> Self {
    debug_assert_matches!(path.first().unwrap().ty, TaskType::Start);
    debug_assert_matches!(path.last().unwrap().ty, TaskType::End);
    Self(path)
  }

  fn iter_edges<'a>(&'a self) -> impl Iterator<Item=(&'a PvTask, &'a PvTask)> + 'a {
    self.0.iter().tuple_windows()
  }

  fn iter_edges_owned<'a>(&'a self) -> impl Iterator<Item=(PvTask, PvTask)> + 'a {
    self.0.iter().copied().tuple_windows()
  }
}


/// A AV vehicle route, beginning at the AV origin depot and ending the AV destination depot
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct AvRoute(Vec<Task>);

/// A PV cycle, first task is equal to the last task
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct PvCycle(Vec<PvTask>);

/// An AV cycle, first task is equal to the last task
#[derive(Debug, Clone, Eq, PartialEq)]
pub struct AvCycle(Vec<Task>);


/// A (possibly cyclic) Active Vehicle path.  If the path is a cycle, the first and last Tasks are the same.
pub(crate) type AvPath = Vec<Task>;
/// A (possibly cyclic) Passive Vehicle path.  If the path is a cycle, the first and last Tasks are the same.
pub(crate) type PvPath = Vec<PvTask>;

pub fn iter_solution_log(p: impl AsRef<Path>) -> Result<impl Iterator<Item=SerialisableSolution>> {
  let contents = std::io::Cursor::new(std::fs::read(&p).read_context(&p)?);
  let stream = serde_json::Deserializer::from_reader(contents)
    .into_iter()
    .map(|r| r.expect("failure parsing file"));
  Ok(stream)
}

#[inline]
fn discard_edge_weights<T, W>(pairs: Vec<(T, W)>) -> Vec<T> {
  pairs.into_iter().map(|(x, _)| x).collect()
}

#[inline]
fn to_path_nodes<N: Copy, W>(pairs: &[(Vec<(N, N)>, W)]) -> Vec<Vec<N>> {
  pairs.iter()
    .map(|(arcs, _)| {
      std::iter::once(arcs[0].0)
        .chain(arcs.iter().map(|(_, n)| *n))
        .collect()
    })
    .collect()
}


/// Given a list of task pairs (from the Y variables), construct routes for an Active Vehicle class, plus any cycles which occur.
#[tracing::instrument(level="debug", skip(task_pairs))]
pub fn construct_av_routes(task_pairs: &[(Task, Task)]) -> (Vec<AvPath>, Vec<AvPath>) {
  trace!(?task_pairs);
  use crate::graph::DecomposableDigraph;

  #[derive(Debug)]
  struct AvGraph {
    pub odepot_arcs: FnvHashSet<(Task, Task)>,
    pub succ: Map<Task, Task>,
  }

  impl DecomposableDigraph<Task, (Task, Task), u8> for AvGraph {
    fn is_sink(&self, node: &Task) -> bool { node.ty == TaskType::DDepot }

    #[tracing::instrument(level = "trace", name = "find_start", fields(succ = ? & self.succ, od_arcs = ? & self.odepot_arcs), skip(self))]
    fn next_start(&self) -> Option<Task> {
      self.odepot_arcs.iter().next()
        .map(|(od, _)| *od)
        .or_else(|| {
          trace!("no start arcs left");
          self.succ.keys().next().copied()
        })
    }

    #[tracing::instrument(level = "trace", name = "find_succ", fields(succ = ? & self.succ, od_arcs = ? & self.odepot_arcs), skip(self))]
    fn next_outgoing_arc(&self, task: &Task) -> ((Task, Task), Task, u8) {
      let arc = if task.ty == TaskType::ODepot {
        *self.odepot_arcs.iter().next().unwrap()
      } else {
        (*task, self.succ[task])
      };
      trace!(next_task=?arc.1);
      (arc, arc.1, 1)
    }

    #[tracing::instrument(level = "trace", fields(succ = ? & self.succ, od_arcs = ? & self.odepot_arcs), skip(self))]
    fn subtract_arc(&mut self, arc: &(Task, Task), _: u8) {
      let value_removed =
        if arc.0.ty == TaskType::ODepot {
          trace!("remove depot arc");
          self.odepot_arcs.remove(arc)
        } else {
          trace!("remove non-depot arc");
          self.succ.remove(&arc.0).is_some()
        };
      debug_assert!(value_removed)
    }
  }

  let graph = {
    let mut odepot_arcs = FnvHashSet::default();
    let mut succ = Map::default();
    for (t1, t2) in task_pairs {
      if t1.ty == TaskType::ODepot {
        odepot_arcs.insert((*t1, *t2));
      } else {
        succ.insert(*t1, *t2);
      }
    }
    AvGraph { odepot_arcs, succ }
  };

  let (paths, cycles) = graph.decompose_paths_cycles();
  (to_path_nodes(&paths), to_path_nodes(&cycles))
}

/// Given a list of tasks, construct the route for a Passive vehicle, plus any cycles which occur.
#[tracing::instrument(level="debug", skip(tasks_used))]
pub fn construct_pv_route(tasks_used: &[PvTask]) -> (PvPath, Vec<PvPath>) {
  trace!(?tasks_used);
  debug_assert!(!tasks_used.is_empty());
  let first_task = *tasks_used.first().unwrap();

  if tasks_used.len() == 1 {
    trace!(task=?first_task, "single-task route");
    debug_assert_eq!(first_task.ty, TaskType::Direct);
    return (vec![first_task], vec![]);
  }

  struct PvGraph {
    pub pv_origin: Loc,
    pub pv_dest: Loc,
    pub task_by_start: Map<Loc, PvTask>,
  }

  impl DecomposableDigraph<Loc, PvTask, u8> for PvGraph {
    fn is_sink(&self, node: &Loc) -> bool { node == &self.pv_dest }

    fn next_start(&self) -> Option<Loc> {
      if self.task_by_start.contains_key(&self.pv_origin) { Some(self.pv_origin) } else { self.task_by_start.keys().next().copied() }
    }

    fn next_outgoing_arc(&self, node: &Loc) -> (PvTask, Loc, u8) {
      let task = self.task_by_start[node];
      (task, task.end, 1)
    }
    // s
    fn subtract_arc(&mut self, arc: &PvTask, _: u8) {
      let removed = self.task_by_start.remove(&arc.start);
      debug_assert!(removed.is_some())
    }
  }

  let pv_origin = Loc::Po(first_task.p);
  let graph = PvGraph {
    pv_origin,
    pv_dest: pv_origin.dest(),
    task_by_start: tasks_used.iter().map(|&t| (t.start, t)).collect(),
  };

  let (pv_paths, pv_cycles) = graph.decompose_paths_cycles();

  debug_assert_eq!(pv_paths.len(), 1);
  let pv_path = discard_edge_weights(pv_paths).pop().unwrap();
  (pv_path, discard_edge_weights(pv_cycles))
}

pub trait QueryVarValues {
  fn get(&self, vars: impl Iterator<Item=Var>) -> Result<Vec<f64>>;
}

impl<'a> QueryVarValues for MIPSolCtx<'a> {
  #[inline]
  fn get(&self, vars: impl Iterator<Item=Var>) -> Result<Vec<f64>> {
    Ok(self.get_solution(vars)?)
  }
}

impl<'a> QueryVarValues for Model {
  #[inline]
  fn get(&self, vars: impl Iterator<Item=Var>) -> Result<Vec<f64>> {
    Ok(self.get_obj_attr_batch(attr::X, vars)?)
  }
}

#[inline]
pub fn get_var_values<'a, M: QueryVarValues, K: Hash + Eq + Copy>(ctx: &M, var_dict: &'a Map<K, Var>) -> Result<impl Iterator<Item=(K, f64)> + 'a> {
  let vals = ctx.get(var_dict.values().copied())?;
  Ok(var_dict.keys().copied().zip(vals).filter(|(_, v)| v.abs() > 0.01))
}


#[inline]
pub fn get_var_values_mapped<'a, M, K, V, F>(ctx: &M, var_dict: &'a Map<K, Var>, map: F) -> Result<impl Iterator<Item=(K, V)> + 'a>
  where
    M: QueryVarValues,
    K: Hash + Eq + Copy,
    F: Fn(f64) -> V + 'a,
{
  let vals = ctx.get(var_dict.values().copied())?;
  let var_vals = var_dict.keys()
    .copied().zip(vals)
    .filter(|(_, v)| v.abs() > 0.01)
    .map(move |(k, v)| (k, map(v))
  );
  Ok(var_vals)
}


#[tracing::instrument(level = "trace", skip(ctx, yvars))]
pub fn get_task_pairs_by_av<M: QueryVarValues>(ctx: &M, yvars: &Map<(Avg, Task, Task), Var>) -> Result<Map<Avg, Vec<(Task, Task)>>> {
  let mut task_pairs = Map::default();

  for ((av, t1, t2), val) in get_var_values(ctx, yvars)? {
    trace!(av, ?t1, ?t2, ?val);
    task_pairs.entry(av).or_insert_with(Vec::new).push((t1, t2));
  }

  Ok(task_pairs)
}

#[tracing::instrument(level = "trace", skip(ctx, xvars))]
pub fn get_tasks_by_pv<M: QueryVarValues>(ctx: &M, xvars: &Map<PvTask, Var>) -> Result<Map<Pv, Vec<PvTask>>> {
  let mut tasks = Map::default();

  for (t, val) in get_var_values(ctx, xvars)? {
    trace!(?t, ?val);
    tasks.entry(t.p).or_insert_with(Vec::new).push(t);
  }

  Ok(tasks)
}


#[derive(Debug, Clone)]
pub struct Solution {
  /// Total objective cost (optional)
  pub objective: Option<Cost>,
  pub av_routes: Vec<(Avg, Vec<Task>)>,
  pub pv_routes: Vec<(Pv, Vec<PvTask>)>,
}

impl Solution {
  pub fn to_serialisable(&self) -> SerialisableSolution {
    let av_routes = self.av_routes.iter()
      .map(|(avg, tasks)| (*avg, tasks.iter().map(|t| ShorthandTask::from(*t)).collect()))
      .collect();
    let pv_routes = self.pv_routes.iter()
      .map(|(pv, tasks)| (*pv, tasks.iter().map(|t| ShorthandPvTask::from(*t)).collect()))
      .collect();

    SerialisableSolution {
      objective: self.objective,
      av_routes,
      pv_routes,
    }
  }

  pub fn from_mp(mp: &TaskModelMaster) -> Result<Self> {
    let mut av_routes = Vec::new();

    for (av, pairs) in get_task_pairs_by_av(&mp.model, &mp.vars.y)? {
      let (av_paths, av_cycles) = construct_av_routes(&pairs);
      if av_cycles.len() > 0 {
        anyhow::bail!("cycles in solution (av = {}, cycles = {:?})", av, &av_cycles)
      }
      av_routes.extend(av_paths.into_iter().map(|r| (av, r)));
    }

    let mut pv_routes = Vec::new();
    for (pv, tasks) in get_tasks_by_pv(&mp.model, &mp.vars.x)? {
      let (pv_path, pv_cycles) = construct_pv_route(&tasks);
      if pv_cycles.len() > 0 {
        anyhow::bail!("cycles in solution (pv = {}, cycles = {:?})", pv, &pv_cycles)
      }
      pv_routes.push((pv, pv_path));
    }

    Ok(Solution { objective: Some(mp.obj_bound()?), av_routes, pv_routes })
  }

  pub fn solve_for_times(&self, lu: &Lookups) -> Result<SpSolution> {
    let mut sp = TimingSubproblem::build(lu, self)?;
    sp.model.optimize()?;
    use grb::Status::*;
    match sp.model.status()? {
      Optimal => SpSolution::from_sp(&sp),
      status => Err(anyhow::anyhow!("unexpected subproblem status: {:?}", status)),
    }
  }
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerialisableSolution {
  pub objective: Option<Cost>,
  pub av_routes: Vec<(Avg, Vec<ShorthandTask>)>,
  pub pv_routes: Vec<(Pv, Vec<ShorthandPvTask>)>,
}

impl SerialisableSolution {
  pub fn to_solution(&self, tasks: impl AsRef<Tasks>) -> Solution {
    let tasks = tasks.as_ref();
    let av_routes = self.av_routes.iter()
      .map(|(avg, route)| {
        let route = route.iter()
          .map(move |t| tasks.by_shorthand[t])
          .collect();
        (*avg, route)
      })
      .collect();
    let pv_routes = self.pv_routes.iter()
      .map(|(pv, route)| {
        let route = route.iter()
          .map(move |t| tasks.by_shorthandpv[t])
          .collect();
        (*pv, route)
      })
      .collect();

    Solution {
      objective: self.objective,
      av_routes,
      pv_routes,
    }
  }
}


#[derive(Clone, Debug)]
pub struct SpSolution {
  pub av_routes: Vec<(Avg, Vec<(Task, Time)>)>,
  pub pv_routes: Vec<(Pv, Vec<(PvTask, Time)>)>,
}


impl SpSolution {
  pub fn print_objective_breakdown(&self, data: impl AsRef<Data>, obj: &ObjWeights) {
    let data = data.as_ref();

    let mut theta_costs = 0;
    let mut theta_y = 0;
    let mut y_travel_costs = 0;
    let mut x_travel_costs = 0;

    for (_, route) in &self.pv_routes {
      for (t, _) in route {
        x_travel_costs += data.travel_cost[&(t.start, t.end)];
      }
    }

    for (_, route) in &self.av_routes {
      for ((t1,_), (t2,_)) in route.iter().tuple_windows() {
        y_travel_costs += data.travel_cost[&(t1.end, t2.start)];
      }
      let (t, time) = route[route.len()-2];
      theta_costs += time;
      theta_y += data.travel_time[&(t.start, t.end)] + data.travel_time[&(t.end, Loc::Ad)];
    }

    println!("\nObjective terms:");
    println!("Component   Unweighted    Weighted");
    println!("    Theta     {:8}    {:8}", theta_costs, obj.av_finish_time * theta_costs);
    println!("     Y TT     {:8}    {:8}", theta_y, obj.av_finish_time * theta_y);
    println!("   X Cost     {:8}    {:8}", x_travel_costs, obj.tt * x_travel_costs);
    println!("   Y Cost     {:8}    {:8}", y_travel_costs, obj.tt * y_travel_costs);
  }

  pub fn from_sp(sp: &TimingSubproblem) -> Result<SpSolution> {
    let task_start_times: Map<_, _> = get_var_values(&sp.model, &sp.vars)?
      .map(|(task, time)| (task, time.round() as Time))
      .collect();

    let mut avr: Vec<_> = sp.mp_sol.av_routes.iter()
      .map(|(av, route)| {
        let mut sched: Vec<_> = route[..route.len() - 1].iter()
          .map(|task| (*task, *task_start_times.get(task).unwrap_or(&0)))
          .collect();

        let (second_last_task, t) = sched.last().unwrap().clone();
        let ddepot_task = route.last().unwrap().clone();
        debug_assert_eq!(ddepot_task.ty, TaskType::DDepot);
        sched.push((ddepot_task, t + second_last_task.tt + sp.lu.data.travel_time[&(second_last_task.end, ddepot_task.start)]));
        (*av, sched)
      })
      .collect();

    avr.sort_by_key(|(av, _)| *av);

    let mut pvr: Vec<_> = sp.mp_sol.pv_routes.iter()
      .map(|(pv, route)| {
        let sched = route.iter()
          .map(|t| (*t, task_start_times[&sp.lu.tasks.pvtask_to_task[t]])).collect();
        (*pv, sched)
      })
      .collect();

    pvr.sort_by_key(|(pv, _)| *pv);

    Ok(SpSolution { av_routes: avr, pv_routes: pvr })
  }

  pub fn pretty_print(&self, data: impl AsRef<Data>) {
    let data = data.as_ref();
    use prettytable::*;

    for (av, route) in &self.av_routes {
      let mut table = Table::new();
      let mut task_row = vec![cell!("Task")];
      let mut release_time = vec![cell!("Rel")];
      let mut st_row = vec![cell!("ST")];
      let mut tt_time = vec![cell!("TT")];

      println!("Active Vehicle Group {}", av);
      for ((task1, t1), (task2, _)) in route.iter().tuple_windows() {
        task_row.push(cell!(format!("{:?}", task1)));
        task_row.push(cell!(format!("(drive)")));
        st_row.push(cell!(format!("{:?}", t1)));
        st_row.push(cell!(format!("")));
        tt_time.push(cell!(format!("{:?}", task1.tt)));
        tt_time.push(cell!(data.travel_time[&(task1.end, task2.start)]));
        release_time.push(cell!(format!("{}", task1.t_release)));
        release_time.push(cell!(""));
      }

      if let Some((task, t)) = route.last() {
        task_row.push(cell!(format!("{:?}", task)));
        st_row.push(cell!(format!("{:?}", t)));
        tt_time.push(cell!(format!("{:?}", task.tt)));
        release_time.push(cell!(format!("{}", task.t_release)));
      }

      table.add_row(Row::new(task_row));
      table.add_row(Row::new(release_time));
      table.add_row(Row::new(st_row));
      table.add_row(Row::new(tt_time));
      table.printstd();
      // println!("{:?}", route)
    }

    for (pv, route) in &self.pv_routes {
      let mut table = Table::new();
      let mut task_row = vec![cell!("Task")];
      let mut st_row = vec![cell!("ST")];
      let mut tt_row = vec![cell!("TT")];
      // let mut et_row = vec![cell!("ET")];
      // let mut lt_row = vec![cell!("LT")];

      for (task, t) in route {
        task_row.push(cell!(format!("{:?}", task)));
        st_row.push(cell!(format!("{:?}", t)));
        tt_row.push(cell!(format!("{:?}", task.tt)));

        //
        // et_row.push(
        //   self.data.start_time.get(&task.start)
        //     .map(|t| cell!(format!("{}", t)))
        //     .unwrap_or_else(|| cell!(""))
        // )
      }

      table.add_row(Row::new(task_row));
      // table.add_row(Row::new(et_row));
      table.add_row(Row::new(st_row));
      table.add_row(Row::new(tt_row));

      println!("Passive Vehicle {}", pv);
      table.printstd();
    }
  }
}

mod debugging {
  use super::*;

  #[derive(Deserialize, Copy, Clone, Debug, Hash, Eq, PartialEq)]
  struct JsonTask {
    start: RawLoc,
    end: RawLoc,
    p: i32,
  }

  #[derive(Deserialize, Clone)]
  struct JsonSolution {
    index: usize,
    objective: Cost,
    av_routes: Map<String, Vec<Vec<JsonTask>>>,
    pv_routes: Map<String, Vec<JsonTask>>,
  }

  impl JsonTask {
    #[tracing::instrument(level = "trace", name="json_task_to_sh_task")]
    pub fn to_shorthand(&self, lss: &LocSetStarts) -> ShorthandPvTask {
      use ShorthandPvTask::*;

      let start = lss.decode(self.start);
      let end = lss.decode(self.end);
      let p =
        if self.p < 0 { None } else { Some(self.p as Pv) };

      let sht = match start {
        Loc::Ao => unimplemented!(),
        Loc::Ad => unimplemented!(),
        Loc::ReqP(r) => Request(p.unwrap(), r),
        Loc::Po(p) => {
          match end {
            Loc::Pd(p2) if p2 == p => Direct(p),
            Loc::ReqP(r) => Start(p, r),
            _ => unreachable!("unmatched end: start={:?}, end={:?}", start, end),
          }
        }
        Loc::ReqD(r1) => {
          match end {
            Loc::ReqP(r2) => Transfer(p.unwrap(), r1, r2),
            Loc::Pd(p) => End(p, r1),
            _ => unreachable!("unmatched end: start={:?}, end={:?}", start, end)
          }
        }
        _ => unreachable!("unmatched start: start={:?}, end={:?}", start, end)
      };
      trace!(sh_task=?sht);
      sht
    }
  }


  pub fn load_michael_soln(path: impl AsRef<Path>, tasks: impl AsRef<Tasks>, lss: &LocSetStarts) -> Result<Solution> {
    let tasks = tasks.as_ref();
    let s = std::fs::read_to_string(path)?;
    let soln: JsonSolution = serde_json::from_str(&s)?;

    let mut av_routes = Vec::new();
    for (av, routes) in &soln.av_routes {
      let av: Avg = av.parse().context("parsing AV")?;
      for route in routes {
        let mut r = Vec::with_capacity(route.len() + 2);
        r.push(tasks.odepot);
        r.extend(
          route.iter() .map(|t| tasks.by_shorthand[&t.to_shorthand(lss).into()])
        );
        r.push(tasks.ddepot);
        av_routes.push((av, r));
      }
    }

    let mut pv_routes = Vec::new();
    for (pv, route) in &soln.pv_routes {
      let pv: Pv = pv.parse().context("parsing PV")?;
      let route = route.iter()
        .map(|t| tasks.by_shorthandpv[&t.to_shorthand(&lss).into()])
        .collect();
      pv_routes.push((pv, route));
    }

    Ok(Solution { objective:  Some(soln.objective), av_routes, pv_routes })
  }
}

pub use debugging::load_michael_soln;
use std::ops::Deref;
