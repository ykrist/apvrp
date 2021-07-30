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

mod route {
  use super::*;

  /// A Passive vehicle route, beginning at a PV origin depot and ending at a PV destination depot
  #[derive(Debug, Clone, Eq, PartialEq)]
  pub struct PvRoute(Vec<PvTask>);

  impl PvRoute {
    pub fn new(path: Vec<PvTask>) -> Self {
      debug_assert!(
        (path.len() == 1 && path.first().unwrap().ty == TaskType::Direct) ||
          (path.first().unwrap().ty == TaskType::Start && path.last().unwrap().ty == TaskType::End)
      );

      Self(path)
    }
  }


  /// A PV cycle, first task is equal to the last task
  #[derive(Debug, Clone, Eq, PartialEq)]
  pub struct PvCycle(Vec<PvTask>);

  impl PvCycle {
    pub fn new(path: Vec<PvTask>) -> Self {
      debug_assert_eq!(path.first().unwrap(), path.last().unwrap());
      Self(path)
    }

    pub fn unique_tasks(&self) -> &[PvTask] {
      &self.0[..self.len() - 1]
    }
  }


  macro_rules! impl_common_pv {
    ($ty:path) => {
      impl Deref for $ty {
        type Target = [PvTask];

        fn deref(&self) -> &Self::Target {
          self.0.as_slice()
        }
      }

      impl $ty {
        pub fn pv(&self) -> Pv { self.0[0].p }

        pub fn into_inner(self) -> Vec<PvTask> { self.0 }

        pub fn iter_edges<'a>(&'a self) -> impl Iterator<Item=(&'a PvTask, &'a PvTask)> + 'a {
          self.0.iter().tuple_windows()
        }

        pub fn iter_edges_owned<'a>(&'a self) -> impl Iterator<Item=(PvTask, PvTask)> + 'a {
          self.0.iter().copied().tuple_windows()
        }
      }

    };
  }

  impl_common_pv!(PvRoute);
  impl_common_pv!(PvCycle);


  /// A AV vehicle route, beginning at the AV origin depot and ending the AV destination depot
  #[derive(Debug, Clone, Eq, PartialEq)]
  pub struct AvRoute {
    tasks: Vec<Task>,
    av: Av,
  }


  impl AvRoute {
    pub fn new(av: Av, tasks: Vec<Task>) -> Self {
      debug_assert_matches!(tasks.first().unwrap().ty, TaskType::ODepot);
      debug_assert_matches!(tasks.last().unwrap().ty, TaskType::DDepot);
      AvRoute { av, tasks }
    }

    pub fn without_depots(&self) -> &[Task] {
      &self[1..self.len() - 1]
    }

    pub fn theta_task(&self) -> &Task {
      &self[self.len() - 2]
    }
  }


  /// An AV cycle, first task is equal to the last task
  #[derive(Debug, Clone, Eq, PartialEq)]
  pub struct AvCycle {
    tasks: Vec<Task>,
    av: Av,
  }

  impl AvCycle {
    pub fn new(av: Av, tasks: Vec<Task>) -> Self {
      debug_assert_eq!(tasks.first().unwrap(), tasks.last().unwrap());
      AvCycle { av, tasks }
    }
  }

  macro_rules! impl_common_av {
    ($ty:path) => {
      impl Deref for $ty {
        type Target = [Task];

        fn deref(&self) -> &Self::Target {
          self.tasks.as_slice()
        }
      }

      impl $ty {
        pub fn av(&self) -> Av { self.av }

        pub fn into_inner(self) -> Vec<Task> { self.tasks }

        pub fn iter_edges<'a>(&'a self) -> impl Iterator<Item=(&'a Task, &'a Task)> + 'a {
          self.tasks.iter().tuple_windows()
        }

        pub fn iter_edges_owned<'a>(&'a self) -> impl Iterator<Item=(Task, Task)> + 'a {
          self.tasks.iter().copied().tuple_windows()
        }

        pub fn iter_y_vars<'a>(&'a self) -> impl Iterator<Item=(Av, Task, Task)> + 'a {
          self.iter_edges_owned().map(move |(t1, t2)| (self.av, t1, t2))
        }
      }

    };
  }

  impl_common_av!(AvCycle);
  impl_common_av!(AvRoute);
}

pub use route::*;

pub fn iter_solution_log(p: impl AsRef<Path>) -> Result<impl Iterator<Item=SerialisableSolution>> {
  let contents = std::io::Cursor::new(std::fs::read(&p).read_context(&p)?);
  let stream = serde_json::Deserializer::from_reader(contents)
    .into_iter()
    .map(|r| r.expect("failure parsing file"));
  Ok(stream)
}


/// Given a list of task pairs (from the Y variables), construct routes for an Active Vehicle class, plus any cycles which occur.
#[tracing::instrument(level = "debug", skip(task_pairs))]
pub fn construct_av_routes(task_pairs: impl IntoIterator<Item=(Av, Task, Task)>) -> (Vec<AvRoute>, Vec<AvCycle>) {
  use crate::graph::DecomposableDigraph;

  #[derive(Copy, Clone, Debug, Hash, Eq, PartialEq)]
  struct AvTask {
    task: Task,
    av: Av,
  }

  #[derive(Debug)]
  struct AvGraph {
    pub odepot_arcs: SmallVec<[(AvTask, AvTask); crate::constants::NUM_AV_UB]>,
    pub succ: Map<AvTask, AvTask>,
  }

  impl DecomposableDigraph<AvTask, (AvTask, AvTask), u8> for AvGraph {
    fn is_sink(&self, node: &AvTask) -> bool { node.task.ty == TaskType::DDepot }

    #[tracing::instrument(level = "trace", name = "find_start", fields(succ = ? & self.succ, od_arcs = ? & self.odepot_arcs), skip(self))]
    fn next_start(&self) -> Option<AvTask> {
      self.odepot_arcs.iter().next()
        .map(|(od, _)| *od)
        .or_else(|| {
          trace!("no start arcs left");
          self.succ.keys().next().copied()
        })
    }

    #[tracing::instrument(level = "trace", name = "find_succ", fields(succ = ? & self.succ, od_arcs = ? & self.odepot_arcs), skip(self))]
    fn next_outgoing_arc(&self, t: &AvTask) -> ((AvTask, AvTask), AvTask, u8) {
      let arc = if t.task.ty == TaskType::ODepot {
        *self.odepot_arcs.last().unwrap()
      } else {
        let next_task = self.succ[t];
        (*t, next_task)
      };
      trace!(next_task=?arc.1);
      (arc, arc.1, 1)
    }

    #[tracing::instrument(level = "trace", fields(succ = ? & self.succ, od_arcs = ? & self.odepot_arcs), skip(self))]
    fn subtract_arc(&mut self, arc: &(AvTask, AvTask), _: u8) {
      if arc.0.task.ty == TaskType::ODepot {
        trace!("remove depot arc");
        let s = self.odepot_arcs.pop();
        debug_assert_eq!(s.as_ref(), Some(arc));
      } else {
        trace!("remove non-depot arc");
        let removed = self.succ.remove(&arc.0).is_some();
        debug_assert!(removed)
      };
    }
  }

  let graph = {
    let mut odepot_arcs = SmallVec::new();
    let mut succ = Map::default();
    for (av, t1, t2) in task_pairs {
      trace!(av, ?t1, ?t2);
      let t1 = AvTask { task: t1, av };
      let t2 = AvTask { task: t2, av };

      if t1.task.ty == TaskType::ODepot {
        odepot_arcs.push((t1, t2));
      } else {
        succ.insert(t1, t2);
      }
    }

    AvGraph { odepot_arcs, succ }
  };

  let (paths, cycles) = graph.decompose_paths_cycles();

  #[inline]
  fn path_nodes(arcs: Vec<(AvTask, AvTask)>) -> (Av, Vec<Task>) {
    let first_arc = arcs[0];
    let AvTask { av, task } = first_arc.0;
    let path = std::iter::once(task)
      .chain(arcs.into_iter().map(|a| a.1.task))
      .collect();
    (av, path)
  }

  let paths = paths.into_iter()
    .map(|(arcs, _)| {
      let (av, path) = path_nodes(arcs);
      AvRoute::new(av, path)
    })
    .collect();

  let cycles = cycles.into_iter()
    .map(|(arcs, _)| {
      let (av, path) = path_nodes(arcs);
      AvCycle::new(av, path)
    })
    .collect();

  (paths, cycles)
}

/// Given a list of tasks, construct the route for a Passive vehicle, plus any cycles which occur.
#[tracing::instrument(level = "debug", skip(tasks_used))]
pub fn construct_pv_routes(tasks_used: impl IntoIterator<Item=PvTask>) -> (Vec<PvRoute>, Vec<PvCycle>) {
  struct PvGraph {
    tasks_by_start: Map<Loc, PvTask>,
  }

  impl DecomposableDigraph<Loc, PvTask, u8> for PvGraph {
    fn is_sink(&self, node: &Loc) -> bool { matches!(node, Loc::Pd(_)) }

    fn next_start(&self) -> Option<Loc> {
      self.tasks_by_start.keys()
        .find(|l| matches!(l, Loc::Po(_)))
        .copied()
        .or_else(|| self.tasks_by_start.keys().next().copied())
    }

    fn next_outgoing_arc(&self, node: &Loc) -> (PvTask, Loc, u8) {
      let task = self.tasks_by_start[node];
      (task, task.end, 1)
    }

    fn subtract_arc(&mut self, arc: &PvTask, _: u8) {
      let removed = self.tasks_by_start.remove(&arc.start);
      debug_assert!(removed.is_some())
    }
  }

  let tasks_by_start = tasks_used.into_iter().map(|t| (t.start, t)).collect();
  trace!(?tasks_by_start);
  let graph = PvGraph { tasks_by_start: tasks_by_start };

  let (pv_paths, pv_cycles) = graph.decompose_paths_cycles();

  let pv_paths = pv_paths.into_iter()
    .map(|(path, _)| PvRoute::new(path))
    .collect();


  let pv_cycles = pv_cycles.into_iter()
    .map(|(mut cycle, _)| {
      debug!(?cycle);
      cycle.push(cycle[0]);
      PvCycle::new(cycle)
    })
    .collect();

  (pv_paths, pv_cycles)
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

/// Return an iterator of variable values, discarding any which have 0 value
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
  pub av_routes: Vec<AvRoute>,
  pub av_cycles: Vec<AvCycle>,
  pub pv_routes: Vec<PvRoute>,
  pub pv_cycles: Vec<PvCycle>,
}

impl Solution {
  pub fn to_serialisable(&self) -> SerialisableSolution {
    SerialisableSolution {
      objective: self.objective,
      av_routes: self.av_routes.iter().map(|r| (r.av(), r.shorthand())).collect(),
      av_cycles: self.av_cycles.iter().map(|r| (r.av(), r.shorthand())).collect(),
      pv_routes: self.pv_routes.iter().map(|r| (r.pv(), r.shorthand())).collect(),
      pv_cycles: self.pv_cycles.iter().map(|r| (r.pv(), r.shorthand())).collect(),
    }
  }

  pub fn from_mp(mp: &TaskModelMaster) -> Result<Self> {
    let task_pairs = get_var_values(&mp.model, &mp.vars.y)?.map(|(key, _)| key);
    let (av_routes, av_cycles) = construct_av_routes(task_pairs);

    let pv_tasks = get_var_values(&mp.model, &mp.vars.x)?.map(|(key, _)| key);
    let (pv_routes, pv_cycles) = construct_pv_routes(pv_tasks);

    Ok(Solution {
      objective: Some(mp.obj_val()?),
      av_routes,
      av_cycles,
      pv_routes,
      pv_cycles,
    })
  }

  pub fn solve_for_times(&self, lu: &Lookups) -> Result<SpSolution> {
    if !(self.pv_cycles.is_empty() && self.av_cycles.is_empty()) {
      anyhow::bail!("Cycles in solution")
    }
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
  pub av_cycles: Vec<(Avg, Vec<ShorthandTask>)>,
  pub pv_routes: Vec<(Pv, Vec<ShorthandPvTask>)>,
  pub pv_cycles: Vec<(Pv, Vec<ShorthandPvTask>)>,
}

impl SerialisableSolution {
  pub fn to_solution(&self, tasks: impl AsRef<Tasks>) -> Solution {
    let tasks = tasks.as_ref();

    let lookup_shorthand = |path: &[ShorthandTask]| -> Vec<Task> {
      path.iter().map(move |t| tasks.by_shorthand[t]).collect()
    };

    let av_routes = self.av_routes.iter()
      .map(|(av, route)| AvRoute::new(*av, lookup_shorthand(route)))
      .collect();

    let av_cycles = self.av_cycles.iter()
      .map(|(av, cycle)| AvCycle::new(*av, lookup_shorthand(cycle)))
      .collect();

    let lookup_shorthand = |path: &[ShorthandPvTask]| -> Vec<PvTask> {
      path.iter().map(move |t| tasks.by_shorthandpv[t]).collect()
    };

    let pv_routes = self.pv_routes.iter()
      .map(|(_, route)| PvRoute::new( lookup_shorthand(route)))
      .collect();

    let pv_cycles = self.pv_cycles.iter()
      .map(|(_, cycle)| PvCycle::new( lookup_shorthand(cycle)))
      .collect();

    Solution {
      objective: self.objective,
      av_routes,
      av_cycles,
      pv_routes,
      pv_cycles,
    }
  }
}


#[derive(Clone, Debug)]
pub struct SpSolution {
  pub av_routes: Vec<(AvRoute, Vec<Time>)>,
  pub pv_routes: Vec<(PvRoute, Vec<Time>)>,
}

impl SpSolution {
  pub fn print_objective_breakdown(&self, data: impl AsRef<Data>, obj: &ObjWeights) {
    let data = data.as_ref();

    let mut theta_costs = 0;
    let mut theta_y = 0;
    let mut y_travel_costs = 0;
    let mut x_travel_costs = 0;

    for (route, _) in &self.pv_routes {
      for t in route.iter() {
        x_travel_costs += data.travel_cost[&(t.start, t.end)];
      }
    }

    for (route, times) in &self.av_routes {
      for (t1, t2) in route.iter_edges() {
        y_travel_costs += data.travel_cost[&(t1.end, t2.start)];
      }
      let t = route.theta_task();
      let time = times[times.len() - 2];
      theta_costs += time;
      let to_depot = data.travel_time[&(t.start, t.end)] + data.travel_time[&(t.end, Loc::Ad)];
      theta_y += to_depot;
      debug_assert_eq!(time + to_depot, *times.last().unwrap())
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

    let av_routes: Vec<_> = sp.mp_sol.av_routes.iter().cloned()
      .map(|route: AvRoute| {
        let mut sched: Vec<_> = std::iter::once(0) // ODepot
          .chain(route.without_depots().iter().map(|t| task_start_times[t]))
          .collect();
        // DDepot
        sched.push(*sched.last().unwrap() + sp.lu.data.travel_time_to_ddepot(route.theta_task()));
        (route, sched)
      })
      .collect();

    let pv_routes: Vec<_> = sp.mp_sol.pv_routes.iter().cloned()
      .map(|route| {
        let sched = route.iter()
          .map(|t| task_start_times[&sp.lu.tasks.pvtask_to_task[t]]).collect();
        (route, sched)
      })
      .collect();


    Ok(SpSolution { av_routes, pv_routes })
  }

  pub fn pretty_print(&self, data: impl AsRef<Data>) {
    let data = data.as_ref();
    use prettytable::*;

    for (route, sched) in &self.av_routes {
      let mut table = Table::new();
      let mut task_row = vec![cell!("Task")];
      let mut release_time = vec![cell!("Rel")];
      let mut st_row = vec![cell!("ST")];
      let mut tt_time = vec![cell!("TT")];

      println!("Active Vehicle Group {}", route.av());
      for ((task1, task2), t1) in route.iter().tuple_windows().zip(sched) {
        task_row.push(cell!(format!("{:?}", task1)));
        task_row.push(cell!(format!("(drive)")));
        st_row.push(cell!(format!("{:?}", t1)));
        st_row.push(cell!(format!("")));
        tt_time.push(cell!(format!("{:?}", task1.tt)));
        tt_time.push(cell!(data.travel_time[&(task1.end, task2.start)]));
        release_time.push(cell!(format!("{}", task1.t_release)));
        release_time.push(cell!(""));
      }

      if let Some(task) = route.last() {
        task_row.push(cell!(format!("{:?}", task)));
        st_row.push(cell!(format!("{:?}", sched.last().unwrap())));
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

    for (route, sched) in &self.pv_routes {
      let mut table = Table::new();
      let mut task_row = vec![cell!("Task")];
      let mut st_row = vec![cell!("ST")];
      let mut tt_row = vec![cell!("TT")];

      for (task, t) in route.iter().zip(sched) {
        task_row.push(cell!(format!("{:?}", task)));
        st_row.push(cell!(format!("{:?}", t)));
        tt_row.push(cell!(format!("{:?}", task.tt)));
      }

      table.add_row(Row::new(task_row));
      table.add_row(Row::new(st_row));
      table.add_row(Row::new(tt_row));

      println!("Passive Vehicle {}", route.pv());
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
    #[tracing::instrument(level = "trace", name = "json_task_to_sh_task")]
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
          route.iter().map(|t| tasks.by_shorthand[&t.to_shorthand(lss).into()])
        );
        r.push(tasks.ddepot);
        av_routes.push(AvRoute::new(av, r));
      }
    }

    let mut pv_routes = Vec::new();
    for (_, route) in &soln.pv_routes {
      let route = route.iter()
        .map(|t| tasks.by_shorthandpv[&t.to_shorthand(&lss).into()])
        .collect();
      pv_routes.push(PvRoute::new(route));
    }

    Ok(Solution {
      objective: Some(soln.objective),
      av_routes,
      pv_routes,
      av_cycles: vec![],
      pv_cycles: vec![],
    })
  }
}

pub use debugging::load_michael_soln;
use std::ops::Deref;
use smallvec::SmallVec;
use crate::Shorthand;
