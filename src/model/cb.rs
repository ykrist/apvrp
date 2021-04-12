use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use grb::callback::{Callback, Where, CbResult, MIPSolCtx};
use fnv::FnvHashSet;
use super::sp::{BendersCut, TimingSubproblem};
use crate::graph::DecomposableDigraph;
use tracing::{info, info_span, debug, trace, error_span};
use std::fmt;
use grb::constr::IneqExpr;
use variant_count::VariantCount;

#[derive(Debug, Clone)]
pub enum CbError {
  Status(grb::Status),
  Other(String),
}

impl fmt::Display for CbError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    f.write_str("Callback error: ")?;
    match self {
      CbError::Status(s) => f.write_fmt(format_args!("unexpected status: {:?}", s))?,
      CbError::Other(s) => f.write_str(s)?,
    }
    Ok(())
  }
}

impl std::error::Error for CbError {}

pub enum Component {
  Path(Vec<Task>),
  Cycle(Vec<Task>),
}

#[inline]
fn discard_edge_weights<T,W>(pairs: Vec<(T, W)>) -> Vec<T> {
  pairs.into_iter().map(|(x, _)| x).collect()
}

pub(super) type AvPath = Vec<(Task, Task)>;
pub(super) type PvPath = Vec<Task>;

/// Given a list of task pairs (from the Y variables), construct routes for an Active Vehicle class, plus any cycles which occur.
#[tracing::instrument(skip(tasks, task_pairs))]
pub fn construct_av_routes(tasks: &Tasks, task_pairs: &[(Task, Task)]) -> (Vec<AvPath>, Vec<AvPath>) {
  trace!(?task_pairs);
  use crate::graph::DecomposableDigraph;

  #[derive(Debug)]
  struct AvGraph {
    pub odepot_arcs: FnvHashSet<(Task, Task)>,
    pub succ: Map<Task, Task>,
  }

  impl DecomposableDigraph<Task, (Task, Task), u8> for AvGraph {
    fn is_sink(&self, node: &Task) -> bool { node.ty == TaskType::DDepot }

    #[tracing::instrument(level="error", name="find_start", fields(succ=?&self.succ, od_arcs=?&self.odepot_arcs), skip(self))]
    fn next_start(&self) -> Option<Task> {
      self.odepot_arcs.iter().next()
        .map(|(od, _)| *od)
        .or_else(|| { trace!("no start arcs left"); self.succ.keys().next().copied() })
    }

    #[tracing::instrument(level="error", name="find_succ", fields(succ=?&self.succ, od_arcs=?&self.odepot_arcs), skip(self))]
    fn next_outgoing_arc(&self, task: &Task) -> ((Task, Task), Task, u8) {
      let arc = if task.ty == TaskType::ODepot {
        *self.odepot_arcs.iter().next().unwrap()
      } else {
        (*task, self.succ[task])
      };
      trace!(next_task=?arc.1);
      (arc, arc.1, 1)
    }

    #[tracing::instrument(level="error", fields(succ=?&self.succ, od_arcs=?&self.odepot_arcs), skip(self))]
    fn subtract_arc(&mut self, arc: &(Task, Task), _: u8) {
      let value_removed =
        if arc.0.ty == TaskType::ODepot {
          trace!("remove depot arc");
          self.odepot_arcs.remove(arc)
        }
        else {
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
  (discard_edge_weights(paths), discard_edge_weights(cycles))
}

/// Given a list of tasks, construct the route for a Passive vehicle, plus any cycles which occur.
#[tracing::instrument(skip(data, tasks_used))]
pub fn construct_pv_route(data: &Data, tasks_used: &[Task]) -> (PvPath, Vec<PvPath>) {
  trace!(?tasks_used);
  debug_assert!(!tasks_used.is_empty());
  let first_task = *tasks_used.first().unwrap();

  if tasks_used.len() == 1 {
    trace!(task=?first_task, "single-task route");
    debug_assert_eq!(first_task.ty, TaskType::Direct);
    return (vec![first_task], vec![])
  }

  struct PvGraph {
    pub pv_origin: Loc,
    pub pv_dest: Loc,
    pub task_by_start: Map<Loc, Task>,
  }

  impl DecomposableDigraph<Loc, Task, u8> for PvGraph {
    fn is_sink(&self, node: &Loc) -> bool { node == &self.pv_dest }

    fn next_start(&self) -> Option<Loc> {
      if self.task_by_start.contains_key(&self.pv_origin) { Some(self.pv_origin) }
      else { self.task_by_start.keys().next().copied() }
    }

    fn next_outgoing_arc(&self, node: &Loc) -> (Task, Loc, u8) {
      let task = self.task_by_start[node];
      (task, task.end, 1)
    }

    fn subtract_arc(&mut self, arc: &Task, _: u8) {
      let removed = self.task_by_start.remove(&arc.start);
      debug_assert!(removed.is_some())
    }
  }

  let pv_origin = first_task.p.unwrap();
  let graph = PvGraph {
    pv_origin,
    pv_dest: pv_origin + data.n_passive,
    task_by_start: tasks_used.iter().map(|&t| (t.start, t)).collect()
  };


  let (pv_paths, pv_cycles) = graph.decompose_paths_cycles();

  debug_assert_eq!(pv_paths.len(), 1);
  let pv_path = discard_edge_weights(pv_paths).pop().unwrap();
  (pv_path, discard_edge_weights(pv_cycles))
}

#[derive(VariantCount, Copy, Clone, Debug)]
#[repr(usize)]
pub enum CutType {
  LpOpt = 0,
  LpFeas,
  AvCycle,
  PvCycle,
}

pub struct CutCounts {
  lp_opt: usize,
  lp_feas: usize,
  av_cycle: usize,
  pv_cycle: usize,
}

#[derive(Clone)]
pub struct CbStats {
  n_cuts: Vec<usize>,
}

impl std::default::Default for CbStats {
  fn default() -> Self {
    CbStats { n_cuts: vec![0; CutType::VARIANT_COUNT] }
  }
}

impl CbStats {
  #[inline]
  pub fn inc_cut_count(&mut self, cut_ty: CutType) -> usize {
    #[cfg(debug_assertions)]
      let val = self.n_cuts.get_mut(cut_ty as usize).expect("vec should be large enough: will cause UB in release builds!");

    // Safety: We know the discriminant of `CutType` is always a valid index because the only
    // way to create a CbStats struct is to call `default()`, which allocates a `Vec` large enough.
    #[cfg(not(debug_assertions))]
      let val = unsafe { self.n_cuts.get_unchecked_mut(cut_ty as usize) };

    let val = unsafe { self.n_cuts.get_unchecked_mut(cut_ty as usize) };
    let old_val = *val;
    *val += 1;
    old_val
  }

  pub fn get_cut_counts(&self) -> Vec<(CutType, usize)> {
    self.n_cuts.iter()
      .enumerate()
      .map(|(cut_ty, &cnt)| {
        // Safety: We know the discriminant of `CutType` is always a valid index because the only
        // way to create a CbStats struct is to call `default()`, which allocates a `Vec` of length
        // equal to the number of variants.
        let cut_ty: CutType = unsafe { std::mem::transmute(cut_ty) };
        (cut_ty, cnt)
      })
      .collect()
  }
}

pub struct Cb<'a> {
  data: &'a Data,
  sets: &'a Sets,
  tasks: &'a Tasks,
  mp_vars: super::mp::MpVars, // needs to be owned to avoid mutability issues with the Model object.
  stats: CbStats,
  cut_cache: Vec<(String, IneqExpr)>,
}

impl<'a> Cb<'a> {
  pub fn new(data: &'a Data, sets: &'a Sets, tasks: &'a Tasks, mp_vars: super::mp::MpVars) -> Self {
    let stats = CbStats::default();
    Cb {
      data,
      sets,
      tasks,
      mp_vars,
      stats,
      cut_cache: Vec::new(),
    }
  }

  pub fn flush_cut_cache(&mut self, mp: &mut Model) -> Result<()> {
    for (name, cut) in self.cut_cache.drain(0..) {
      mp.add_constr(&name, cut)?;
    }
    Ok(())
  }

  #[tracing::instrument(level="trace", skip(self, ctx))]
  fn get_tasks_by_pv(&self, ctx: &MIPSolCtx) -> anyhow::Result<Map<Pv, Vec<Task>>> {
    let mut pv_tasks = map_with_capacity(self.data.n_passive);

    for (t, val) in self.mp_vars.x.keys()
      .zip(ctx.get_solution(self.mp_vars.x.values())?) {
      if val > 0.9 {
        trace!(t=?&t, ?val, "non-zero X var");
        if !t.is_depot() {
          pv_tasks.entry(t.p.unwrap())
            .or_insert_with(Vec::new)
            .push(*t);
        }
      }
    }
    Ok(pv_tasks)
  }

  fn get_task_pairs_by_av(&self, ctx: &MIPSolCtx) -> anyhow::Result<Map<Avg, Vec<(Task, Task)>>> {
    let mut task_pairs = map_with_capacity(self.data.av_groups.len());
    for ((av, t1, t2), val) in self.mp_vars.y.keys()
      .zip(ctx.get_solution(self.mp_vars.y.values())?)
    {
      if val > 0.9 {
        trace!(av, ?t1, ?t2, ?val);
        task_pairs.entry(*av).or_insert_with(Vec::new).push((*t1, *t2));
      }
    }
    Ok(task_pairs)
  }

  fn av_cycle_cut(&self, cycle: &AvPath) -> IneqExpr {
    let ysum = cycle.iter()
      .flat_map(|&(t1, t2)| {
        self.sets.avs()
          .filter_map( move |a| self.mp_vars.y.get(&(a, t1, t2)))
      })
      .grb_sum();
    c!(ysum <= cycle.len() - 1)
  }

  fn pv_cycle_cut(&self, cycle: &PvPath) -> IneqExpr {
    let xsum = cycle.iter()
      .flat_map(|t|
        self.tasks.by_locs[&(t.start, t.end)].iter()
          .map(|t| self.mp_vars.x[t]))
      .grb_sum();
    c!(xsum <= cycle.len() - 1)
  }
}


impl<'a> Callback for Cb<'a> {
  fn callback(&mut self, w: Where) -> CbResult {
    let _span = info_span!("cb").entered();

    match w {
      Where::MIPSol(ctx) => {
        info!("integer MP solution found");
        let old_cut_cache_len = self.cut_cache.len();
        // if old_cut_cache_len > 15 { ctx.terminate(); return Ok(()); } // TODO REMOVE

        let task_pairs = self.get_task_pairs_by_av(&ctx)?;
        let mut av_routes = map_with_capacity(task_pairs.len());
        for (&av, av_task_pairs) in &task_pairs {
          let (av_paths, av_cycles) = construct_av_routes(&self.tasks, av_task_pairs);
          if av_cycles.len() > 0 {
            info!(num_cycles=av_cycles.len(), "AV cycles found");
            trace!(?av_cycles);
            for cycle in av_cycles {
              self.stats.inc_cut_count(CutType::AvCycle);
              self.cut_cache.push(("av_cycle".to_string(), self.av_cycle_cut(&cycle)));
            }
          }

          av_routes.insert(av, av_paths);

          // TODO heuristic AV-path time-feasibility cuts
          // TODO heuristic AV-path cycle cuts
        }

        let pv_tasks = self.get_tasks_by_pv(&ctx)?;
        let mut pv_routes = map_with_capacity(pv_tasks.len());
        for (&pv, tasks) in &pv_tasks {
          let (pv_path, pv_cycles) = construct_pv_route(&self.data, tasks);
          pv_routes.insert(pv, pv_path);

          for cycle in pv_cycles {
            self.cut_cache.push(("pv_cycle".to_string(), self.pv_cycle_cut(&cycle)));
          }
          // TODO heuristic PV feasibility cuts
        }


        if self.cut_cache.len() == old_cut_cache_len {
          let _span = error_span!("lp_sp").entered();
          info!("no heuristic cuts found, building LP subproblem");
          let sp = TimingSubproblem::build(self.data, self.tasks, &av_routes, &pv_routes)?;
          info!("solving LP subproblem");
          let cuts = sp.solve(self.tasks)?;
          info!(ncuts=cuts.len(), "LP cuts generated");

          for cut in cuts {
            let name = match &cut {
              BendersCut::Optimality(_) => {
                self.stats.inc_cut_count(CutType::LpOpt);
                "sp_opt"
              }
              BendersCut::Feasibility(_) => {
                self.stats.inc_cut_count(CutType::LpFeas);
                "sp_feas"
              }
            };
            let cut = cut.into_ineq(self.sets, self.tasks, &self.mp_vars);
            self.cut_cache.push((name.to_string(), cut));
          }
        } else {
          info!(ncuts=self.cut_cache.len()-old_cut_cache_len, "heuristic cuts added");
        }

        for (_, cut) in &self.cut_cache[old_cut_cache_len..] {
          ctx.add_lazy(cut.clone())?;
        }


      }
      _ => {}
    }
    Ok(())
  }
}
