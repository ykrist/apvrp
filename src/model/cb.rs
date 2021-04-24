use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use grb::callback::{Callback, Where, CbResult, MIPSolCtx};
use fnv::FnvHashSet;
use super::sp::{BendersCut, TimingSubproblem};
use crate::graph::DecomposableDigraph;
use tracing::{info, info_span, debug, trace, error_span, warn};
use std::fmt;
use grb::constr::IneqExpr;
use variant_count::VariantCount;
use anyhow::Context;
use std::hash::Hash;
use std::env::var;
use std::collections::{HashSet, HashMap};

#[derive(Debug, Clone)]
pub enum CbError {
  Assertion,
  Status(grb::Status),
  Other(String),
}

impl fmt::Display for CbError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    f.write_str("Callback error: ")?;
    match self {
      CbError::Status(s) => f.write_fmt(format_args!("unexpected status: {:?}", s))?,
      CbError::Assertion => f.write_str("an assertion failed")?,
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

/// A (possibly cyclic) Active Vehicle path.  If the path is a cycle, the first and last Tasks are the same.
pub(crate) type AvPath = Vec<Task>;
/// A (possibly cyclic) Passive Vehicle path.  If the path is a cycle, the first and last Tasks are the same.
pub(crate) type PvPath = Vec<Task>;

/// Given a list of task pairs (from the Y variables), construct routes for an Active Vehicle class, plus any cycles which occur.
#[tracing::instrument(skip(task_pairs))]
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
  (to_path_nodes(&paths), to_path_nodes(&cycles))
}

/// Given a list of tasks, construct the route for a Passive vehicle, plus any cycles which occur.
#[tracing::instrument(skip(tasks_used))]
pub fn construct_pv_route(tasks_used: &[Task]) -> (PvPath, Vec<PvPath>) {
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

  let pv_origin = Loc::Po(first_task.p.unwrap());
  let graph = PvGraph {
    pv_origin,
    pv_dest: pv_origin.dest(),
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
  AvChain,
  PvChain,
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
  n_cuts_total: usize,
}

impl std::default::Default for CbStats {
  fn default() -> Self {
    CbStats { n_cuts: vec![0; CutType::VARIANT_COUNT], n_cuts_total: 0 }
  }
}

impl CbStats {
  #[inline]
  pub fn inc_cut_count(&mut self, cut_ty: CutType, inc: usize) -> usize {
    #[cfg(debug_assertions)]
      let val = self.n_cuts.get_mut(cut_ty as usize)
      .expect("vec should be large enough: will cause UB in release builds!");

    // Safety: We know the discriminant of `CutType` is always a valid index because the only
    // way to create a CbStats struct is to call `default()`, which allocates a `Vec` large enough.
    #[cfg(not(debug_assertions))]
      let val = unsafe { self.n_cuts.get_unchecked_mut(cut_ty as usize) };

    let old_val = *val;
    *val += inc;
    self.n_cuts_total += inc;
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

pub trait QueryVarValues {
  fn get(&self, vars: impl Iterator<Item=Var>) -> Result<Vec<f64>>;
}

impl<'a> QueryVarValues for MIPSolCtx<'a> {
  #[inline]
  fn get(&self, vars: impl Iterator<Item=Var>) -> Result<Vec<f64>> {
    Ok(self.get_solution(vars)?)
  }
}

impl <'a> QueryVarValues for Model {
  #[inline]
  fn get(&self, vars: impl Iterator<Item=Var>) -> Result<Vec<f64>> {
    Ok(self.get_obj_attr_batch(attr::X, vars)?)
  }
}

#[inline]
pub fn get_var_values<'a, M: QueryVarValues, K: Hash + Eq + Copy>(ctx: &M, var_dict: &'a Map<K, Var>) -> Result<impl Iterator<Item=(K, f64)> + 'a> {
  let vals = ctx.get(var_dict.values().copied())?;
  Ok(var_dict.keys().copied().zip(vals).filter(|(_, v)| v.abs() > 1e-8))
}

pub struct Cb<'a> {
  data: &'a Data,
  sets: &'a Sets,
  tasks: &'a Tasks,
  var_names: Map<Var, String>,
  mp_vars: super::mp::MpVars, // needs to be owned to avoid mutability issues with the Model object.
  pub stats: CbStats,
  cut_cache: Vec<(String, IneqExpr)>,
  av_cycles: HashMap<Vec<Task>, usize>,
  sp_env: Env,
}

impl<'a> Cb<'a> {
  pub fn new(data: &'a Data, sets: &'a Sets, tasks: &'a Tasks, mp: &super::mp::TaskModelMaster) -> Result<Self> {
    let stats = CbStats::default();
    let var_names: Map<_, _> = {
      let vars = mp.model.get_vars()?;
      let names = mp.model.get_obj_attr_batch(attr::VarName, vars.iter().copied())?;
      vars.iter().copied()
        .zip(names)
        .collect()
    };

    let sp_env = {
      let ctx_msg = "create SP environment";
      let mut e = Env::empty().context(ctx_msg)?;
      e.set(param::OutputFlag, 0)
        .and_then(|e| e.set(param::Threads, 1))
        .context(ctx_msg)?;
      e.start()?
    };

    Ok(Cb {
      data,
      sets,
      tasks,
      mp_vars: mp.vars.clone(),
      stats,
      cut_cache: Vec::new(),
      sp_env,
      var_names,
      av_cycles: Default::default(),
    })
  }

  pub fn flush_cut_cache(&mut self, mp: &mut Model) -> Result<()> {
    for (name, cut) in self.cut_cache.drain(0..) {
      mp.add_constr(&name, cut)?;
    }
    Ok(())
  }

  #[tracing::instrument(level="trace", skip(self, ctx))]
  fn get_tasks_by_pv(&self, ctx: &MIPSolCtx) -> anyhow::Result<Map<Pv, Vec<Task>>> {
    let mut pv_tasks = map_with_capacity(self.data.n_passive as usize);

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

  #[tracing::instrument(level="trace", skip(self))]
  fn av_cycle_cut(&mut self, cycle: &AvPath) -> IneqExpr {
    // trace!("{:#?}", cycle);
    #[cfg(debug_assertions)] {
      let mut sorted_cycle = cycle.clone();
      sorted_cycle.sort_by_key(|t| t.id());
      let count = self.av_cycles.entry(sorted_cycle).or_insert(0);
      if *count > 0 { warn!(?count, "cycle has been encountered before")}
      *count += 1;
    }
    let _self = &*self;

    let n = cycle.len();
    let ysum = cycle[..n-1].iter()
      .cartesian_product(cycle[..n-1].iter())
      .flat_map(|(&t1, &t2)| {
        _self.sets.avs()
          .filter_map( move |a| _self.mp_vars.y.get(&(a, t1, t2)))
      })
      .grb_sum();

    let c = c!(ysum <= n - 2); // cycle is a list of n+1 nodes (start = end), which form n arcs
    trace!(cut=?c.with_names(&self.var_names));
    // trace!(?cycle, cut=?&c);
    c
  }

  #[tracing::instrument(level="trace", skip(self))]
  fn av_chain_infork_cut(&mut self, chain: &[Task]) -> IneqExpr {
    todo!()
  }

  #[tracing::instrument(level="trace", skip(self))]
  fn av_chain_outfork_cut(&mut self, chain: &[Task]) -> IneqExpr {
    todo!()
  }

  #[tracing::instrument(level="trace", skip(self))]
  fn av_chain_tournament_cut(&self, chain: &[Task]) -> IneqExpr {
    todo!()
  }

  fn shortest_illegal_av_chain<'b>(&self, chain: &'b [Task]) -> &'b [Task] {
    for l in 3..=chain.len() {
      for start_pos in 0..(chain.len()-l) {
        let c = &chain[start_pos..(start_pos + l)];
        if !schedule::check_av_route(self.data, c) {
          return c;
        }
      }
    }
    unreachable!("chain must be illegal")
  }



  /// Generate an inequality to cut off `chain`, assuming `chain` violates timing constraints
  #[tracing::instrument(level="trace", skip(self))]
  fn av_chain_fork_cuts(&mut self, chain: &[Task]) -> (IneqExpr, IneqExpr) {

    debug_assert!(!schedule::check_av_route(self.data, chain));

    let mut end_idx = 2;
    while schedule::check_av_route(self.data, &chain[..end_idx]) {
      end_idx += 1;
    }

    let mut start_idx = end_idx-2;
    while schedule::check_av_route(self.data, &chain[start_idx..end_idx]) {
      start_idx -= 1;
    }


    let chain = &chain[start_idx..end_idx];
    let l = chain.len() - 1;

    debug_assert!(!schedule::check_av_route(self.data, chain));
    debug_assert!(chain.len() > 1);

    let y_sum_middle_bit = chain[1..l].iter()
      .tuple_windows()
      .flat_map(|(&t1, &t2)| {
        let y = &self.mp_vars.y;
        self.sets.avs().filter_map(move |a| y.get(&(a, t1 ,t2)).copied())
      })
      .grb_sum();

    let mut av_route = chain.to_owned();

    let after_cut = {
      let mut ysum = y_sum_middle_bit.clone() +
        self.sets.avs()
          .filter_map(|a| self.mp_vars.y.get(&(a, chain[0], chain[1])))
          .grb_sum();

      let second_last_task = chain[l-1];
      for &last_task in &self.tasks.succ[&second_last_task] {
        *av_route.last_mut().unwrap() = last_task;
        if !schedule::check_av_route(self.data, &av_route) {
          for av in self.sets.avs() {
            if let Some(&y) = self.mp_vars.y.get(&(av, second_last_task, last_task)) {
              ysum = ysum + y;
            }
          }
        }
      }
      *av_route.last_mut().unwrap() = chain[l];
      c!(ysum <= l-1)
    };

    trace!(short_chain=?chain, after_cut=?after_cut.with_names(&self.var_names));
    let before_cut = {
      let mut ysum = y_sum_middle_bit.clone() +
        self.sets.avs()
          .filter_map(|a| self.mp_vars.y.get(&(a, chain[l-1], chain[l])))
          .grb_sum();

      let second_task = chain[1];

      for &first_task in &self.tasks.pred[&second_task] {
        *av_route.first_mut().unwrap() = first_task;
        if !schedule::check_av_route(self.data, &av_route) {
          for av in self.sets.avs() {
            if let Some(&y) = self.mp_vars.y.get(&(av, first_task, second_task)) {
              ysum = ysum + y;
            }
          }
        }
      }
      c!(ysum <= l-1)
    };

    trace!(short_chain=?chain, before_cut=?before_cut.with_names(&self.var_names));
    (before_cut, after_cut)
  }


  fn pv_cycle_cut(&self, cycle: &PvPath) -> IneqExpr {
    let xsum = cycle.iter()
      .flat_map(|t|
        self.tasks.by_locs[&(t.start, t.end)].iter()
          .map(|t| self.mp_vars.x[t]))
      .grb_sum();
    c!(xsum <= cycle.len() - 2) // cycle is a list of n+1 nodes (start = end), which form n arcs
  }

  #[allow(unused_variables)]
  fn pv_chain_cuts(&self, chain: &PvPath) -> (IneqExpr, IneqExpr) {
    todo!()
  }
}


impl<'a> Callback for Cb<'a> {
  fn callback(&mut self, w: Where) -> CbResult {
    let _span = info_span!("cb").entered();

    match w {
      Where::MIPSol(ctx) => {
        debug!("integer MP solution found");
        let old_cut_cache_len = self.cut_cache.len();

        let task_pairs = self.get_task_pairs_by_av(&ctx)?;
        let mut av_routes = map_with_capacity(task_pairs.len());
        for (&av, av_task_pairs) in &task_pairs {
          let (av_paths, av_cycles) = construct_av_routes(av_task_pairs);
          if av_cycles.len() > 0 {
            info!(num_cycles=av_cycles.len(), "AV cycles found");
            trace!(?av_cycles);
            for cycle in av_cycles {
              if !schedule::check_av_route(self.data, &cycle[..cycle.len()-1]) {
                self.stats.inc_cut_count(CutType::AvChain, 2);
                let (before_cut, after_cut) = self.av_chain_fork_cuts(&cycle[..cycle.len()-1]);
                self.cut_cache.push(("av_chain_before".to_string(), before_cut));
                self.cut_cache.push(("av_chain_after".to_string(), after_cut));
              } else {
                self.stats.inc_cut_count(CutType::AvCycle, 1);
                let cut = self.av_cycle_cut(&cycle);
                self.cut_cache.push(("av_cycle".to_string(), cut));
              }
            }
          }

          for av_path in &av_paths {
            if !schedule::check_av_route(self.data, av_path) {
              self.stats.inc_cut_count(CutType::AvChain, 2);
              let (before_cut, after_cut) = self.av_chain_fork_cuts(av_path);
              self.cut_cache.push(("av_chain_before".to_string(), before_cut));
              self.cut_cache.push(("av_chain_after".to_string(), after_cut));
            }
          }

          av_routes.insert(av, av_paths);
        }

        let pv_tasks = self.get_tasks_by_pv(&ctx)?;
        let mut pv_routes = map_with_capacity(pv_tasks.len());
        for (&pv, tasks) in &pv_tasks {
          let (pv_path, pv_cycles) = construct_pv_route(tasks);

          for cycle in pv_cycles {
            let cut =  self.pv_cycle_cut(&cycle);
            self.stats.inc_cut_count(CutType::PvCycle, 1);
            self.cut_cache.push(("pv_cycle".to_string(), cut));
          }

          if !schedule::check_pv_route(self.data, &pv_path) {
            let (before_cut, after_cut) = self.pv_chain_cuts(&pv_path);
            self.stats.inc_cut_count(CutType::PvChain, 2);
            self.cut_cache.push(("pv_chain_before".to_string(), before_cut));
            self.cut_cache.push(("pv_chain_after".to_string(), after_cut));
          }

          pv_routes.insert(pv, pv_path);
        }


        if self.cut_cache.len() == old_cut_cache_len {
          let _span = error_span!("lp_sp").entered();
          info!("no heuristic cuts found, solving LP subproblem");
          let sp = TimingSubproblem::build(&self.sp_env, self.data, self.tasks, &av_routes, &pv_routes)?;

          let theta : Map<_, _> = get_var_values(&ctx, &self.mp_vars.theta)?.collect();
          trace!(?theta);
          let estimate = theta.values().sum::<f64>().round() as Time;
          let cuts = match sp.solve(self.tasks, estimate) {
            Err(e) => {
              // ctx.terminate();
              return Err(e);
              // return Ok(())
            }
            Ok(cuts) => cuts
          };


          for cut in cuts {
            let name = match &cut {
              BendersCut::Optimality(_) => {
                // continue;
                info!("LP optimality cut generated");
                format!("sp_opt[{}]", self.stats.inc_cut_count(CutType::LpOpt, 1))
              }
              BendersCut::Feasibility(_) => {
                info!("LP feasibility cut generated");
                format!("sp_feas[{}]", self.stats.inc_cut_count(CutType::LpFeas, 1))
              }
            };
            let cut = cut.into_ineq(self.sets, self.tasks, &self.mp_vars);
            trace!(cut=?cut.with_names(&self.var_names));
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
