use super::{sp::*, Phase};
use crate::solution::*;
use crate::tasks::{chain::ChainExt, TaskId};
use crate::utils::PermuteSliceClone;
use crate::*;
use fnv::FnvHashSet;
use grb::callback::{Callback, CbResult, MIPNodeCtx, MIPSolCtx, Where};
use grb::prelude::*;

use crate::logging::*;
use crate::model::mp::{MpVar, ObjWeights};
use anyhow::Context;
use experiment::{CycleHandling, Params, SpSolverKind};
use grb::constr::IneqExpr;
use sawmill::InferenceModel;
use serde::{Deserialize, Serialize};
use slurm_harray::Experiment;
use smallvec::SmallVec;
use std::collections::{HashMap, HashSet};
use std::env::var;
use std::fmt;
use std::fmt::Debug;
use std::hash::Hash;
use std::io::Write;
use variant_count::VariantCount;

#[derive(Debug, Clone)]
pub enum CbError {
  Assertion,
  // InvalidBendersCut{ estimate: Cost, obj: Cost, solution: Solution },
  InvalidBendersCut {
    estimate: Cost,
    obj: Cost,
    solution: (),
  },
  Status(grb::Status),
  Other(String),
}

impl fmt::Display for CbError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    f.write_str("Callback error: ")?;
    match self {
      CbError::Status(s) => f.write_fmt(format_args!("unexpected status: {:?}", s))?,
      CbError::Assertion => f.write_str("an assertion failed")?,
      CbError::InvalidBendersCut { estimate, obj, .. } => f.write_fmt(format_args!(
        "invalid Benders estimate ( estimate = {} > {} = obj )",
        estimate, obj
      ))?,
      CbError::Other(s) => f.write_str(s)?,
    }
    Ok(())
  }
}

impl std::error::Error for CbError {}

#[derive(VariantCount, Copy, Clone, Debug, Serialize, Deserialize, Eq, PartialEq, Hash)]
#[repr(usize)]
#[serde(rename_all = "snake_case")]
pub enum CutType {
  // Optimality cuts
  LpOpt = 0,
  EndTime,
  // Feasibility cuts
  LpCycle,
  LpPath,
  AvCycle,
  AvChainForwardTourn,
  AvChainFullTourn,
  AvChainFixStartFullTourn,
  AvChainFixEndFullTourn,
  AvChainInfork,
  AvChainOutfork,
  PvCycle,
  PvChainInfork,
  PvChainOutfork,
}

type CutCountArray = [u64; CutType::VARIANT_COUNT];

fn iter_cut_counts<'a>(arr: &'a CutCountArray) -> impl Iterator<Item = (CutType, u64)> + 'a {
  arr.iter().enumerate().map(|(cut_ty, &cnt)| {
    // Safety: We know the discriminant of `CutType` is always a valid index because the only
    // way to create a CbStats struct is to call `default()`, which allocates a `Vec` of length
    // equal to the number of variants.
    let cut_ty: CutType = unsafe { std::mem::transmute(cut_ty) };
    (cut_ty, cnt)
  })
}

impl CutType {
  pub fn is_opt_cut(&self) -> bool {
    matches!(self, Self::EndTime | Self::LpOpt)
  }
}

mod cb_stats_serde {
  use super::*;
  use serde::ser::SerializeMap;
  use serde::{Deserializer, Serializer};

  pub fn serialize<S: Serializer>(arr: &CutCountArray, s: S) -> Result<S::Ok, S::Error> {
    let mut m = s.serialize_map(Some(arr.len()))?;
    for (ty, n) in iter_cut_counts(arr) {
      m.serialize_entry(&ty, &n)?;
    }
    m.end()
  }

  pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<CutCountArray, D::Error> {
    let map = Map::<CutType, u64>::deserialize(d)?;
    let mut arr = CutCountArray::default();
    for (ty, n) in map {
      let idx: usize = unsafe { std::mem::transmute(ty) };
      arr[idx] = n;
    }
    Ok(arr)
  }
}

#[derive(Clone, Deserialize, Serialize, Debug)]
pub struct CbStats {
  #[serde(with = "cb_stats_serde")]
  n_cuts: CutCountArray,
  n_cuts_total: u64,
  n_mipsol: u64,
  n_subproblems_global: u64,
  n_subproblems: u64,
  subproblem_cover_sizes: Vec<SmallVec<[SpSolnIteration; 11]>>, // the SmallVec is 32 bytes total size
}

impl std::default::Default for CbStats {
  fn default() -> Self {
    CbStats {
      n_cuts: [0; CutType::VARIANT_COUNT],
      n_cuts_total: 0,
      n_mipsol: 0,
      n_subproblems: 0,
      n_subproblems_global: 0,
      subproblem_cover_sizes: Vec::new(),
    }
  }
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct SpSolnIteration {
  cover_size: u8,
  n_constraints: u8,
}

impl CbStats {
  #[inline]
  pub fn inc_cut_count(&mut self, cut_ty: CutType, inc: u64) -> u64 {
    #[cfg(debug_assertions)]
    let val = self
      .n_cuts
      .get_mut(cut_ty as usize)
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

  pub fn get_cut_counts<'a>(&'a self) -> impl Iterator<Item = (CutType, u64)> + 'a {
    iter_cut_counts(&self.n_cuts)
  }

  pub fn print_cut_counts(&self) {
    for (cut_ty, num) in self.get_cut_counts() {
      println!("Num {:?} Cuts: {}", cut_ty, num);
    }
  }

  fn inc_and_return_old(val: &mut u64) -> u64 {
    std::mem::replace(val, *val + 1)
  }

  pub fn inc_n_mipsol(&mut self) -> u64 {
    Self::inc_and_return_old(&mut self.n_mipsol)
  }

  pub fn inc_n_sp(&mut self) -> u64 {
    Self::inc_and_return_old(&mut self.n_subproblems) + self.n_subproblems_global
  }

  pub fn finish_phase(&mut self) -> Self {
    let mut new = CbStats::default();
    new.n_subproblems_global = self.n_subproblems_global + self.n_subproblems;
    std::mem::replace(self, new)
  }

  pub fn subproblem_cover_sizes(&mut self, sizes: impl IntoIterator<Item = (u8, u8)>) {
    self.subproblem_cover_sizes.push(
      sizes
        .into_iter()
        .map(|(cover_size, n_constraints)| SpSolnIteration {
          cover_size,
          n_constraints,
        })
        .collect(),
    );
  }
}

#[derive(Debug, Clone, Default)]
pub struct InfeasibilityTracker {
  cycles: HashMap<Vec<SpConstr>, usize>,
  paths: HashMap<Vec<SpConstr>, usize>,
}

impl InfeasibilityTracker {
  pub fn av_cycle(&mut self, lu: &Lookups, cycle: &AvCycle) {
    let mut sorted_cycle: Vec<_> = cycle.sp_constraints(lu).collect();
    sorted_cycle.sort_unstable();
    let count = self.cycles.entry(sorted_cycle).or_insert(0);
    InfeasibilityTracker::check_and_inc_count(count, cycle);
  }

  pub fn lp_iis(&mut self, iis: &Iis) {
    let mut cons: Vec<_> = iis.constraints().iter().copied().collect();
    cons.sort_unstable();
    let count = match iis {
      Iis::Path(_) => self.paths.entry(cons).or_insert(0),
      Iis::Cycle(_) => self.cycles.entry(cons).or_insert(0),
    };
    InfeasibilityTracker::check_and_inc_count(count, iis);
  }

  #[inline(always)]
  fn check_and_inc_count<T: Debug>(count: &mut usize, iis: T) {
    if *count > 100 {
      error!(?count, ?iis, "IIS is not being cut off");
      panic!("bugalug")
    } else if *count > 0 {
      warn!(?count, ?iis, "IIS has been encountered before")
    }
    *count += 1;
  }
}

#[derive(Debug, Clone)]
pub struct CachedCut {
  ty: CutType,
  idx: u64,
  expr: IneqExpr,
}

pub type ThetaVals = Map<(Avg, Task), Time>;

pub struct Cb<'a> {
  pub mp_obj: ObjWeights,
  pub inference_model: &'a sawmill::InferenceModel<MpVar, SpConstr>,
  pub lu: &'a Lookups,
  pub params: &'a Params,
  pub var_names: Map<Var, String>,
  // needs to be owned to avoid mutability issues with the Model object.
  pub mp_vars: super::mp::MpVars,
  pub stats: CbStats,
  // Lazy constraints added to the model
  pub added_lazy_constraints: Vec<CachedCut>,
  // Lazy constraints discovered, but not yet added as lazy constraints.
  pub stored_lazy_constraints: Vec<CachedCut>,

  pub sol_log: Option<std::io::BufWriter<std::fs::File>>,
  pub phase: Phase,

  #[cfg(debug_assertions)]
  pub infeasibilities: InfeasibilityTracker,
  #[cfg(debug_assertions)]
  pub var_vals: Map<Var, f64>,
  #[cfg(debug_assertions)]
  pub error: Option<CbError>,

  pub cached_solution: Option<(MpSolution, ThetaVals)>,
}

impl<'a> Cb<'a> {
  pub fn new(
    lu: &'a Lookups,
    exp: &'a experiment::ApvrpExp,
    mp: &super::mp::TaskModelMaster,
    inf_model: &'a InferenceModel<MpVar, SpConstr>,
    initial_phase: Phase,
    obj: ObjWeights,
  ) -> Result<Self> {
    let stats = CbStats::default();
    let var_names: Map<_, _> = {
      let vars = mp.model.get_vars()?;
      let names = mp
        .model
        .get_obj_attr_batch(attr::VarName, vars.iter().copied())?;
      vars.iter().copied().zip(names).collect()
    };

    let sol_log = if exp.aux_params.soln_log {
      let log = exp.get_output_path(&exp.outputs.solution_log);
      let log = std::fs::OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(log)?;
      let log = std::io::BufWriter::new(log);
      Some(log)
    } else {
      None
    };

    Ok(Cb {
      lu,
      inference_model: inf_model,
      phase: initial_phase,
      mp_vars: mp.vars.clone(),
      mp_obj: obj,
      stats,
      added_lazy_constraints: Vec::new(),
      stored_lazy_constraints: Vec::new(),
      var_names,
      params: &exp.parameters,
      sol_log,
      #[cfg(debug_assertions)]
      infeasibilities: Default::default(),
      #[cfg(debug_assertions)]
      var_vals: Map::default(),
      #[cfg(debug_assertions)]
      error: None,
      cached_solution: None,
    })
  }

  #[cfg(debug_assertions)]
  fn cut_check(&self, cut: &IneqExpr, ty: &CutType) {
    let (lhs, rhs) = cut.evaluate(&self.var_vals);
    let _s = trace_span!("cut_check", ?lhs, ?rhs, cut=?cut.with_names(&self.var_names)).entered();
    if lhs <= rhs + 1e-6 {
      error!("cut is not violated!");
      panic!("bugalug");
    }

    if !ty.is_opt_cut() {
      let coeff_error = || {
        error!("feasiblity cut has a coefficient != 1");
        panic!("bugalug");
      };
      match &cut.lhs {
        Expr::Linear(l) => {
          if !l.iter_terms().all(|(_, &coeff)| (coeff - 1.0).abs() < 1e-6) {
            coeff_error()
          }
        }
        Expr::Term(coeff, _) => {
          if (coeff - 1.0).abs() > 1e-6 {
            coeff_error()
          }
        }
        _ => {}
      }
    }

    if self.params.pvcg
      && matches!(
        ty,
        CutType::PvCycle | CutType::PvChainInfork | CutType::PvChainOutfork
      )
    {
      error!("shouldn't need to add PV-cuts with PVCG");
      panic!("bugalug")
    }

    trace!("ok")
  }

  #[tracing::instrument(level = "info", skip(self, cut), fields(idx))]
  pub fn enqueue_cut(&mut self, cut: IneqExpr, ty: CutType) {
    let idx = self.stats.inc_cut_count(ty, 1);
    tracing::Span::current().record("idx", &idx);

    #[cfg(debug_assertions)]
    {
      self.cut_check(&cut, &ty);
    }

    let cut = CachedCut { ty, idx, expr: cut };
    if matches!(&self.phase, Phase::NoAvTTCost) && ty.is_opt_cut() {
      info!("store cut");
      self.stored_lazy_constraints.push(cut);
    } else {
      info!("add cut");
      self.added_lazy_constraints.push(cut);
    }
  }

  pub fn flush_cut_cache(&mut self, mp: &mut Model) -> Result<()> {
    let cuts = self
      .added_lazy_constraints
      .drain(0..)
      .chain(self.stored_lazy_constraints.drain(0..));

    for CachedCut { ty, idx, expr, .. } in cuts {
      let name = format!("{:?}[{}|{}]", ty, self.phase.idx(), idx);
      mp.add_constr(&name, expr)?;
    }

    mp.update()?;
    Ok(())
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_cycle_cut(&mut self, cycle: &AvCycle) {
    // TODO: might be able to use the IIS stronger cut?
    #[cfg(debug_assertions)]
    {
      self.infeasibilities.av_cycle(self.lu, cycle);
    }

    let cycle_tasks = &cycle[..cycle.len() - 1];

    let ysum = cycle_tasks
      .iter()
      .cartesian_product(cycle_tasks.iter())
      .flat_map(|(&t1, &t2)| self.mp_vars.y_sum_av_possibly_empty(self.lu, t1, t2))
      .grb_sum();

    // cycle is a list of n+1 nodes (start = end), which form n arcs
    self.enqueue_cut(c!(ysum <= cycle.len() - 2), CutType::AvCycle);
  }

  #[inline]
  fn av_chain_fork_lhs(&self, chain: &[Task]) -> Expr {
    // TODO: could forward-tournament here.
    chain
      .iter()
      .tuple_windows()
      .flat_map(move |(&t1, &t2)| self.mp_vars.y_sum_av(self.lu, t1, t2))
      .grb_sum()
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_chain_infork_cut(&mut self, chain: &[Task]) {
    let chain = &chain[1..];
    let first_task = *chain
      .first()
      .expect("chain should have at least three tasks");
    let n = chain.len() - 1; // number of arcs in legal chain
    let y = &self.mp_vars.y;
    debug_assert!(schedule::check_av_route(self.lu, chain));

    let rhs_sum = chain
      .legal_before(self.lu)
      .flat_map(|t| {
        self
          .lu
          .sets
          .av_groups()
          .filter_map(move |av| y.get(&(av, t, first_task)).copied())
      })
      .grb_sum();

    let lhs = self.av_chain_fork_lhs(chain);

    self.enqueue_cut(c!(lhs <= n - 1 + rhs_sum), CutType::AvChainInfork);
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_chain_outfork_cut(&mut self, chain: &[Task]) {
    let chain = &chain[..chain.len() - 1];
    let last_task = *chain
      .last()
      .expect("chain should have at least three tasks");
    let n = chain.len() - 1; // number of arcs in legal chain
    let y = &self.mp_vars.y;

    debug_assert!(schedule::check_av_route(self.lu, chain));

    let rhs_sum = chain
      .legal_after(self.lu)
      .flat_map(|t2| {
        self
          .lu
          .sets
          .av_groups()
          .filter_map(move |av| y.get(&(av, last_task, t2)).copied())
      })
      .grb_sum();

    let lhs = self.av_chain_fork_lhs(chain);
    self.enqueue_cut(c!(lhs <= n - 1 + rhs_sum), CutType::AvChainOutfork);
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_chain_tournament_cut(&mut self, chain: &[Task]) {
    let n = chain.len() - 1;
    // Are all permutations illegal?
    let mut all_illegal = true;
    // Are all permutations where the first Task in the chain is held fixed illegal?
    let mut all_illegal_first_fixed = true;
    // Are all permutations where the last Task in the chain is held fixed illegal?
    let mut all_illegal_last_fixed = true;

    for c in chain.permutations() {
      let c = c.as_slice();
      if schedule::check_av_route(self.lu, c) {
        all_illegal = false;

        if c[0] == chain[0] {
          all_illegal_first_fixed = false;
        }

        if c[n] == chain[n] {
          all_illegal_last_fixed = false
        }

        if !all_illegal_first_fixed && !all_illegal_last_fixed && !all_illegal {
          break;
        }
      }
    }

    // Forward tournament
    let mut lhs = chain
      .iter()
      .enumerate()
      .flat_map(|(k, &t1)| chain[(k + 1)..].iter().map(move |&t2| (t1, t2)))
      .flat_map(|(t1, t2)| self.mp_vars.y_sum_av_possibly_empty(self.lu, t1, t2))
      .grb_sum();

    let mut add_similar_tasks_to_lhs = |i, j| {
      for y in self
        .mp_vars
        .y_sum_av_possibly_empty(self.lu, chain[i], chain[j])
      {
        lhs += y;
      }
    };

    // Full tournament, add backarcs as well
    let ty = if all_illegal {
      for i in 1..=n {
        for j in 0..i {
          add_similar_tasks_to_lhs(i, j);
        }
      }
      CutType::AvChainFullTourn

      // Partial full tournament, add backarcs for all except chain[n]
    } else if all_illegal_last_fixed {
      for i in 1..n {
        for j in 0..i {
          add_similar_tasks_to_lhs(i, j)
        }
      }
      CutType::AvChainFixEndFullTourn

      // Partial full tournament, add backarcs for all except chain[0]
    } else if all_illegal_first_fixed {
      for i in 1..=n {
        for j in 1..i {
          add_similar_tasks_to_lhs(i, j)
        }
      }
      CutType::AvChainFixStartFullTourn

      // Fallback
    } else {
      CutType::AvChainForwardTourn
    };

    self.enqueue_cut(c!(lhs <= chain.len() - 2), ty)
  }

  /// Given `chain`, which violates the AV timing constraints, find the shortest subchain which still violates
  /// the timing constraints.
  #[instrument(level = "trace", skip(self))]
  fn shorten_illegal_av_chain<'b>(&self, chain: &'b [Task]) -> &'b [Task] {
    for l in 3..=chain.len() {
      for start_pos in 0..=(chain.len() - l) {
        let c = &chain[start_pos..(start_pos + l)];
        trace!(?c);
        if !schedule::check_av_route(self.lu, c) {
          return c;
        }
      }
    }
    error!(?chain, "chain must be illegal"); // FIXME 2-phase min-VI fails here for seed 92821, instances 147, 159
    unreachable!()
  }

  /// Given `chain`, which violates the PV timing constraints, find the shortest subchain which still violates
  /// the timing constraints.
  #[instrument(level = "trace", skip(self))]
  fn shorten_illegal_pv_chain<'b>(&self, chain: &'b [PvTask]) -> &'b [PvTask] {
    for l in 3..=chain.len() {
      for start_pos in 0..=(chain.len() - l) {
        let c = &chain[start_pos..(start_pos + l)];
        trace!(?c);
        if !schedule::check_pv_route(self.lu, c) {
          return c;
        }
      }
    }
    error!("chain must be illegal");
    unreachable!()
  }

  /// Given a `cycle` where `cycle.first() == cycle.last()`, returns the shortest acyclic subchain
  /// that violates the AV timing constraints, if one exists.
  fn illegal_av_chain_in_cycle(&self, cycle: &[Task]) -> Option<Vec<Task>> {
    debug_assert_eq!(cycle.first(), cycle.last());
    // If cycle is
    // [0, 1, 2, 3, 4, 0] // .len() = 6
    // then cycle rep holds
    // [0, 1, 2, 3, 4, 0, 1, 2, 3]
    // which allows us to sweep all the cyclic permutations with slices
    let cycle_rep = {
      let mut v = Vec::with_capacity(2 * cycle.len() - 3);
      v.extend_from_slice(cycle);
      v.extend_from_slice(&cycle[1..cycle.len() - 2]);
      v
    };

    for l in 3..=(cycle.len() - 1) {
      for start_pos in 0..(cycle.len() - 1) {
        let c = &cycle_rep[start_pos..(start_pos + l)];
        if !schedule::check_av_route(self.lu, c) {
          return Some(c.to_vec());
        }
      }
    }

    None
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn pv_cycle_cut(&mut self, cycle: &PvCycle) {
    let xsum = cycle
      .unique_tasks()
      .iter()
      .flat_map(|t| &self.lu.tasks.task_to_pvtasks[&self.lu.tasks.pvtask_to_task[t]])
      .map(|t| self.mp_vars.x[t])
      .grb_sum();
    // cycle is a list of n+1 nodes (start == end), which form n arcs
    self.enqueue_cut(c!(xsum <= cycle.len() - 2), CutType::PvCycle);
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn pv_chain_infork_cut(&mut self, chain: &[PvTask]) {
    // FIXME sum over all p?
    let chain = &chain[1..];
    let rhs_sum = chain
      .legal_before(self.lu)
      .map(|t| self.mp_vars.x[&t])
      .grb_sum();

    let lhs_sum = chain.iter().map(|t| self.mp_vars.x[t]).grb_sum();

    self.enqueue_cut(
      c!(lhs_sum <= chain.len() - 1 + rhs_sum),
      CutType::PvChainInfork,
    );
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn pv_chain_outfork_cut(&mut self, chain: &[PvTask]) {
    // FIXME sum over all p?
    let chain = &chain[..chain.len() - 1];
    let rhs_sum = chain
      .legal_after(self.lu)
      .flat_map(|t| &self.lu.tasks.pvtask_to_similar_pvtask[&t])
      .map(|t| self.mp_vars.x[t])
      .grb_sum();

    let lhs_sum = chain.iter().map(|t| self.mp_vars.x[t]).grb_sum();

    self.enqueue_cut(
      c!(lhs_sum <= chain.len() - 1 + rhs_sum),
      CutType::PvChainOutfork,
    );
  }

  fn separate_pv_cuts(&mut self, routes: &[PvRoute], cycles: &[PvCycle]) -> Result<()> {
    if self.params.cycle_cuts {
      for cycle in cycles {
        self.pv_cycle_cut(cycle);
      }
    }

    if !self.params.pv_fork_cuts.is_empty() {
      for route in routes {
        if !schedule::check_pv_route(self.lu, route) {
          let chain = self.shorten_illegal_pv_chain(route);
          let n = chain.len() as u32;
          if self.params.pv_fork_cuts.contains(&n) {
            self.pv_chain_infork_cut(chain);
            self.pv_chain_outfork_cut(chain);
          }
        }
      }
    }

    Ok(())
  }

  fn av_chain_cuts(&mut self, chain: &[Task]) {
    let n = chain.len() as u32;
    if self.params.av_fork_cuts.contains(&n) {
      self.av_chain_infork_cut(&chain);
      self.av_chain_outfork_cut(&chain);
    }

    if self.params.av_tournament_cuts.contains(&n) {
      self.av_chain_tournament_cut(&chain);
    }
  }

  #[instrument(level = "trace", skip(self))]
  fn endtime_cut(&mut self, finish_time: Time, route: &AvRoute) {
    let av_route_nd = route.without_depots();
    let av = route.av();
    let n = av_route_nd.len() - 1;

    let mut lhs = finish_time * self.mp_vars.y[&(av, av_route_nd[n], self.lu.tasks.ddepot)];

    for k in 0..n {
      let partial_finish_time = schedule::av_route_finish_time(self.lu, &av_route_nd[k + 1..]);
      if partial_finish_time != finish_time {
        lhs += (finish_time - partial_finish_time)
          * (self.mp_vars.y[&(av, av_route_nd[k], av_route_nd[k + 1])] - 1)
      }
    }

    self.enqueue_cut(
      c!(lhs <= self.mp_vars.theta[&(av, av_route_nd[n])]),
      CutType::EndTime,
    );
  }

  fn separate_av_cuts(
    &mut self,
    theta: &Map<(Avg, Task), Time>,
    routes: &[AvRoute],
    cycles: &[AvCycle],
  ) -> Result<()> {
    if self.params.cycle_cuts {
      for cycle in cycles {
        if let Some(chain) = self.illegal_av_chain_in_cycle(&cycle) {
          if chain.len() <= 3 {
            self.av_chain_infork_cut(&chain);
            self.av_chain_outfork_cut(&chain);
          } else {
            self.av_chain_tournament_cut(&chain);
          }
        } else {
          self.av_cycle_cut(&cycle);
        }
      }
    };

    if !self.params.av_fork_cuts.is_empty()
      || !self.params.av_tournament_cuts.is_empty()
      || self.params.endtime_cuts
    {
      for route in routes {
        if !schedule::check_av_route(self.lu, &route) {
          let chain = self.shorten_illegal_av_chain(route.without_depots());
          let n = chain.len() as u32;
          if self.params.av_fork_cuts.contains(&n) {
            self.av_chain_infork_cut(&chain);
            self.av_chain_outfork_cut(&chain);
          }
          if self.params.av_tournament_cuts.contains(&n) {
            self.av_chain_tournament_cut(&chain);
          }
        } else if self.params.endtime_cuts {
          let finish_time = schedule::av_route_finish_time(self.lu, route.without_depots());
          let estimate = theta[&(route.av(), *route.theta_task())];
          if estimate < finish_time {
            self.endtime_cut(finish_time, route);
          }
        }
      }
    }

    Ok(())
  }

  #[allow(unused_variables)]
  fn update_var_values(&mut self, ctx: &MIPSolCtx) -> Result<()> {
    #[cfg(debug_assertions)]
    {
      let vars = self
        .mp_vars
        .x
        .values()
        .chain(self.mp_vars.y.values())
        .chain(self.mp_vars.theta.values());

      for (&var, val) in vars.clone().zip(ctx.get_solution(vars)?) {
        self.var_vals.insert(var, val);
      }
    }

    Ok(())
  }

  fn get_av_routes(&self, ctx: &MIPSolCtx) -> Result<(Vec<AvRoute>, Vec<AvCycle>)> {
    let task_pairs = get_var_values(ctx, &self.mp_vars.y)?.map(|(key, _)| key);
    Ok(construct_av_routes(task_pairs))
  }

  fn get_pv_routes(&self, ctx: &MIPSolCtx) -> Result<(Vec<PvRoute>, Vec<PvCycle>)> {
    let task_pairs = get_var_values(ctx, &self.mp_vars.x)?.map(|(key, _)| key);
    Ok(construct_pv_routes(task_pairs))
  }

  #[instrument(level = "info", name = "save_sol", skip(self, sol))]
  fn update_cached_solution(&mut self, mut sol: MpSolution, obj: Cost, theta: ThetaVals) -> bool {
    match self.cached_solution.as_ref() {
      Some((sp, _)) => {
        let saved_obj = sp.objective.unwrap();
        if saved_obj < obj {
          info!(
            saved_obj,
            "solution discarded: previously cached solution is better"
          );
          return false;
        }
      }
      None => {}
    }
    sol.objective = Some(obj);
    self.cached_solution = Some((sol, theta));
    info!("saved new solution for final phase");
    true
  }

  fn suggest_solution(&self, ctx: &MIPNodeCtx, sol: &MpSolution, theta: &ThetaVals) -> Result<()> {
    debug_assert!(sol.av_cycles.is_empty());
    debug_assert!(sol.pv_cycles.is_empty());
    let xvars = sol
      .pv_routes
      .iter()
      .flat_map(|r| r.iter().map(|t| self.mp_vars.x[t]));
    let yvars = sol
      .av_routes
      .iter()
      .flat_map(|r| r.iter_y_vars().map(|key| self.mp_vars.y[&key]));

    let grb_soln = xvars.chain(yvars).map(|var| (var, 1.0)).chain(
      theta
        .iter()
        .map(|(key, val)| (self.mp_vars.theta[key], *val as f64)),
    );

    #[allow(unused_variables)]
    let obj = ctx.set_solution(grb_soln)?;

    #[cfg(debug_assertions)]
    match obj.map(|o| o.round() as Cost) {
      None => {
        error!("cached solution was infeasible");
        panic!("bugalug");
      }
      Some(obj) => {
        debug!(grb_obj = obj, stored_obj = sol.objective.unwrap());
        if obj > sol.objective.unwrap() {
          error!(
            gurobi_obj = obj,
            "Gurobi-computed objective implies dodgy optimality cuts"
          );
          panic!("bugalug");
        }
      }
    }
    info!("posted solution");
    Ok(())
  }
}

impl<'a> Callback for Cb<'a> {
  fn callback(&mut self, w: Where) -> CbResult {
    // TODO - save best global solution in no-AVTT-cost phase
    //  note: the best solution found wrt to the final phase may not be the
    //  the same as the best solution found wrt to the initial phase
    match w {
      Where::MIPSol(ctx) => {
        let icu = self.stats.inc_n_mipsol();
        let _span = info_span!("cb", icu, ph = self.phase.idx()).entered();
        debug!("integer MP solution found");

        #[cfg(debug_assertions)]
        self.update_var_values(&ctx)?;

        let new_lazy_constr_start = self.added_lazy_constraints.len();

        let (pv_routes, pv_cycles) = self.get_pv_routes(&ctx)?;
        self.separate_pv_cuts(&pv_routes, &pv_cycles)?;

        if self.added_lazy_constraints.len() > new_lazy_constr_start {
          info!(
            ncuts = self.added_lazy_constraints.len() - new_lazy_constr_start,
            "heuristic cuts added (PV only)"
          );
        } else {
          let (av_routes, av_cycles) = self.get_av_routes(&ctx)?;
          let theta: Map<_, _> =
            get_var_values_mapped(&ctx, &self.mp_vars.theta, |t| t.round() as Time)?.collect();
          self.separate_av_cuts(&theta, &av_routes, &av_cycles)?;

          if self.added_lazy_constraints.len() > new_lazy_constr_start {
            info!(
              ncuts = self.added_lazy_constraints.len() - new_lazy_constr_start,
              "heuristic cuts added"
            );
          } else {
            let sp_idx = self.stats.inc_n_sp();
            let _span = error_span!("sp", sp_idx, estimate = tracing::field::Empty).entered();
            info!("no heuristic cuts found, solving LP subproblem");
            let sol = MpSolution {
              objective: None,
              av_cycles,
              av_routes,
              pv_routes,
              pv_cycles,
            };
            if let Some(sol_log) = self.sol_log.as_mut() {
              serde_json::to_writer(&mut *sol_log, &sol.to_serialisable())?; // need to re-borrow here
              write!(sol_log, "\n")?;
            }
            let active_vars: Set<_> = sol.mp_vars().collect();
            let mut active_sp_cons = self.inference_model.implied_constraints(&active_vars);
            self
              .inference_model
              .remove_dominated_constraints(&mut active_sp_cons);

            debug!(?theta);
            let estimate: Time = theta.values().sum();
            tracing::span::Span::current().record("estimate", &estimate);
            let correct_theta = match self.params.sp {
              SpSolverKind::Dag => {
                let mut sp =
                  dag::GraphModel::build(self.lu, &active_sp_cons, sol.sp_objective_tasks());
                sp.solve_subproblem_and_add_cuts(self, &active_vars, &theta, estimate)?
              }
              SpSolverKind::Lp => {
                let mut sp =
                  lp::TimingSubproblem::build(self.lu, &active_sp_cons, sol.sp_objective_tasks())?;
                sp.solve_subproblem_and_add_cuts(self, &active_vars, &theta, estimate)?
              }
            };

            if let Some(theta) = correct_theta {
              let sp_obj: Time = theta.values().sum();
              if sp_obj == estimate {
                info!("estimate is correct!")
              } else if estimate > sp_obj {
                warn!(sp_obj, "estimate is too large")
              }

              if matches!(self.phase, Phase::NoAvTTCost) {
                let obj = ctx.obj()?.round() as Cost + sp_obj;
                self.update_cached_solution(sol, obj, theta);
              }
            }
          }
        }

        for cut in &self.added_lazy_constraints[new_lazy_constr_start..] {
          ctx.add_lazy(cut.expr.clone())?;
        }
      }

      Where::MIPNode(ctx) if self.phase.is_final() => {
        if let Some((sol, theta)) = &self.cached_solution {
          let inc_obj = ctx.obj_best()?.round() as Time;
          let saved_obj = sol.objective.unwrap();
          let _s = error_span!("post_sol", inc_obj, saved_obj).entered();

          if saved_obj < inc_obj {
            self.suggest_solution(&ctx, sol, theta)?;
          } else {
            info!("ditching saved solution: incumbent is better or equal");
            self.cached_solution = None;
          }
        }
      }

      _ => {}
    }
    Ok(())
  }
}
