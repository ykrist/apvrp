use crate::*;
use crate::tasks::{TaskId, chain::{ChainExt}};
use crate::solution::*;
use grb::prelude::*;
use grb::callback::{Callback, Where, CbResult, MIPSolCtx};
use fnv::FnvHashSet;
use super::sp::*;
use crate::model::sp::PathIis;

#[macro_use]
use crate::logging::*;
use std::fmt;
use grb::constr::IneqExpr;
use variant_count::VariantCount;
use anyhow::Context;
use std::hash::Hash;
use std::env::var;
use std::collections::{HashSet, HashMap};
use std::io::Write;
use experiment::{Params, SpSolverKind};
use slurm_harray::{Experiment};
use crate::utils::PermuteSliceClone;
use smallvec::SmallVec;


#[derive(Debug, Clone)]
pub enum CbError {
  Assertion,
  // InvalidBendersCut{ estimate: Cost, obj: Cost, solution: Solution },
  InvalidBendersCut{ estimate: Cost, obj: Cost, solution: () },
  Status(grb::Status),
  Other(String),
}

impl fmt::Display for CbError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    f.write_str("Callback error: ")?;
    match self {
      CbError::Status(s) => f.write_fmt(format_args!("unexpected status: {:?}", s))?,
      CbError::Assertion => f.write_str("an assertion failed")?,
      CbError::InvalidBendersCut { estimate, obj, .. } =>
        f.write_fmt(format_args!("invalid Benders estimate ( estimate = {} > {} = obj )", estimate, obj))?,
      CbError::Other(s) => f.write_str(s)?,
    }
    Ok(())
  }
}

impl std::error::Error for CbError {}


#[derive(VariantCount, Copy, Clone, Debug)]
#[repr(usize)]
pub enum CutType {
  LpOpt = 0,
  LpFeas,
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
  EndTime,
}

impl CutType {
  pub fn is_opt_cut(&self) -> bool {
    matches!(self, Self::EndTime | Self::LpOpt)
  }
}

#[derive(Clone)]
pub struct CbStats {
  n_cuts: Vec<usize>,
  n_cuts_total: usize,
  n_mipsol: usize,
  n_subproblems: usize,
}

impl std::default::Default for CbStats {
  fn default() -> Self {
    CbStats {
      n_cuts: vec![0; CutType::VARIANT_COUNT],
      n_cuts_total: 0,
      n_mipsol: 0,
      n_subproblems: 0
    }
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

  fn inc_and_return_old(val: &mut usize) -> usize {
    std::mem::replace(val, *val + 1)
  }

  pub fn inc_n_mipsol(&mut self) -> usize {
    Self::inc_and_return_old(&mut self.n_mipsol)
  }

  pub fn inc_n_sp(&mut self) -> usize {
    Self::inc_and_return_old(&mut self.n_subproblems)
  }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Phase {
  NoWaitCost,
  Final,
}

impl Phase {
  #[inline]
  pub fn next(&self) -> Self {
    use Phase::*;
    match self {
      NoWaitCost => Final,
      _ => unimplemented!("in Final phase"),
    }
  }

  #[inline]
  pub fn set_next(&mut self) {
    *self = self.next();
  }
}

#[derive(Debug, Clone, Default)]
pub struct InfeasibilityTracker {
  av_cycles: HashMap<Vec<Task>, usize>,
  path_iis: Set<PathIis>,
}

impl InfeasibilityTracker {
  pub fn av_cycle(&mut self, cycle: &[Task]) {
    let mut sorted_cycle = cycle.to_vec();
    sorted_cycle.sort_by_key(|t| t.id());
    let count = self.av_cycles.entry(sorted_cycle).or_insert(0);
    if *count > 0 { warn!(?count, ?cycle, "cycle has been encountered before") }
    *count += 1;
  }

  pub fn path_iis(&mut self, iis: &PathIis) {
    if self.path_iis.contains(iis) {
      error!(?iis, "IIS has been seen before");
      panic!("bugalug")
    } else {
      self.path_iis.insert(iis.clone());
    }
  }
}

pub struct Cb<'a> {
  pub lu: &'a Lookups,
  pub params: &'a Params,
  pub var_names: Map<Var, String>,
  pub mp_vars: super::mp::MpVars,
  // needs to be owned to avoid mutability issues with the Model object.
  pub stats: CbStats,
  pub cut_cache: Vec<(CutType, usize, IneqExpr)>,
  #[cfg(debug_assertions)]
  pub infeasibilities: InfeasibilityTracker,
  #[cfg(debug_assertions)]
  pub var_vals: Map<Var, f64>,
  #[cfg(debug_assertions)]
  pub error: Option<CbError>,
  pub sol_log: Option<std::io::BufWriter<std::fs::File>>
}

impl<'a> Cb<'a> {

  pub fn new(lu: &'a Lookups, exp: &'a experiment::ApvrpExp, mp: &super::mp::TaskModelMaster) -> Result<Self> {
    let stats = CbStats::default();
    let var_names: Map<_, _> = {
      let vars = mp.model.get_vars()?;
      let names = mp.model.get_obj_attr_batch(attr::VarName, vars.iter().copied())?;
      vars.iter().copied()
        .zip(names)
        .collect()
    };

    let sol_log = if exp.parameters.soln_log {
      let log = exp.get_output_path(&exp.outputs.solution_log);
      let log = std::fs::OpenOptions::new().write(true).truncate(true).create(true).open(log)?;
      let log = std::io::BufWriter::new(log);
      Some(log)
    } else { None };

    Ok(Cb {
      lu,
      mp_vars: mp.vars.clone(),
      stats,
      cut_cache: Vec::new(),
      var_names,
      params: &exp.parameters,
      sol_log,
      #[cfg(debug_assertions)]
      infeasibilities: Default::default(),
      #[cfg(debug_assertions)]
      var_vals: Map::default(),
      #[cfg(debug_assertions)]
      error: None,
    })
  }

  #[tracing::instrument(level = "info", skip(self, cut))]
  pub fn enqueue_cut(&mut self, cut: IneqExpr, ty: CutType) {
    let i = self.stats.inc_cut_count(ty, 1);

    #[cfg(debug_assertions)]
      let _s = {
      let (lhs, rhs) = cut.evaluate(&self.var_vals);
      let span = trace_span!("cut_check", ?lhs, ?rhs, cut=?cut.with_names(&self.var_names)).entered();
      if lhs <= rhs + 1e-6 {
        error!(index = i, "cut is not violated!");
        panic!("bugalug");
      }
      span
    };


    info!(index = i, "add cut");
    self.cut_cache.push((ty, i, cut));
  }

  pub fn flush_cut_cache(&mut self, mp: &mut Model) -> Result<()> {
    for (ty, i, cut) in self.cut_cache.drain(0..) {
      let name = format!("{:?}[{}]", ty, i);
      mp.add_constr(&name, cut)?;
    }
    Ok(())
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_cycle_cut(&mut self, cycle: &AvPath) {
    // TODO: might be able to use the IIS stronger cut?
    #[cfg(debug_assertions)] {
      self.infeasibilities.av_cycle(cycle);
    }

    let cycle_tasks = &cycle[..cycle.len() - 1];

    let ysum = cycle_tasks.iter()
      .cartesian_product(cycle_tasks.iter())
      .flat_map(|(&t1, &t2)|
        self.mp_vars.y_sum_av_possibly_empty(self.lu, t1, t2)
      )
      .grb_sum();

    // cycle is a list of n+1 nodes (start = end), which form n arcs
    self.enqueue_cut(c!(ysum <= cycle.len() - 2), CutType::AvCycle);
    // panic!()
  }

  #[inline]
  fn av_chain_fork_lhs(&self, chain: & [Task]) -> Expr {
    chain.iter()
      .tuple_windows()
      .flat_map(move |(&t1, &t2)|
        self.mp_vars.y_sum_av(self.lu, t1, t2)
      )
      .grb_sum()
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_chain_infork_cut(&mut self, chain: &[Task]) {
    let chain = &chain[1..];
    let first_task = *chain.first().expect("chain should have at least three tasks");
    let n = chain.len() - 1; // number of arcs in legal chain
    let y = &self.mp_vars.y;
    debug_assert!(schedule::check_av_route(self.lu, chain));

    let rhs_sum = chain.legal_before(self.lu)
      .flat_map(|t| {
        self.lu.sets.avs().filter_map(move |av| y.get(&(av, t, first_task)).copied())
      })
      .grb_sum();

    let lhs = self.av_chain_fork_lhs(chain);

    self.enqueue_cut(c!(lhs <= n - 1 + rhs_sum), CutType::AvChainInfork);
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_chain_outfork_cut(&mut self, chain: &[Task]) {
    let chain = &chain[..chain.len() - 1];
    let last_task = *chain.last().expect("chain should have at least three tasks");
    let n = chain.len() - 1;  // number of arcs in legal chain
    let y = &self.mp_vars.y;

    debug_assert!(schedule::check_av_route(self.lu, chain));

    let rhs_sum = chain.legal_after(self.lu)
      .flat_map(|t2| {
        self.lu.sets.avs().filter_map(move |av| y.get(&(av, last_task, t2)).copied())
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
    let mut lhs = chain.iter().enumerate()
      .flat_map(|(k, &t1)| chain[(k + 1)..].iter().map(move |&t2| (t1, t2)))
      .flat_map(|(t1, t2)| self.mp_vars.y_sum_av_possibly_empty(self.lu, t1, t2))
      .grb_sum();

    let mut add_similar_tasks_to_lhs = |i,j| {
      for y in self.mp_vars.y_sum_av_possibly_empty(self.lu, chain[i], chain[j]) {
        lhs += y;
      }
    };

    // Full tournament, add backarcs as well
    let ty = if all_illegal {
      for i in 1..=n {
        for j in 0..i {
          add_similar_tasks_to_lhs(i,j);
        }
      }
      CutType::AvChainFullTourn

      // Partial full tournament, add backarcs for all except chain[n]
    } else if all_illegal_last_fixed {
      for i in 1..n {
        for j in 0..i {
          add_similar_tasks_to_lhs(i,j)
        }
      }
      CutType::AvChainFixEndFullTourn

      // Partial full tournament, add backarcs for all except chain[0]
    } else if all_illegal_first_fixed {
      for i in 1..=n {
        for j in 1..i {
          add_similar_tasks_to_lhs(i,j)
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
  fn shorten_illegal_av_chain<'b>(&self, chain: &'b [Task]) -> &'b [Task] {
    for l in 3..=chain.len() {
      for start_pos in 0..(chain.len() - l) {
        let c = &chain[start_pos..(start_pos + l)];
        if !schedule::check_av_route(self.lu, c) {
          return c;
        }
      }
    }
    unreachable!("chain must be illegal")
  }

  /// Given `chain`, which violates the PV timing constraints, find the shortest subchain which still violates
  /// the timing constraints.
  fn shorten_illegal_pv_chain<'b>(&self, chain: &'b [PvTask]) -> &'b [PvTask] {
    for l in 3..=chain.len() {
      for start_pos in 0..(chain.len() - l) {
        let c = &chain[start_pos..(start_pos + l)];
        if !schedule::check_pv_route(self.lu, c) {
          return c;
        }
      }
    }
    unreachable!("chain must be illegal")
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
  fn pv_cycle_cut(&mut self, cycle: &PvPath) {
    debug_assert!(cycle.first() != cycle.last());
    // TODO tournament?
    let xsum = cycle.iter()
      .flat_map(|t| &self.lu.tasks.task_to_pvtasks[&self.lu.tasks.pvtask_to_task[t]])
      .map(|t| self.mp_vars.x[t])
      .grb_sum();
    // cycle is a list of n nodes (start != end), which form n-1 arcs
    self.enqueue_cut(c!(xsum <= cycle.len() - 1), CutType::PvCycle);
  }

  // #[allow(unused_variables)]
  // fn pv_chain_tournament_cut(&mut self, chain: &[Task]) {
  //   todo!()
  // }
  //
  #[allow(unused_variables)]
  #[tracing::instrument(level = "trace", skip(self))]
  fn pv_chain_infork_cut(&mut self, chain: &[PvTask]) {
    // FIXME sum over all p?
    let chain = &chain[1..];
    let rhs_sum = chain.legal_before(self.lu)
      .map(|t| self.mp_vars.x[&t])
      .grb_sum();

    let lhs_sum = chain.iter()
      .map(|t| self.mp_vars.x[t])
      .grb_sum();

    self.enqueue_cut(c!(lhs_sum <= chain.len() - 1 + rhs_sum), CutType::PvChainInfork);
  }

  #[allow(unused_variables)]
  #[tracing::instrument(level = "trace", skip(self))]
  fn pv_chain_outfork_cut(&mut self, chain: &[PvTask]) {
    // FIXME sum over all p?
    let chain = &chain[..chain.len() - 1];
    let rhs_sum = chain.legal_after(self.lu)
      .flat_map(|t| &self.lu.tasks.pvtask_to_similar_pvtask[&t])
      .map(|t| self.mp_vars.x[t])
      .grb_sum();

    let lhs_sum = chain.iter()
      .map(|t| self.mp_vars.x[t])
      .grb_sum();

    self.enqueue_cut(c!(lhs_sum <= chain.len() - 1 + rhs_sum), CutType::PvChainOutfork);
  }


  fn sep_pv_cuts(&mut self, ctx: &MIPSolCtx) -> Result<Vec<(Pv, PvPath)>> {
    let pv_tasks = get_tasks_by_pv(ctx, &self.mp_vars.x)?;
    let mut pv_routes = Vec::with_capacity(pv_tasks.len());
    for (&pv, tasks) in &pv_tasks {
      let (pv_path, pv_cycles) = construct_pv_route(tasks);

      for cycle in pv_cycles {
        self.pv_cycle_cut(&cycle);
      }

      if !schedule::check_pv_route(self.lu, &pv_path) {
        let chain = self.shorten_illegal_pv_chain(&pv_path);
        let n = chain.len() as u32;
        if self.params.pv_fork_cuts_min_chain_len <= n && n <= self.params.pv_fork_cuts_max_chain_len {
          self.pv_chain_infork_cut(chain);
          self.pv_chain_outfork_cut(chain);
        }
      }
      pv_routes.push((pv, pv_path));
    }
    Ok(pv_routes)
  }

  fn av_chain_cuts(&mut self, chain: &[Task]) {
    let n = chain.len() as u32;
    if (self.params.av_fork_cuts_min_chain_len..=self.params.av_fork_cuts_max_chain_len).contains(&n) {
      self.av_chain_infork_cut(&chain);
      self.av_chain_outfork_cut(&chain);
    }

    if (self.params.av_tournament_cuts_min_chain_len..=self.params.av_tournament_cuts_max_chain_len).contains(&n) {
      self.av_chain_tournament_cut(&chain);
    }
  }

  // fn endtime_cut(&mut self, finish_time: Time, av: Av, av_route: &[Task]) {
  //   let _s = trace_span!("endtime_cut", finish_time, av, ?av_route).entered();
  //   // first task is ODp, last is DDp
  //   let n = av_route.len() - 2;
  //   let mut lhs = finish_time * self.mp_vars.y[&(av, av_route[n], av_route[n + 1])];
  //
  //   for k in 1..n {
  //     let partial_finish_time = schedule::av_route_finish_time(self.data, &av_route[k + 1..]);
  //     if partial_finish_time != finish_time {
  //       lhs = lhs + (finish_time - partial_finish_time) * (self.mp_vars.y[&(av, av_route[k], av_route[k + 1])] - 1)
  //     }
  //   }
  //
  //   self.enqueue_cut(c!( lhs <= self.mp_vars.theta[&(av, av_route[n])] ), EndTime);
  // }

  fn sep_av_cuts(&mut self, ctx: &MIPSolCtx) -> Result<Vec<(Av, AvPath)>> {
    let task_pairs = get_task_pairs_by_av(ctx, &self.mp_vars.y)?;
    let mut av_routes = Vec::with_capacity(task_pairs.len());

    for (&av, av_task_pairs) in &task_pairs {
      let (av_paths, av_cycles) = construct_av_routes(av_task_pairs);
      if av_cycles.len() > 0 {
        info!(num_cycles = av_cycles.len(), "AV cycles found");
        trace!(?av_cycles);
        for cycle in av_cycles {
          if let Some(chain) = self.illegal_av_chain_in_cycle(&cycle) {
            self.av_chain_cuts(&chain);
          } else {
            self.av_cycle_cut(&cycle);
          }
        }
      }

      for av_path in av_paths {
        if !schedule::check_av_route(self.lu, &av_path) {
          let chain = self.shorten_illegal_av_chain(&av_path);
          self.av_chain_cuts(chain);
        } else if self.params.endtime_cuts {
          let finish_time = schedule::av_route_finish_time(self.lu, &av_path[1..]);
          let n = av_path.len() - 2;
          let theta_val = ctx.get_solution(std::iter::once(self.mp_vars.theta[&(av, av_path[n])]))?[0];

          // if (theta_val.round() as Time) < finish_time {
          //   self.endtime_cut(finish_time, av, &av_path);
          // } // FIXME compare?
        }
        av_routes.push((av, av_path));
      }
    }
    Ok(av_routes)
  }

  fn update_var_values(&mut self, ctx: &MIPSolCtx) -> Result<()> {
    #[cfg(debug_assertions)]
      {
        let vars = self.mp_vars.x.values()
          .chain(self.mp_vars.y.values())
          .chain(self.mp_vars.theta.values());

        for (&var, val) in vars.clone().zip(ctx.get_solution(vars)?) {
          self.var_vals.insert(var, val);
        }
      }

    Ok(())
  }
}


impl<'a> Callback for Cb<'a> {
  fn callback(&mut self, w: Where) -> CbResult {
    let cb_it = self.stats.inc_n_mipsol();
    let _span = info_span!("cb", cb_it).entered();

    match w {
      Where::MIPSol(ctx) => {
        debug!("integer MP solution found");

        #[cfg(debug_assertions)]
          self.update_var_values(&ctx)?;


        let initial_cut_cache_len = self.cut_cache.len();
        let pv_routes = self.sep_pv_cuts(&ctx)?;


        if self.cut_cache.len() > initial_cut_cache_len {
          info!(ncuts = self.cut_cache.len() - initial_cut_cache_len, "heuristic cuts added (PV only)");
        } else {
          let av_routes = self.sep_av_cuts(&ctx)?;

          if self.cut_cache.len() > initial_cut_cache_len {
            info!(ncuts = self.cut_cache.len() - initial_cut_cache_len, "heuristic cuts added");
          } else {
            let sp_idx = self.stats.inc_n_sp();
            let _span = error_span!("sp", sp_idx, estimate=tracing::field::Empty).entered();
            info!("no heuristic cuts found, solving LP subproblem");
            let sol = Solution{ objective: None, av_routes, pv_routes };
            if let Some(sol_log) = self.sol_log.as_mut() {
              serde_json::to_writer(&mut *sol_log, &sol.to_serialisable())?; // need to re-borrow here
              write!(sol_log, "\n")?;
            }

            let theta: Map<_, _> = get_var_values_mapped(&ctx, &self.mp_vars.theta, |t| t.round() as Time)?.collect();
            trace!(?theta);
            let estimate : Time = theta.values().sum();
            tracing::span::Span::current().record("estimate", &estimate);
            match self.params.sp {
              SpSolverKind::Dag => {
                dag::GraphModel::build(self.lu, &sol, &theta)?
                  .solve_subproblem_and_add_cuts(self, estimate)?;
              },
              SpSolverKind::Lp => {
                lp::TimingSubproblem::build(self.lu, &sol)?
                  .solve_subproblem_and_add_cuts(self, estimate)?;
              }
            }
          }
        }
        for (_, _, cut) in &self.cut_cache[initial_cut_cache_len..] {
          ctx.add_lazy(cut.clone())?;
        }
      }

      _ => {}
    }
    Ok(())
  }
}
