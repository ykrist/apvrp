use crate::*;
use crate::tasks::{chain::ChainIterExt, TaskId};
use crate::solution::*;
use grb::prelude::*;
use grb::callback::{Callback, Where, CbResult, MIPSolCtx};
use fnv::FnvHashSet;
use super::sp::{BendersCut, TimingSubproblem};
use tracing::{info, info_span, debug, trace, error_span, warn, error};
use std::fmt;
use grb::constr::IneqExpr;
use variant_count::VariantCount;
use anyhow::Context;
use std::hash::Hash;
use std::env::var;
use std::collections::{HashSet, HashMap};
use experiment::Params;
use slurm_harray::ExperimentAuto;

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


#[derive(VariantCount, Copy, Clone, Debug)]
#[repr(usize)]
pub enum CutType {
  LpOpt = 0,
  LpFeas,
  AvCycle,
  AvChainTournament,
  AvChainInfork,
  AvChainOutfork,
  PvCycle,
  PvChainInfork,
  PvChainOutfork,
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

pub struct Cb<'a> {
  data: &'a Data,
  sets: &'a Sets,
  tasks: &'a Tasks,
  var_names: Map<Var, String>,
  mp_vars: super::mp::MpVars,
  // needs to be owned to avoid mutability issues with the Model object.
  pub stats: CbStats,
  cut_cache: Vec<(String, IneqExpr)>,
  av_cycles: HashMap<Vec<Task>, usize>,
  sp_env: Env,
  params: &'a Params,
  #[cfg(debug_assertions)]
  var_vals: Map<Var, f64>,
}

impl<'a> Cb<'a> {
  pub fn sp_env(&self) -> &Env { &self.sp_env }

  pub fn new(data: &'a Data, exp: &'a experiment::ApvrpExp, sets: &'a Sets, tasks: &'a Tasks, mp: &super::mp::TaskModelMaster) -> Result<Self> {
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
      params: &exp.parameters,
      #[cfg(debug_assertions)]
      var_vals: Map::default(),
    })
  }

  #[tracing::instrument(level = "trace", skip(self, cut))]
  pub fn enqueue_cut(&mut self, cut: IneqExpr, ty: CutType) {
    let i = self.stats.inc_cut_count(ty, 1);
    let name = format!("{:?}[{}]", ty, i);

    #[cfg(debug_assertions)]
    {
      let (lhs, rhs) = cut.evaluate(&self.var_vals);
      if lhs <= rhs + 1e-6  {
        error!(?lhs, ?rhs, cut=?cut.with_names(&self.var_names), "cut is not violated!");
        panic!("found a bug!"); // FIXME INDEX 1 crashes here quickly
      }
      trace!(?lhs, ?rhs, cut=?cut.with_names(&self.var_names));
    }

    self.cut_cache.push((name, cut));
  }

  pub fn flush_cut_cache(&mut self, mp: &mut Model) -> Result<()> {
    for (name, cut) in self.cut_cache.drain(0..) {
      mp.add_constr(&name, cut)?;
    }
    Ok(())
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_cycle_cut(&mut self, cycle: &AvPath) {
    #[cfg(debug_assertions)] {
      let mut sorted_cycle = cycle.clone();
      sorted_cycle.sort_by_key(|t| t.id());
      let count = self.av_cycles.entry(sorted_cycle).or_insert(0);
      if *count > 0 { warn!(?count, "cycle has been encountered before") }
      *count += 1;
    }

    let cycle_tasks = &cycle[..cycle.len()-1];

    let ysum = cycle_tasks.iter()
      .cartesian_product(cycle_tasks.iter())
      .flat_map(|(&t1, &t2)|
        self.mp_vars.ysum_similar_tasks(self.sets, self.tasks, t1, t2)
      )
      .grb_sum();

    // cycle is a list of n+1 nodes (start = end), which form n arcs
    self.enqueue_cut(c!(ysum <= cycle.len() - 2), CutType::AvCycle);
    // panic!()
  }

  #[inline]
  fn av_chain_fork_lhs(&self, chain: &[Task]) -> Expr {
    chain.iter()
      .tuple_windows()
      .flat_map(|(&t1, &t2)|
        self.mp_vars.ysum_similar_tasks(self.sets, self.tasks, t1, t2)
      )
      .grb_sum()
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_chain_infork_cut(&mut self, chain: &[Task]) {
    let chain = &chain[1..];
    let t2 = *chain.first().expect("chain should have at least three tasks");
    let n = chain.len() - 1; // number of arcs in legal chain

    debug_assert!(schedule::check_av_route(self.data, chain));

    let rhs_sum = chain.av_legal_before(self.data, self.tasks)
      .flat_map(|t1| {
        let y = &self.mp_vars.y;
        self.sets.avs().filter_map(move |av| y.get(&(av, t1, t2)).copied())
      })
      .grb_sum();

    let lhs = self.av_chain_fork_lhs(chain);
    self.enqueue_cut(c!(lhs <= n - 1 + rhs_sum), CutType::AvChainInfork);
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_chain_outfork_cut(&mut self, chain: &[Task]) {
    let chain = &chain[..chain.len()-1];
    let t1 = *chain.last().expect("chain should have at least three tasks");
    let n = chain.len() - 1;  // number of arcs in legal chain


    debug_assert!(schedule::check_av_route(self.data, chain));

    let rhs_sum = chain.av_legal_after(self.data, self.tasks)
      .flat_map(|t2| {
        let y = &self.mp_vars.y;
        self.sets.avs().filter_map(move |av| y.get(&(av, t1, t2)).copied())
      })
      .grb_sum();

    let lhs = self.av_chain_fork_lhs(chain);
    self.enqueue_cut(c!(lhs <= n - 1 + rhs_sum), CutType::AvChainOutfork);
  }

  #[tracing::instrument(level = "trace", skip(self))]
  fn av_chain_tournament_cut(&mut self, chain: &[Task]) {
    // let n = chain.len()-1;
    // /// Are all permutations illegal?
    // let mut all_illegal = true;
    // /// Are all permutations where the first Task in the chain is held fixed illegal?
    // let mut all_illegal_first_fixed = true;
    // /// Are all permutations where the last Task in the chain is held fixed illegal?
    // let mut all_illegal_last_fixed = true;
    //
    // for c in chain.permutations() {
    //   let c = c.as_slice();
    //   if schedule::check_av_route(self.data, c) {
    //     all_illegal = false;
    //
    //     if c[0] == chain[0] {
    //       all_illegal_first_fixed = false;
    //     }
    //
    //     if c[n] == chain[n] {
    //       all_illegal_last_fixed = false
    //     }
    //   }
    // }
    //
    // if all_illegal {
    //   // add cut
    // } else if all_illegal_last_fixed | all_illegal_first_fixed {
    //   if all_illegal_first_fixed {
    //     // add cut
    //   }
    //   if all_illegal_last_fixed {
    //     // add cut
    //   }
    // } else {
    //   // basic forward tornamenting
    // }

    // basic forward tornamenting

    let lhs = chain.iter().enumerate()
      .flat_map(|(k, &t1)| chain[(k + 1)..].iter().map(move |&t2| (t1, t2)))
      .flat_map(|(t1, t2)| self.mp_vars.ysum_similar_tasks(self.sets, self.tasks, t1, t2))
      .grb_sum();

    self.enqueue_cut(c!(lhs <= chain.len() - 2), CutType::AvChainTournament)
  }

  /// Given `chain`, which violates the AV timing constraints, find the shortest subchain which still violates
  /// the timing constraints.
  fn shorten_illegal_av_chain<'b>(&self, chain: &'b [Task]) -> &'b [Task] {
    for l in 3..=chain.len() {
      for start_pos in 0..(chain.len() - l) {
        let c = &chain[start_pos..(start_pos + l)];
        if !schedule::check_av_route(self.data, c) {
          return c;
        }
      }
    }
    unreachable!("chain must be illegal")
  }

  /// Given `chain`, which violates the PV timing constraints, find the shortest subchain which still violates
  /// the timing constraints.
  fn shorten_illegal_pv_chain<'b>(&self, chain: &'b [Task]) -> &'b [Task] {
    for l in 3..=chain.len() {
      for start_pos in 0..(chain.len() - l) {
        let c = &chain[start_pos..(start_pos + l)];
        if !schedule::check_pv_route(self.data, c) {
          return c;
        }
      }
    }
    unreachable!("chain must be illegal")
  }

  /// Given a `cycle` where `cycle.first() == cycle.last()`, returns the shortest acyclic subchain
  /// that violates the AV timing constraints, if one exists.
  fn illegal_av_chain_in_cycle(&self, cycle: &[Task]) -> Option<Vec<Task>> {
    // If cycle is
    // [0, 1, 2, 3, 4, 0] // .len() = 6
    // then cycle rep holds
    // [0, 1, 2, 3, 4, 0, 1, 2, 3]
    // which allows us to sweep all the cyclic permutations with slices
    let cycle_rep = {
      let mut v = Vec::with_capacity(2*cycle.len() - 3);
      v.extend_from_slice(cycle);
      v.extend_from_slice(&cycle[1..cycle.len() - 2]);
      v
    };

    for l in 3..=(cycle.len() - 1) {
      for start_pos in 0..(cycle.len() - 1) {
        let c = &cycle_rep[start_pos..(start_pos + l)];
        if !schedule::check_av_route(self.data, c) {
          return Some(c.to_vec());
        }
      }
    }

    None
  }

  #[tracing::instrument(level="trace", skip(self))]
  fn pv_cycle_cut(&mut self, cycle: &PvPath) {
    let xsum = cycle.iter()
      .flat_map(|t|
        self.tasks.by_locs[&(t.start, t.end)].iter()
          .map(|t| self.mp_vars.x[t]))
      .grb_sum();
    self.enqueue_cut(c!(xsum <= cycle.len() - 2), CutType::PvCycle); // cycle is a list of n+1 nodes (start = end), which form n arcs
  }

  // #[allow(unused_variables)]
  // fn pv_chain_tournament_cut(&mut self, chain: &[Task]) {
  //   todo!()
  // }
  //
  #[allow(unused_variables)]
  #[tracing::instrument(level="trace", skip(self))]
  fn pv_chain_infork_cut(&mut self, chain: &[Task]) {
    let chain = &chain[1..];
    let rhs_sum = chain.pv_legal_before(self.data, self.tasks)
      .map(|t| self.mp_vars.x[&t])
      .grb_sum();

    let lhs_sum = chain.iter()
      .map(|t| self.mp_vars.x[t])
      .grb_sum();

    self.enqueue_cut(c!(lhs_sum <= chain.len() - 1 + rhs_sum), CutType::PvChainInfork);
  }

  #[allow(unused_variables)]
  #[tracing::instrument(level="trace", skip(self))]
  fn pv_chain_outfork_cut(&mut self, chain: &[Task]) {
    let chain = &chain[..chain.len()-1];
    let rhs_sum = chain.pv_legal_after(self.data, self.tasks)
      .map(|t| self.mp_vars.x[&t])
      .grb_sum();

    let lhs_sum = chain.iter()
      .map(|t| self.mp_vars.x[t])
      .grb_sum();

    self.enqueue_cut(c!(lhs_sum <= chain.len() - 1 + rhs_sum), CutType::PvChainOutfork);
  }


  fn sep_pv_cuts(&mut self, ctx: &MIPSolCtx) -> Result<Vec<(Pv, PvPath)>>  {
    let pv_tasks = get_tasks_by_pv(ctx, &self.mp_vars.x)?;
    let mut pv_routes = Vec::with_capacity(pv_tasks.len());
    for (&pv, tasks) in &pv_tasks {
      let (pv_path, pv_cycles) = construct_pv_route(tasks);

      for cycle in pv_cycles {
        self.pv_cycle_cut(&cycle);
      }

      if !schedule::check_pv_route(self.data, &pv_path) {
        let chain = self.shorten_illegal_pv_chain(&pv_path);
        if chain.len() >= self.params.pv_fork_cuts_min_chain_len
          && chain.len() <= self.params.pv_fork_cuts_max_chain_len {
          self.pv_chain_infork_cut(chain);
          self.pv_chain_outfork_cut(chain);
        }
      }
      pv_routes.push((pv, pv_path));
    }
    Ok(pv_routes)
  }

  fn av_chain_cuts(&mut self, chain: &[Task]) {
    if self.params.av_fork_cuts_min_chain_len <= chain.len() &&
      chain.len() <= self.params.av_fork_cuts_max_chain_len {
      self.av_chain_infork_cut(&chain);
      self.av_chain_outfork_cut(&chain);
    }

    if self.params.av_tournament_cuts_min_chain_len <= chain.len()
      && chain.len() <= self.params.av_tournament_cuts_max_chain_len {
      self.av_chain_tournament_cut(&chain);
    }
  }

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
        if !schedule::check_av_route(self.data, &av_path) {
          let chain = self.shorten_illegal_av_chain(&av_path);
          self.av_chain_cuts(chain);
        }
        av_routes.push((av, av_path));
      }
    }
    Ok(av_routes)
  }

  fn update_var_values(&mut self,  ctx: &MIPSolCtx) -> Result<()> {
    #[cfg(debug_assertions)]
      {
        let vars = self.mp_vars.x.values()
          .chain(self.mp_vars.y.values());

        for (&var, val) in vars.clone().zip(ctx.get_solution(vars)?) {
          self.var_vals.insert(var, val);
        }
      }

    Ok(())
  }

}





impl<'a> Callback for Cb<'a> {

  fn callback(&mut self, w: Where) -> CbResult {
    let _span = info_span!("cb").entered();

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
            let _span = error_span!("lp_sp").entered();
            info!("no heuristic cuts found, solving LP subproblem");
            let sp = TimingSubproblem::build(&self.sp_env, self.data, self.tasks, &av_routes, &pv_routes)?;

            let theta: Map<_, _> = get_var_values(&ctx, &self.mp_vars.theta)?.collect();
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
          }
        }

        for (_, cut) in &self.cut_cache[initial_cut_cache_len..] {
          trace!("add cut {:?}", cut.with_names(&self.var_names));
          ctx.add_lazy(cut.clone())?;
        }

      }

      _ => {}
    }
    Ok(())
  }
}
