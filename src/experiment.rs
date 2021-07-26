use slurm_harray::*;
use std::time::Duration;
use serde::{Deserialize, Serialize};
use anyhow::{Context, Result};
use grb::prelude::*;
use std::str::FromStr;
use std::path::PathBuf;
use std::ops::RangeInclusive;

#[derive(Debug, Clone, StructOpt, Serialize, Deserialize)]
pub struct Inputs {
  pub index: usize
}

impl IdStr for Inputs {
  fn id_str(&self) -> String {
    format!("{}", self.index)
  }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum SpSolverKind {
  Dag,
  Lp
}

impl_arg_enum! { SpSolverKind;
  Dag = "dag",
  Lp = "lp",
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum CycleHandling {
  Cuts,
  Sp,
}

impl_arg_enum! { CycleHandling;
  Cuts = "cuts",
  Sp = "sp",
}

#[derive(Debug, Clone, StructOpt, Serialize, Deserialize)]
pub struct AuxParams {
  // #[structopt(long="no-soln-log", parse(from_flag=std::ops::Not::not))]
  /// Log incumbent solutions which generate a subproblem
  #[structopt(long)]
  pub soln_log: bool,
}

impl Default for AuxParams {
  fn default() -> Self {
    Self {
      soln_log: false,
    }
  }
}

fn cl_parse_range(s: &str) -> Result<RangeInclusive<u32>> {
  let mut tok = s.split(',');
  let ctx_msg = || format!("unable to parse string `{}`, expected `integer,integer`", s);
  let err = || anyhow::Error::msg(ctx_msg());

  let a : u32 = tok.next().ok_or_else(err)?.parse().with_context(ctx_msg)?;
  let b : u32 = tok.next().ok_or_else(err)?.parse().with_context(ctx_msg)?;
  if tok.next().is_some() {
    Err(err())
  } else {
    Ok(a..=b)
  }
}

#[derive(Debug, Clone, StructOpt, Serialize, Deserialize)]
pub struct Params {
  /// Give a time limit in second
  #[structopt(long, short, default_value="7200", value_name="seconds")]
  pub timelimit: u64,

  /// Number of threads to use
  #[structopt(long)]
  #[cfg_attr(debug_assertions, structopt(default_value="1"))]
  #[cfg_attr(not(debug_assertions), structopt(default_value="4"))]
  pub cpus: u32,

  /// Use the supplied string as a parameter name, instead of generating one from
  /// a parameter hash.
  #[structopt(long)]
  pub param_name: Option<String>,

  /// Subproblem algorithm
  #[structopt(long, default_value="dag", possible_values=SpSolverKind::choices())]
  pub sp: SpSolverKind,

  /// Minimum and maximum infeasible chain length at which to add Active Vehicle Fork cuts
  #[structopt(long, default_value="0,4", parse(try_from_str = cl_parse_range))]
  pub av_fork_cuts: std::ops::RangeInclusive<u32>,

  /// Minimum and maximum infeasible chain length at which to add Active Vehicle Tournament cuts
  #[structopt(long, default_value="3,6", parse(try_from_str = cl_parse_range))]
  pub av_tournament_cuts: std::ops::RangeInclusive<u32>,


  /// Minimum and maximum infeasible chain length at which to add Passive Vehicle Fork cuts
  #[structopt(long, default_value="0,3", parse(try_from_str = cl_parse_range))]
  pub pv_fork_cuts: std::ops::RangeInclusive<u32>,

  /// Minimum and maximum infeasible chain length at which to add Passive Vehicle Tournament cuts
  #[structopt(long, default_value="0,10000", parse(try_from_str = cl_parse_range))]
  pub pv_tournament_cuts: std::ops::RangeInclusive<u32>,

  /// Disable End-time cuts
  #[structopt(long="no-endtime-cuts", parse(from_flag=std::ops::Not::not))]
  pub endtime_cuts: bool,

  /// Try to separate cycle cuts without solving the subproblem
  #[structopt(long)]
  pub cycle_cuts: bool,

  /// Solve a preliminary MIP first, which discards the Active Vehicle Travel-Time component of the objective.
  #[structopt(long)]
  pub two_phase: bool,
}

impl IdStr for Params {
  fn id_str(&self) -> String {
    self.param_name.clone()
      .unwrap_or_else(|| {
        #[cfg(debug_assertions)] {
          String::from("debug")
        }
        #[cfg(not(debug_assertions))] {
          id_from_serialised(self)
        }
    })
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Outputs {
  pub trace_log: String,
  pub solution_log: String,
  pub info: String,
}

impl NewOutput for Outputs {
  type Inputs = Inputs;
  type Params = Params;
  type AuxParams = AuxParams;

  fn new(inputs: &Inputs, _params: &Params, _aux_params: &AuxParams) -> Self {
    Outputs{
      solution_log: format!("{}-sollog.ndjson", inputs.index).into(),
      trace_log: format!("{}-log.ndjson", inputs.index).into(),
      info: format!("{}-info.json", inputs.index).into(),
    }
  }
}

#[derive(Debug, Clone)]
pub struct ApvrpExp {
  pub inputs: Inputs,
  pub parameters: Params,
  pub aux_params: AuxParams,
  pub outputs: Outputs,
}

impl Experiment for ApvrpExp {
  impl_experiment_helper! {
    inputs: Inputs;
    parameters: Params;
    outputs: Outputs;
    aux_params: AuxParams;
  }

  fn log_root_dir() -> PathBuf {
    concat!(env!("CARGO_MANIFEST_DIR"), "/logs/").into()
  }

  fn post_parse(inputs: &Self::Inputs, params: &mut Self::Parameters) {
    if params.param_name.is_none() {
      params.param_name = Some(params.id_str())
    }
  }
}


impl ResourcePolicy for ApvrpExp {
  fn time(&self) -> Duration { Duration::from_secs(self.parameters.timelimit) }
  fn memory(&self) -> MemoryAmount { MemoryAmount::from_gb(4) }
  fn script(&self) -> String { String::from("#!/bin/bash\nconda activate or") }
  fn cpus(&self) -> usize { self.parameters.cpus as usize }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct GurobiInfo {
  pub fingerprint: u32,
}

impl GurobiInfo {
  pub fn new(model: &grb::Model) -> Result<Self> {
    // don't care about the value, only the bit-pattern
    let fingerprint = unsafe{ std::mem::transmute(model.get_attr(attr::Fingerprint)?) };
    Ok(GurobiInfo {
      fingerprint
    })
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Info {
  pub gurobi: GurobiInfo
}
