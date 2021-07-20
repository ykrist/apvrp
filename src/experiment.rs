use slurm_harray::*;
use std::time::Duration;
use serde::{Deserialize, Serialize};
use anyhow::Result;
use grb::prelude::*;
use std::str::FromStr;
use std::path::PathBuf;

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

#[derive(Debug, Clone, StructOpt, Serialize, Deserialize)]
pub struct Params {
  #[structopt(long, short, default_value="7200", value_name="seconds")]
  pub timelimit: u64,

  #[structopt(long)]
  #[cfg_attr(debug_assertions, structopt(default_value="1"))]
  #[cfg_attr(not(debug_assertions), structopt(default_value="4"))]
  pub cpus: u32,

  #[structopt(long)]
  pub param_name: Option<String>,

  /// Subproblem algorithm
  #[structopt(long, default_value="dag", possible_values=SpSolverKind::choices())]
  pub sp: SpSolverKind,

  #[structopt(long, default_value="0")]
  pub av_fork_cuts_min_chain_len: u32,

  #[structopt(long, default_value="4")]
  pub av_fork_cuts_max_chain_len: u32,

  #[structopt(long, default_value="3")]
  pub av_tournament_cuts_min_chain_len: u32,

  #[structopt(long, default_value="6")]
  pub av_tournament_cuts_max_chain_len: u32,

  #[structopt(long, default_value="0")]
  pub pv_fork_cuts_min_chain_len: u32,

  #[structopt(long, default_value="3")]
  pub pv_fork_cuts_max_chain_len: u32,

  #[structopt(long, default_value="0")]
  pub pv_tournament_cuts_min_chain_len: u32,

  #[structopt(long, default_value="100000000")]
  pub pv_tournament_cuts_max_chain_len: u32,

  #[structopt(long="no_endtime_cuts", parse(from_flag=std::ops::Not::not))]
  pub endtime_cuts: bool,

  #[structopt(long="no_soln_log", parse(from_flag=std::ops::Not::not))]
  pub soln_log: bool,
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

  fn new(inputs: &Inputs, _params: &Params) -> Self {
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
  pub outputs: Outputs,
}

impl Experiment for ApvrpExp {
  impl_experiment_helper! {
    ApvrpExp;
    inputs: Inputs;
    parameters: Params;
    outputs: Outputs;
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
