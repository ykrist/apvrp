use slurm_harray::*;
use std::time::Duration;
use serde::{Deserialize, Serialize};
use anyhow::Result;
use grb::prelude::*;

#[derive(Debug, Clone, FromArgs, AddArgs, Serialize, Deserialize)]
#[slurm(inputs)]
pub struct Inputs {
  pub index: usize
}

impl ExpInputs for Inputs {
  fn id_str(&self) -> String {
    format!("{}", self.index)
  }
}


#[derive(Debug, Clone, FromArgs, AddArgs, Serialize, Deserialize)]
#[slurm(parameters)]
pub struct Params {
  #[slurm(valname="seconds")]
  pub timelimit: u64,
  pub cpus: u32,
  pub param_name: String,
  pub av_fork_cuts_min_chain_len: u32,
  pub av_fork_cuts_max_chain_len: u32,
  pub av_tournament_cuts_min_chain_len: u32,
  pub av_tournament_cuts_max_chain_len: u32,
  pub pv_fork_cuts_min_chain_len: u32,
  pub pv_fork_cuts_max_chain_len: u32,
  pub pv_tournament_cuts_min_chain_len: u32,
  pub pv_tournament_cuts_max_chain_len: u32,
  #[slurm(default="true")]
  pub endtime_cuts: bool,
  #[slurm(default="true")]
  pub soln_log: bool,
}

impl std::default::Default for Params {
  fn default() -> Self {
    Params {
      timelimit: 7200,
      cpus: 4,
      param_name: String::new(),
      av_fork_cuts_min_chain_len: 0,
      av_fork_cuts_max_chain_len: 4,
      av_tournament_cuts_min_chain_len: 3,
      av_tournament_cuts_max_chain_len: 6,
      pv_fork_cuts_min_chain_len: 0,
      pv_fork_cuts_max_chain_len: 3,
      pv_tournament_cuts_min_chain_len: 0,
      pv_tournament_cuts_max_chain_len: u32::MAX,
      endtime_cuts: true,
      soln_log: true,
    }
  }
}

impl ExpParameters for Params {
  fn id_str(&self) -> String {
    if self.param_name.len() > 0 {
      self.param_name.clone()
    } else {
      #[cfg(debug_assertions)] {
        String::from("debug")
      }
      #[cfg(not(debug_assertions))] {
        id_from_serialised(self)
      }
    }
  }
}

#[derive(Debug, Clone, Serialize)]
pub struct Outputs {
  pub trace_log: String,
  pub solution_log: String,
  pub info: String,
}

impl ExpOutputs for Outputs {
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

define_experiment!{ pub struct ApvrpExp, Inputs, Params, Outputs }

impl Experiment for ApvrpExp {
  fn log_root_dir() -> std::path::PathBuf {
    concat!(env!("CARGO_MANIFEST_DIR"), "/logs/").into()
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
    let fingerprint = unsafe{ std::mem::transmute(model.get_attr(attr::Fingerprint)?) }; // don't care about the value, only the bit-pattern
    Ok(GurobiInfo {
      fingerprint
    })
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Info {
  pub gurobi: GurobiInfo
}
