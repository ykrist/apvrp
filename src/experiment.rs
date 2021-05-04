use slurm_harray::*;
use std::time::Duration;
use serde::{Deserialize, Serialize};

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
  pub timelimit: u64,
  pub cpus: usize,
  pub av_fork_cuts_min_chain_len: usize,
  pub av_fork_cuts_max_chain_len: usize,
  pub av_tournament_cuts_min_chain_len: usize,
  pub av_tournament_cuts_max_chain_len: usize,
  pub pv_fork_cuts_min_chain_len: usize,
  pub pv_fork_cuts_max_chain_len: usize,
  pub pv_tournament_cuts_min_chain_len: usize,
  pub pv_tournament_cuts_max_chain_len: usize,
}

impl std::default::Default for Params {
  fn default() -> Self {
    Params {
      timelimit: 3600,
      cpus: 4,
      av_fork_cuts_min_chain_len: 0,
      av_fork_cuts_max_chain_len: 4,
      av_tournament_cuts_min_chain_len: 3,
      av_tournament_cuts_max_chain_len: 6,
      pv_fork_cuts_min_chain_len: 0,
      pv_fork_cuts_max_chain_len: 3,
      pv_tournament_cuts_min_chain_len: 0,
      pv_tournament_cuts_max_chain_len: usize::max_value(),
    }
  }
}

impl ExpParameters for Params {}

#[derive(Debug, Clone, Serialize)]
pub struct Outputs {
  pub trace_log: String,
}

impl ExpOutputs for Outputs {
  type Inputs = Inputs;
  type Params = Params;

  fn new(inputs: &Inputs, _params: &Params) -> Self {
    Outputs{
      trace_log: format!("{}-log.ndjson", inputs.index).into()
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
