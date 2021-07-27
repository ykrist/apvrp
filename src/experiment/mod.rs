use slurm_harray::*;
use std::time::Duration;
use serde::{Deserialize, Serialize};
use anyhow::{Context, Result};
use grb::prelude::*;
use std::str::FromStr;
use std::path::PathBuf;
use std::ops::RangeInclusive;
use crate::{Map, Time, Cost};
use crate::model::cb::{CutType, CbStats};
use crate::model::mp::TaskModelMaster;

mod stopwatch;
pub use stopwatch::{Deadline, Stopwatch};

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
  /// Set MIN > MAX to disable.
  #[structopt(long, default_value="0,4", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
  pub av_fork_cuts: std::ops::RangeInclusive<u32>,

  /// Minimum and maximum infeasible chain length at which to add Active Vehicle Tournament cuts
  /// Set MIN > MAX to disable.
  #[structopt(long, default_value="3,6", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
  pub av_tournament_cuts: std::ops::RangeInclusive<u32>,

  /// Minimum and maximum infeasible chain length at which to add Passive Vehicle Fork cuts
  /// Set MIN > MAX to disable.
  #[structopt(long, default_value="0,3", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
  pub pv_fork_cuts: std::ops::RangeInclusive<u32>,

  /// Enable End-time optimality cuts
  #[structopt(long)]
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

  fn post_parse(_inputs: &Self::Inputs, params: &mut Self::Parameters) {
    if params.param_name.is_none() {
      params.param_name = Some(params.id_str())
    }
  }
}

mod instance_groups {
  use std::ops::Range;
  pub const A_TW25 : Range<usize> = 0..20;
  pub const A_TW50 : Range<usize> = 20..40;
  pub const A_TW100 : Range<usize> = 40..60;
  pub const A_TW200 : Range<usize> = 60..80;

  pub const B_TW25 : Range<usize> = 80..100;
  pub const B_TW50 : Range<usize> = 100..120;
  pub const B_TW100 : Range<usize> = 120..140;
  pub const B_TW200 : Range<usize> = 140..160;

  pub const MK : Range<usize> = 160..190;

  // const


}

impl ResourcePolicy for ApvrpExp {
  fn time(&self) -> Duration {
    use instance_groups::*;
    let i = self.inputs.index;

    if A_TW25.contains(&i)
      || A_TW50.contains(&i)
      || A_TW100.contains(&i) {
      return Duration::from_secs(300)
    }

    Duration::from_secs(self.parameters.timelimit + 300)
  }

  fn memory(&self) -> MemoryAmount {
    use instance_groups::*;
    let i = self.inputs.index;

    if A_TW25.contains(&i)
      || A_TW50.contains(&i)
      || A_TW100.contains(&i)
      || A_TW200.contains(&i) {
      return MemoryAmount::from_gb(4)
    }

    MemoryAmount::from_gb(32)
  }

  fn script(&self) -> String {
    include_str!("slurm_job_template.sh").to_string()
  }

  fn cpus(&self) -> usize {
    self.parameters.cpus as usize
  }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct GurobiInfo {
  pub num_vars: u32,
  pub num_cons: u32,
  pub fingerprint: u32,
  pub node_count: Option<u32>,
  pub sol_count: u32,
  pub status: i32,
  pub runtime: f64,
}

impl GurobiInfo {
  pub fn from_model(model: &grb::Model) -> Result<Self> {
    // don't care about the value, only the bit-pattern
    let fingerprint = model.get_attr(attr::Fingerprint)?;
    let fingerprint = unsafe{ std::mem::transmute(fingerprint) };
    let num_vars = model.get_attr(attr::NumVars)? as u32;
    let num_cons = model.get_attr(attr::NumConstrs)? as u32;
    let status = model.status()? as i32;
    let sol_count = model.get_attr(attr::SolCount)? as u32;
    let runtime = model.get_attr(attr::Runtime)?;
    let node_count = model.get_attr(attr::NodeCount).ok().map(|n| n as u32);

    Ok(GurobiInfo {
      fingerprint,
      num_cons,
      num_vars,
      status,
      sol_count,
      runtime,
      node_count,
    })
  }
}

pub enum BoundsRecorder {
  Init,
  PostLp(Cost),
  PostMip(Bounds),
}

impl BoundsRecorder {
  pub fn record_root_lb(&mut self, model: &TaskModelMaster) -> Result<()> {
    use BoundsRecorder::*;
    match self {
      s @ Init => {
        *s = PostLp(model.obj_val()?);
      }
      PostLp(..) | PostMip(..) => panic!("root LB already recorded"),
    }
    Ok(())
  }

  pub fn record_final_bounds(&mut self, model: &TaskModelMaster) -> Result<()> {
    use BoundsRecorder::*;
    match self {
      Init => panic!("record root LB first"),
      PostMip(..) => panic!("already recorded!"),
      PostLp(root_lb)=> {
        let bounds = Bounds {
          lower: model.obj_bound()?,
          upper: model.obj_val()?,
          lower_root: *root_lb,
          upper_full_obj: None,
        };
        *self = PostMip(bounds);
      }
    }
    Ok(())
  }

  pub fn record_ub_full_obj(&mut self, ub: Cost) {
    use BoundsRecorder::*;
    match self {
      Init | PostLp(..) => panic!("record final UB and LB first"),
      PostMip(Bounds{ upper_full_obj, .. })=> {
        if upper_full_obj.is_some() {
          panic!("already recorded!")
        }
        *upper_full_obj = Some(ub);
      }
    }
  }

  pub fn finish(self) -> Bounds {
    use BoundsRecorder::*;
    match self {
      Init | PostLp(..) => panic!("record final UB and LB first"),
      PostMip(bounds) => bounds
    }
  }
}

#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct Bounds {
  pub lower_root: Cost,
  pub lower: Cost,
  pub upper: Cost,
  pub upper_full_obj: Option<Cost>,
}

impl Bounds {
  pub fn new() -> BoundsRecorder { BoundsRecorder::Init }
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhaseInfo {
  phase: String,
  gurobi: GurobiInfo,
  bounds: Option<Bounds>,
  cb: Option<CbStats>,
}

impl PhaseInfo {
  pub fn new_init(model: &grb::Model) -> Result<Self> {
    Ok(PhaseInfo {
      phase: "init".to_string(),
      gurobi: GurobiInfo::from_model(model)?,
      bounds: None,
      cb: None,
    })
  }

  pub fn new_post_mip(name: String,
                      model: &grb::Model,
                      bounds: Bounds,
                      cb_stats: CbStats,
  ) -> Result<Self> {
    Ok(PhaseInfo {
      phase: name,
      gurobi: GurobiInfo::from_model(model).context("failed to gather Gurobi info")?,
      bounds: Some(bounds),
      cb: Some(cb_stats)
    })
  }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Info {
  pub info: Vec<PhaseInfo>,
  pub time: Vec<(String, u128)>,
}

