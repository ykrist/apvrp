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
  /// Log incumbent solutions which generate a subproblem
  #[structopt(long)]
  pub soln_log: bool,

  /// Disable trace logging to STDOUT.
  #[structopt(long, short="q")]
  pub quiet: bool,

  /// Save the final model of the master problem (with all lazy constraints included) to
  /// an LP file.
  #[structopt(long)]
  pub model_file: bool
}

impl Default for AuxParams {
  fn default() -> Self {
    Self {
      soln_log: false,
      quiet: false,
      model_file: false,
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

  /// Minimum and maximum infeasible chain length at which to add Active Vehicle Fork cuts.
  /// Set MIN > MAX to disable.
  #[structopt(long, default_value="1,0", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
  pub av_fork_cuts: std::ops::RangeInclusive<u32>,

  /// Minimum and maximum infeasible chain length at which to add Active Vehicle Tournament cuts.
  /// Set MIN > MAX to disable.
  #[structopt(long, default_value="1,0", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
  pub av_tournament_cuts: std::ops::RangeInclusive<u32>,

  /// Minimum and maximum infeasible chain length at which to add Passive Vehicle Fork cuts.
  /// Set MIN > MAX to disable.
  #[structopt(long, default_value="1,0", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
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

  #[structopt(long, parse(try_from_str=parse_gurobi_param), require_delimiter=true)]
  pub gurobi: Vec<(String, GurobiParamVal)>,

  /// Enable Passive Vehicle Column Generation: generate all Passive Vehicle routes a-priori and use route-based variables
  /// to eliminate the need for Passive Vehicle feasibility cuts.
  #[structopt(long)]
  pub pvcg: bool
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum GurobiParamVal {
  Int(i32),
  Dbl(f64),
}

fn parse_gurobi_param(s: &str) -> Result<(String, GurobiParamVal)> {
  let error_msg = "format must be ATTR_NAME=VALUE";
  let mut kv = s.split('=');
  let key = kv.next().ok_or_else(|| anyhow::Error::msg(error_msg))?;
  let val = kv.next().ok_or_else(|| anyhow::Error::msg(error_msg))?;

  if kv.next().is_some() {
    anyhow::bail!("{}", error_msg);
  }

  if let Ok(val) = val.parse::<i32>() {
    return Ok((key.to_string(), GurobiParamVal::Int(val)))
  }

  if let Ok(val) = val.parse::<f64>() {
    return Ok((key.to_string(), GurobiParamVal::Dbl(val)))
  }

  anyhow::bail!("value must be integer or double (string attr not supported)")
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
  profile: SlurmProfile,
  pub inputs: Inputs,
  pub parameters: Params,
  pub aux_params: AuxParams,
  pub outputs: Outputs,
}

#[inline(always)]
fn scale_time(t: u64, s: f64) -> u64 {
  (t as f64 * s).round() as u64
}

impl Experiment for ApvrpExp {
  impl_experiment_helper! {
    profile;
    inputs: Inputs;
    parameters: Params;
    outputs: Outputs;
    aux_params: AuxParams;
  }

  fn log_root_dir() -> PathBuf {
    concat!(env!("CARGO_MANIFEST_DIR"), "/logs/").into()
  }

  fn post_parse(prof: SlurmProfile, _inputs: &Self::Inputs, params: &mut Self::Parameters, aux_params: &mut Self::AuxParameters) {
    params.gurobi.sort_by_cached_key(|(s, _)| s.clone());
    if params.param_name.is_none() {
      params.param_name = Some(params.id_str())
    }
    let s = params.param_name.as_mut().unwrap();
    match prof {
      SlurmProfile::Test => {
        s.push_str("-test");
        params.timelimit = scale_time(params.timelimit, 1.5);
      },
      SlurmProfile::Trace => {
        s.push_str("-trace");
        aux_params.soln_log = true;
        params.timelimit = scale_time(params.timelimit, 2.5);
      },
      _ => {},
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
}

impl ResourcePolicy for ApvrpExp {
  fn time(&self) -> Duration {
    use instance_groups::*;
    let i = self.inputs.index;
    let timelimit = if A_TW25.contains(&i)
      || A_TW50.contains(&i)
      || A_TW100.contains(&i) {
      Duration::from_secs(300)
    } else {
      Duration::from_secs(self.parameters.timelimit + 300)
    };
    timelimit
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

    MemoryAmount::from_gb(8)
  }

  fn script(&self) -> String {
    let script = match self.profile {
      SlurmProfile::Test | SlurmProfile::Default => include_str!("slurm_job.sh"),
      SlurmProfile::Trace => include_str!("slurm_job_trace.sh"),
    };
    script.to_string()
  }

  fn cpus(&self) -> usize {
    self.parameters.cpus as usize
  }

  fn job_name(&self) -> Option<String> {
    use std::fmt::Write;
    let mut s = self.parameters.param_name.clone();
    if let Some(s) = s.as_mut() {
       if self.profile == SlurmProfile::Trace {
         write!(s, "-{}", self.inputs.index).unwrap();
       }
    }
    s
  }

  fn constraint(&self) -> Option<String> {
    match self.profile {
      SlurmProfile::Default =>Some("R640".to_string()),
      _ => None,
    }

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


#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub struct Bounds {
  // FIXME lower bounds should be floats
  pub lower_root: Cost,
  pub lower: Cost,
  pub upper: Cost,
  pub upper_full_obj: Option<Cost>,
}

impl Bounds {
  pub fn new() -> BoundsRecorder { BoundsRecorder::Init }
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


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhaseInfo {
  pub phase: String,
  pub gurobi: GurobiInfo,
  pub bounds: Option<Bounds>,
  pub cb: Option<CbStats>,
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
  pub commit_hash: String,
}

impl Info {
  pub fn new(info: Vec<PhaseInfo>, time: Vec<(String, u128)>) -> Self {
    Info {
      info,
      time,
      commit_hash: crate::COMMIT_HASH.to_string(),
    }
  }
}

