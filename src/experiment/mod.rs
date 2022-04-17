use crate::model::cb::{CbStats, CutType};
use crate::model::mp::TaskModelMaster;
use crate::{Cost, Map, Time};
use anyhow::{Context, Result};
use grb::prelude::*;
use labrat::*;
use serde::{Deserialize, Serialize};
use std::ops::RangeInclusive;
use std::path::PathBuf;
use std::str::FromStr;
use std::time::Duration;

mod stopwatch;
pub use stopwatch::{Deadline, Stopwatch};

#[derive(Debug, Clone, Args, Serialize, Deserialize)]
pub struct Inputs {
  pub index: usize,
}

impl IdStr for Inputs {
  fn id_str(&self) -> String {
    format!("{}", self.index)
  }
}

#[derive(Debug, Copy, Clone, ArgEnum, Serialize, Deserialize)]
pub enum SpSolverKind {
  Dag,
}


#[derive(Debug, Copy, Clone, Serialize, Deserialize)]
pub enum CycleHandling {
  Cuts,
  Sp,
}

mod resource_limits {
  use std::collections::HashMap;
  use std::path::Path;
  use std::sync::RwLock;
  use lazy_static::lazy_static;
  use super::*;
  pub type Profile = Map<usize, f64>;

  struct Cache(RwLock<HashMap<String, Profile>>);

  impl Cache {
    fn new() -> Self {
      Cache(RwLock::new(HashMap::new()))
    }

    fn get<'a>(&'a self, key: &str) -> Option<Profile> {
      self.0.read().unwrap().get(key).cloned()
    }

    fn insert<'a>(&'a self, key: String, value: Map<usize, f64>) {
      self.0.write().unwrap().insert(key.to_string(), value);
    }
  }

  lazy_static! {
    static ref MEMORY_PROFILES: Cache = Cache::new();
    static ref TIME_PROFILES: Cache = Cache::new();
  }

  fn choices(dir: impl AsRef<Path>) -> String {
    let mut s = String::new();
    for f in std::fs::read_dir(dir).unwrap() {
      let p = f.unwrap().path();
      s.push_str(p.file_stem().unwrap().to_str().unwrap());
      s.push(' ');
    }
    s
  }

  fn find_profile(profile: &str, dir: &str, cache: &Cache) -> Result<Profile> {
    if let Some(m) = cache.get(profile) {
      return Ok(m);
    }

    let dir = Path::new(dir);
    let mut path = dir.join(profile);
    path.set_extension("json");

    if !path.exists() {
      anyhow::bail!(
        "profile `{}` doesn't exist (valid choices: {})",
        profile,
        choices(dir)
      )
    }

    let m: Profile = crate::utils::read_json(path)?;
    cache.insert(profile.to_string(), m.clone());
    Ok(m)
  }

  #[inline(always)]
  pub fn time(profile: &str) -> Result<Profile> {
    find_profile(
      profile,
      concat!(env!("CARGO_MANIFEST_DIR"), "/data/slurm/time"),
      &TIME_PROFILES,
    )
  }

  #[inline(always)]
  pub fn memory(profile: &str) -> Result<Profile> {
    find_profile(
      profile,
      concat!(env!("CARGO_MANIFEST_DIR"), "/data/slurm/memory"),
      &MEMORY_PROFILES,
    )
  }
}

#[derive(Debug, Clone, Args, Serialize, Deserialize)]
pub struct AuxParams {
  /// Log incumbent solutions which generate a subproblem
  #[clap(long)]
  pub soln_log: bool,

  /// Disable trace logging to STDOUT.
  #[clap(long, short = 'q')]
  pub quiet: bool,

  /// Save the final model of the master problem (with all lazy constraints included) to
  /// an LP file.
  #[clap(long)]
  pub model_file: bool,

  /// Slurm memory profile (filenames in `data/slurm/memory` without `.json` extension)) to use
  #[clap(long="slurm-mem", default_value="default", value_name="PROFILE", parse(try_from_str=resource_limits::memory))]
  pub slurm_memory: resource_limits::Profile,

  /// Slurm time limit profile (filenames in `data/slurm/time` without `.json` extension)) to use
  #[clap(long="slurm-time", default_value="default", value_name="PROFILE", parse(try_from_str=resource_limits::time))]
  pub slurm_time: resource_limits::Profile,
}

impl Default for AuxParams {
  fn default() -> Self {
    Self {
      soln_log: false,
      quiet: false,
      model_file: false,
      slurm_memory: resource_limits::memory("default").unwrap(),
      slurm_time: resource_limits::time("default").unwrap(),
    }
  }
}

fn cl_parse_range(s: &str) -> Result<RangeInclusive<u32>> {
  let mut tok = s.split(',');
  let ctx_msg = || format!("unable to parse string `{}`, expected `integer,integer`", s);
  let err = || anyhow::Error::msg(ctx_msg());

  let a: u32 = tok.next().ok_or_else(err)?.parse().with_context(ctx_msg)?;
  let b: u32 = tok.next().ok_or_else(err)?.parse().with_context(ctx_msg)?;
  if tok.next().is_some() {
    Err(err())
  } else {
    Ok(a..=b)
  }
}

#[derive(Debug, Clone, Args, Serialize, Deserialize)]
pub struct Params {
  /// Give a time limit in second
  #[clap(long, short, default_value = "7200", value_name = "seconds")]
  pub timelimit: u64,

  /// Number of threads to use
  #[clap(long, default_value = "1")]
  pub cpus: u32,

  /// Use the supplied string as a parameter name, instead of generating one from
  /// a parameter hash.
  #[clap(long)]
  pub param_name: Option<String>,

  /// Subproblem algorithm
  #[clap(long, default_value = "dag", arg_enum)]
  pub sp: SpSolverKind,

  /// Minimum and maximum infeasible chain length at which to add Active Vehicle Fork cuts.
  /// Set MIN > MAX to disable.
  #[clap(long, default_value="1,0", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
  pub av_fork_cuts: std::ops::RangeInclusive<u32>,

  /// Minimum and maximum infeasible chain length at which to add Active Vehicle Tournament cuts.
  /// Set MIN > MAX to disable.
  #[clap(long, default_value="1,0", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
  pub av_tournament_cuts: std::ops::RangeInclusive<u32>,

  /// Minimum and maximum infeasible chain length at which to add Passive Vehicle Fork cuts.
  /// Set MIN > MAX to disable.
  #[clap(long, default_value="1,0", value_name="MIN,MAX", parse(try_from_str = cl_parse_range))]
  pub pv_fork_cuts: std::ops::RangeInclusive<u32>,

  /// Enable End-time optimality cuts
  #[clap(long)]
  pub endtime_cuts: bool,

  /// Try to separate cycle cuts without solving the subproblem
  #[clap(long)]
  pub cycle_cuts: bool,

  /// Solve a preliminary MIP first, which discards the Active Vehicle Travel-Time component of the objective.
  #[clap(long)]
  pub two_phase: bool,

  /// Pass a comma-separated list of gurobi parameters to set, eg: "PARAM1=VALUE1,PARAM2=VALUE2".
  /// Only integer and float-valued parameters are supported.
  #[clap(
    long,
    parse(try_from_str=parse_gurobi_param),
    use_value_delimiter(true),
    require_value_delimiter(true)
  )]
  pub gurobi: Vec<(String, GurobiParamVal)>,

  /// Enable Passive Vehicle Column Generation: generate all Passive Vehicle routes a-priori and use route-based variables
  /// to eliminate the need for Passive Vehicle feasibility cuts.
  #[clap(long)]
  pub pvcg: bool,
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
    return Ok((key.to_string(), GurobiParamVal::Int(val)));
  }

  if let Ok(val) = val.parse::<f64>() {
    return Ok((key.to_string(), GurobiParamVal::Dbl(val)));
  }

  anyhow::bail!("value must be integer or double (string attr not supported)")
}

impl IdStr for Params {
  fn id_str(&self) -> String {
    self.param_name.clone().unwrap_or_else(|| {
      #[cfg(debug_assertions)]
      {
        String::from("debug")
      }
      #[cfg(not(debug_assertions))]
      {
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

#[derive(Debug, Clone)]
pub struct ApvrpExp {
  profile: Profile,
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
  type Input = Inputs;
  type Parameters = Params;
  type Config = AuxParams;
  type Output = Outputs;

  fn input(&self) -> &Self::Input {
    &self.inputs
  }

  fn output(&self) -> &Self::Output {
    &self.outputs
  }

  fn parameter(&self) -> &Self::Parameters {
    &self.parameters
  }

  fn new(
    profile: Profile,
    config: Self::Config,
    inputs: Self::Input,
    parameters: Self::Parameters,
    outputs: Self::Output,
  ) -> Self {
    ApvrpExp {
      profile,
      inputs,
      parameters,
      outputs,
      aux_params: config,
    }
  }

  fn new_output(
    inputs: &Self::Input,
    _params: &Self::Parameters,
    _config: &Self::Config,
  ) -> Self::Output {
    Outputs {
      solution_log: format!("{}-sollog.ndjson", inputs.index).into(),
      trace_log: format!("{}-log.msgpack", inputs.index).into(),
      info: format!("{}-info.json", inputs.index).into(),
    }
  }

  fn root_dir() -> PathBuf {
    concat!(env!("CARGO_MANIFEST_DIR"), "/logs/").into()
  }

  fn post_parse(
    prof: Profile,
    _inputs: &Self::Input,
    params: &mut Self::Parameters,
    aux_params: &mut Self::Config,
  ) {
    params.gurobi.sort_by_cached_key(|(s, _)| s.clone());
    if params.param_name.is_none() {
      params.param_name = Some(params.id_str())
    }
    let s = params.param_name.as_mut().unwrap();
    match prof {
      Profile::Test => {
        s.push_str("-test");
        params.timelimit = scale_time(params.timelimit, 1.5);
      }
      Profile::Trace => {
        s.push_str("-trace");
        aux_params.soln_log = true;
        params.timelimit = scale_time(params.timelimit, 2.5);
      }
      _ => {}
    }
  }
}

mod instance_groups {
  use std::ops::Range;
  pub const A_TW25: Range<usize> = 0..20;
  pub const A_TW50: Range<usize> = 20..40;
  pub const A_TW100: Range<usize> = 40..60;
  pub const A_TW200: Range<usize> = 60..80;

  pub const B_TW25: Range<usize> = 80..100;
  pub const B_TW50: Range<usize> = 100..120;
  pub const B_TW100: Range<usize> = 120..140;
  pub const B_TW200: Range<usize> = 140..160;

  pub const MK: Range<usize> = 160..190;
}

impl ResourcePolicy for ApvrpExp {
  fn time(&self) -> Duration {
    let i = self.inputs.index;
    let mut limit = self.parameters.timelimit + 300;
    if let Some(&seconds) = self.aux_params.slurm_time.get(&i) {
      limit = limit.min(seconds.round() as u64);
    }
    Duration::from_secs(limit)
  }

  fn memory(&self) -> MemoryAmount {
    self.aux_params.slurm_memory.get(&self.inputs.index)
      .map(|&m| MemoryAmount::from_bytes(m.round() as usize))
      .unwrap_or(MemoryAmount::from_gb(8))
  }

  fn script(&self) -> String {
    let script = match self.profile {
      Profile::Test | Profile::Default => include_str!("slurm_job.sh"),
      Profile::Trace => include_str!("slurm_job_trace.sh"),
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
      if self.profile == Profile::Trace {
        write!(s, "-{}", self.inputs.index).unwrap();
      }
    }
    s
  }

  fn constraint(&self) -> Option<String> {
    match self.profile {
      Profile::Default => Some("R640".to_string()),
      _ => None,
    }
  }

  fn exclude(&self) -> Option<String> {
    // the nerve of this fucking node
    Some("smp-7-3".into())
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
    let fingerprint = unsafe { std::mem::transmute(fingerprint) };
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
  pub fn new() -> BoundsRecorder {
    BoundsRecorder::Init
  }
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
      PostLp(root_lb) => {
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
      PostMip(Bounds { upper_full_obj, .. }) => {
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
      PostMip(bounds) => bounds,
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

  pub fn new_post_mip(
    name: String,
    model: &grb::Model,
    bounds: Bounds,
    cb_stats: CbStats,
  ) -> Result<Self> {
    Ok(PhaseInfo {
      phase: name,
      gurobi: GurobiInfo::from_model(model).context("failed to gather Gurobi info")?,
      bounds: Some(bounds),
      cb: Some(cb_stats),
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
