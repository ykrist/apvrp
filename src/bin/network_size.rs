use std::path::PathBuf;

use anyhow::Context;
use apvrp::{Map, Avg, Lookups, Pv};
use instances::dataset::{apvrp::DSET, IdxNameMap};
use serde::{Deserialize, Serialize};
use structopt::StructOpt;

#[derive(Serialize, Deserialize, Clone, Debug)]
struct Output<'a> {
  instance: &'a str, 
  index: usize,
  num_tasks: usize,
  num_task_per_pv: Map<Pv, usize>,
  num_routes: usize,
  num_routes_per_pv: Map<Pv, usize>,
  num_taskarcs: usize,
  num_taskarcs_per_av: Map<Avg, usize>,
}

#[derive(StructOpt, Debug, Clone)]
struct Args {
  /// Dataset index indicating which instance to use
  data_index: usize,
  /// Output to file instead of STDOUT
  #[structopt(short="o")]
  output: Option<PathBuf>
}


fn main() -> anyhow::Result<()> {
  let args = Args::from_args();
  let mut lu = Lookups::load_data_and_build(args.data_index)?;
  lu.generate_pv_routes();
  let lu = lu;
  
  let mut num_task_per_pv : Map<_, _> = lu.sets.pvs().map(|p| (p,0)).collect();
  let mut num_routes_per_pv = num_task_per_pv.clone();
  let mut num_taskarcs_per_av : Map<_, _> = lu.sets.av_groups().map(|a| (a,0)).collect();

  for (pv, _) in lu.tasks.pvtask.keys() {
      *num_task_per_pv.get_mut(pv).unwrap() += 1;
  }

  for r in &lu.pv_routes.as_ref().unwrap().by_id {
    *num_routes_per_pv.get_mut(&r.first().unwrap().p).unwrap() += 1;
  }

  for (av, _, _) in lu.iter_yvars() {
    *num_taskarcs_per_av.get_mut(&av).unwrap() += 1;
  }
  
  let output = Output {
    index: args.data_index,
    instance: &*DSET.index_to_name(args.data_index)?,
    num_routes: num_routes_per_pv.values().sum(),
    num_routes_per_pv,
    num_tasks: lu.tasks.all.len(),
    num_task_per_pv,
    num_taskarcs: lu.tasks.succ.values().map(Vec::len).sum(),
    num_taskarcs_per_av,
  };

  let output = serde_json::to_string_pretty(&output)?;

  if let Some(ref path) = args.output {
    std::fs::write(path, output).with_context(|| format!("unable to write {:?}", path))?;
  } else {
    println!("{}", output);
  }
  Ok(())
}