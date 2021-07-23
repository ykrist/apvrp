use structopt::*;
use std::path::PathBuf;
use apvrp::*;
use apvrp::solution::{iter_solution_log};
use slurm_harray::Experiment;
use daggylp::{SolveStatus, InfKind};
use daggylp::viz::GraphViz;
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, StructOpt)]
struct Args {
  #[structopt(parse(from_os_str))]
  index_file: PathBuf
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Status {
  Cycle,
  PathIis,
  Optimal
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SolutionInformation {
  dag_status: Status,
}

fn main() -> anyhow::Result<()> {
  let args : Args = Args::from_args();
  let exp = experiment::ApvrpExp::from_index_file(&args.index_file)?;

  let sol_log = args.index_file.with_file_name(&exp.outputs.solution_log);
  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;
  let dummy_theta = Map::default();
  let output_dir = args.index_file.with_file_name(format!("{}-sp", exp.inputs.index));

  std::fs::create_dir_all(&output_dir)?;

  for (k, sol) in iter_solution_log(&sol_log)?.enumerate() {
    let sol = sol.to_solution(&lookups);
    let mut graph = apvrp::model::sp::dag::GraphModel::build(&lookups, &sol, &dummy_theta)?;
    graph.model.write_debug(output_dir.join(format!("{}.txt", k)))?;
    let status = match graph.model.solve() {
      SolveStatus::Optimal => {
        graph.model.compute_mrs();
        Status::Optimal
      },
      SolveStatus::Infeasible(infkind) => {
        graph.model.compute_iis(true);
        match infkind {
          InfKind::Path => Status::PathIis,
          InfKind::Cycle => Status::Cycle,
        }
      },
    };

    let info = SolutionInformation{
      dag_status : status
    };

    std::fs::write(
      output_dir.join(format!("{}.json", k)),
      serde_json::to_string_pretty(&info)?
    )?;

    let var_names = |var: daggylp::Var| {
      match graph.var_to_task.get(&var) {
        Some(t) => format!("{:?}", t),
        None => "ODepot".to_string()
      }
    };

    graph.model.viz()
      .fmt_vars(&var_names)
      .save_svg(output_dir.join(format!("{}.svg", k)));
  }

  Ok(())
}