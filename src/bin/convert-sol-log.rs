use structopt::*;
use std::path::PathBuf;
use apvrp::*;
use apvrp::solution::{SerialisableSolution, Solution, iter_solution_log};
use slurm_harray::Experiment;
use anyhow::Context;
use daggylp::SolveStatus;
use daggylp::viz::GraphViz;

#[derive(Debug, Clone, StructOpt)]
struct Args {
  #[structopt(parse(from_os_str))]
  index_file: PathBuf
}

fn main() -> anyhow::Result<()> {
  let args : Args = Args::from_args();
  let exp = experiment::ApvrpExp::from_index_file(&args.index_file)?;

  let sol_log = args.index_file.with_file_name(&exp.outputs.solution_log);
  let contents = std::fs::read_to_string(&sol_log).with_context(|| format!("read {:?}", &sol_log))?;

  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;
  let dummy_theta = Map::default();
  let output_dir = args.index_file.with_file_name(format!("{}-sp", exp.inputs.index));
  std::fs::create_dir_all(&output_dir)?;

  for (k, sol) in iter_solution_log(&sol_log)?.enumerate() {
    let sol = sol.to_solution(&lookups);
    let mut graph = apvrp::model::sp::dag::GraphModel::build(&lookups, &sol, &dummy_theta)?;
    graph.model.write_debug(output_dir.join(format!("{}-sp.txt", k)))?;
    match graph.model.solve() {
      SolveStatus::Optimal => { graph.model.compute_mrs(); },
      SolveStatus::Infeasible(_) => { graph.model.compute_iis(true); },
    }

    // println!("{:#?}", &graph.var_to_task.keys().collect::<Vec<_>>());
    // println!("{:#?}", &graph.model);

    let var_names = |var: daggylp::Var| {
      match graph.var_to_task.get(&var) {
        Some(t) => format!("{:?}", t),
        None => "ODepot".to_string()
      }
    };

    graph.model.viz()
      .fmt_vars(&var_names)
      .save_svg(output_dir.join(format!("{}-sp.svg", k)));
  }

  Ok(())
}