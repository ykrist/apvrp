use structopt::*;
use apvrp::{solution::SerialisableSolution, ShorthandTask, Lookups};
use anyhow::{Result};
use dot::*;
use std::path::PathBuf;
use apvrp::experiment::ApvrpExp;
use slurm_harray::Experiment;
use apvrp::Json;
use apvrp::model::sp::{dag::GraphModel, SpStatus, Subproblem};
use daggylp::viz::GraphViz;


#[derive(Debug, Clone, StructOpt)]
struct Args {
  #[structopt(parse(from_os_str))]
  index_file: PathBuf,
  solution_file: PathBuf,
  #[structopt(short="o", long="output")]
  output_file: Option<PathBuf>
}

fn main() -> Result<()> {
  let args : Args = StructOpt::from_args();
  let Args { index_file, solution_file, output_file } = args;
  let exp = ApvrpExp::from_index_file(&index_file)?;
  // let solution_file =  solution_file.unwrap_or_else(||
  //   index_file.with_file_name(&exp.outputs.)
  // );
  let output_file = output_file.unwrap_or_else(||
    solution_file.with_extension("svg")
  );

  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;
  let sol = SerialisableSolution::from_json_file(solution_file)?;
  let sol = sol.to_solution(&lookups);

  let dummy_theta = &Default::default();
  let mut graph = GraphModel::build(&lookups, &sol, dummy_theta)?;

  match graph.solve()? {
    SpStatus::Optimal(_, _) => {
      graph.model.compute_mrs();
    },
    SpStatus::Infeasible(_) => {
      graph.model.compute_iis(true);
    },
  }

  let var_names = |var: daggylp::Var| {
    match graph.var_to_task.get(&var) {
      Some(t) => format!("{:?}", t),
      None => "ODepot".to_string()
    }
  };

  graph.model.viz()
    .fmt_vars(&var_names)
    .save_svg(output_file);

  Ok(())
}