use structopt::*;
use apvrp::{solution::SerialisableSolution, Lookups};
use anyhow::{Result};
use std::path::PathBuf;
use apvrp::experiment::ApvrpExp;
use slurm_harray::Experiment;
use apvrp::Json;
use apvrp::model::sp::{dag::GraphModel, SpStatus, Subproblem};
use daggylp::viz::GraphViz;
use daggylp::Var;
use apvrp::model::{EdgeConstrKind, edge_constr_kind};


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

  let edge_kind = |v1: Var, v2: Var| {
    match (graph.var_to_task.get(&v1), graph.var_to_task.get(&v2)) {
      (Some(t1), Some(t2)) =>
        edge_constr_kind(
          &lookups.tasks.pvtask_to_task[t1],
          &lookups.tasks.pvtask_to_task[t2],
        ),
      _ => EdgeConstrKind::AvTravelTime,
    }
  };

  let edge_names = |v1, v2, w| {
    format!("{:?} = {}", edge_kind(v1, v2), w)
  };

  graph.model.viz()
    .fmt_vars(&var_names)
    .fmt_edges(&edge_names)
    .save_svg(output_file);

  Ok(())
}