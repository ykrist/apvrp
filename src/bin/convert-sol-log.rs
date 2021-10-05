use structopt::*;
use std::path::{PathBuf, Path};
use apvrp::*;
use apvrp::solution::{iter_solution_log, Solution};
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
  Optimal,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct SolutionInformation {
  dag_status: Status,
}

fn sawmill_graph(lookups: &Lookups, sol: &Solution) -> anyhow::Result<()> {
  use apvrp::model::sp::*;
  let model = apvrp::model::sp::build_inference_graph(lookups);
  let active_vars = sol.av_routes.iter()
    .flat_map(|r| {
      dbg!(r);
      r.windows(2).map(move |pair| {
        let t1 = pair[0].shorthand();
        let t2 = pair[1].shorthand();
        MpVar::Y(r.av(), t1, t2)
      })
    })
    .chain(sol.pv_routes.iter().flat_map(|r|
      r.iter().map(|t| MpVar::X(t.p, t.shorthand().into()))
    ))
    .collect::<Set<_>>();
  dbg!(&active_vars);
  let active_cons = model.implied_constraints(&active_vars);
  model.viz()
    .cover(&active_vars, &active_cons)
    .show_active_constraints(true)
    .show_active_clauses(true)
    .render_svg("scrap.svg")?;
  panic!();
}

fn process_solution(lookups: &Lookups, output_dir: &Path, sol: Solution, sol_id: usize) -> anyhow::Result<()> {
  let dummy_theta = Map::default();

  let mut graph = apvrp::model::sp::dag::GraphModel::build(&lookups, &sol, &dummy_theta)?;


  for j in 0.. {
    graph.model.write_debug(output_dir.join(format!("{}-it{}.txt", sol_id, j)))?;

    let (status, iis) = match graph.model.solve() {
      SolveStatus::Optimal => {
        graph.model.compute_mrs();
        (Status::Optimal, None)
      }
      SolveStatus::Infeasible(infkind) => {
        let iis = graph.model.compute_iis(true);
        let status = match infkind {
          InfKind::Path => Status::PathIis,
          InfKind::Cycle => Status::Cycle,
        };
        (status, Some(iis))
      }
    };

    let info = SolutionInformation {
      dag_status: status
    };

    std::fs::write(
      output_dir.join(format!("{}-it{}.json", sol_id, j)),
      serde_json::to_string_pretty(&info)?,
    )?;

    let var_names = |var: daggylp::Var| {
      match graph.var_to_task.get(&var) {
        Some(t) => format!("{:?}", t),
        None => "ODepot".to_string()
      }
    };

    let filename = format!("{}-it{}.svg", sol_id, j);
    graph.model.viz()
      .fmt_vars(&var_names)
      .save_svg(output_dir.join(&filename));
    println!("wrote {}", filename);

    match iis {
      Some(iis) => graph.model.remove_iis(&iis),
      None => break,
    }
    sawmill_graph(lookups, &sol)?;
  }
  Ok(())
}

fn main() -> anyhow::Result<()> {
  let args: Args = Args::from_args();
  let exp = experiment::ApvrpExp::from_index_file(&args.index_file)?;

  let sol_log = args.index_file.with_file_name(&exp.outputs.solution_log);
  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;
  let output_dir = args.index_file.with_file_name(format!("{}-sp", exp.inputs.index));
  std::fs::create_dir_all(&output_dir)?;

  for (k, sol) in iter_solution_log(&sol_log)?.enumerate() {
    process_solution(&lookups, output_dir.as_path(), sol.to_solution(&lookups), k)?;
  }

  Ok(())
}