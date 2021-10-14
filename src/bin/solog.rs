use structopt::*;
use std::path::{PathBuf, Path};
use apvrp::*;
use apvrp::solution::{iter_solution_log, MpSolution};
use slurm_harray::Experiment;
use daggylp::{InfKind, viz::GraphViz};
use serde::{Serialize, Deserialize};
use sawmill::InferenceModel;
use apvrp::model::mp::MpVar;
use apvrp::model::sp::{Iis, SpConstr, dag::GraphModel, Subproblem, SpStatus};
use apvrp::IoContext;
use apvrp::logging::*;

#[derive(Debug, Clone, StructOpt)]
struct Args {
  #[structopt(parse(from_os_str))]
  index_file: PathBuf,
  #[structopt(short="s")]
  start: Option<usize>,
  #[structopt(short="e")]
  end: Option<usize>
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

struct OutputFiles<'a> {
  dir: &'a Path,
  major_index: u32,
  minor_index: u32,
}

impl<'a> OutputFiles<'a> {
  pub fn new(dir: &'a Path, major_index: u32) -> anyhow::Result<Self> {
    let p = dir.join(major_index.to_string());
    std::fs::create_dir_all(&p).write_context(&p)?;
    Ok(OutputFiles {
      dir,
      major_index,
      minor_index: 0,
    })
  }

  pub fn inc_minor(&mut self) {
    self.minor_index += 1;
  }

  pub fn get_filepath(&self, suffix: &str) -> String {
    let mut p = self.dir.join(self.major_index.to_string());
    p.push(self.minor_index.to_string());
    let mut p = p.into_os_string().into_string().unwrap();
    p.push_str(suffix);
    p
  }
}

fn sawmill_graph(
  out: &OutputFiles,
  lookups: &Lookups,
  inf_model: &InferenceModel<MpVar, SpConstr>,
  sol: &MpSolution,
  iis: Option<&Iis>,
) -> anyhow::Result<()> {
  let active_vars: Set<_> = sol.mp_vars().collect();
  let active_cons = inf_model.implied_constraints(&active_vars);

  let cover = iis.map(|iis| {
    println!("{:?}\n{:?}", &iis, &active_cons);
    let iis_cons = iis.constraints();
    if !iis_cons.is_subset(&active_cons) {
      let diff = iis_cons.difference(&active_cons).collect::<Set<_>>();
      std::fs::write("dump.json", serde_json::to_string(&inf_model.constraints().collect::<Vec<_>>()).unwrap()).unwrap();
      panic!("inactive iis constraints: {:?}", diff);
    }
    inf_model.cover(&active_vars, &iis_cons)
  });

  let mut viz = inf_model.viz()
    .active_clauses(&active_vars)
    .active_constraints(&active_cons);

  if let Some(cover) = cover {
    let l = inf_model.lift_cover(&cover, &mut apvrp::model::sp::CoverLift::new(lookups));
    viz = viz.cover(&cover).lifted_cover(&l);
  }
  viz.render(&out.get_filepath("-inference.svg"))?;
  Ok(())
}

fn process_solution(mut out: OutputFiles,
                    lookups: &Lookups,
                    inf_model: &InferenceModel<MpVar, SpConstr>,
                    sol: MpSolution,
) -> anyhow::Result<()> {
  let mp_vars : Set<_> = sol.mp_vars().collect();
  let sp_cons = {
    let mut s = inf_model.implied_constraints(&mp_vars);
    inf_model.remove_dominated_constraints(&mut s);
    s
  };

  let mut graph = GraphModel::build(&lookups, &sp_cons, sol.sp_objective_tasks());

  for _ in 0.. {
    graph.model.write_debug(out.get_filepath(".txt"))?;

    let (status, iis) = match graph.solve()? {
      SpStatus::Optimal(_, _) => {
        graph.model.compute_mrs();
        (Status::Optimal, None)
      }
      SpStatus::Infeasible(i) => {
        let iis = graph.extract_and_remove_iis(i)?;
        let status = match i {
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
      out.get_filepath(".json"),
      serde_json::to_string_pretty(&info)?,
    )?;

    let var_names = |var: daggylp::Var| {
      match graph.var_to_task.get(&var) {
        Some(t) => format!("{:?}", t),
        None => "ODepot".to_string()
      }
    };

    let filename = out.get_filepath("-sp.svg");
    graph.model.viz()
      .fmt_vars(&var_names)
      .render(&filename)?;
    info!(%filename, "wrote");

    sawmill_graph(&out, lookups, inf_model, &sol, iis.as_ref())?;
    if iis.is_none() {
      break;
    }
    out.inc_minor();
  }
  Ok(())
}

fn main() -> anyhow::Result<()> {
  let logfile = if cfg!(debug_assertions) {
    Some("solog.ndjson")
  } else {
    None
  };
  let _g = apvrp::logging::init_logging(logfile, true)?;


  let args: Args = Args::from_args();
  let exp = experiment::ApvrpExp::from_index_file(&args.index_file)?;

  let sol_log = args.index_file.with_file_name(&exp.outputs.solution_log);
  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;
  let inf_model = apvrp::model::sp::build_inference_graph(&lookups);

  let output_dir = args.index_file.with_file_name(format!("{}-subproblems", exp.inputs.index));
  std::fs::create_dir_all(&output_dir)?;

  for (k, sol) in iter_solution_log(&sol_log)?.enumerate().skip(args.start.unwrap_or(0)) {
    if k > args.end.unwrap_or(usize::MAX) {
      break
    }
    let out = OutputFiles::new(&output_dir, k as u32)?;
    process_solution(out, &lookups, &inf_model, sol.to_solution(&lookups))?;

  }

  Ok(())
}