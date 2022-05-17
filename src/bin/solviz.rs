use apvrp::logging::*;
use apvrp::model::mp::MpVar;
use apvrp::model::sp::{dag::GraphModel, SpConstr, SpStatus};
use apvrp::solution::{MpSolution, SerialisableSolution};
use apvrp::IoContext;
use apvrp::*;
use clap::Parser;
use daggylp::{viz::GraphViz, InfKind};
use fnv::FnvHashSet;
use indicatif::ProgressIterator;
use labrat::Experiment;
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

type InferenceModel = sawmill::InferenceModel<MpVar, SpConstr>;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum SologSelection {
  Single(usize),
  RangeInclusive { start: usize, end: usize },
  StartFrom(usize),
  UpTo(usize),
  All,
}

fn get_indices(selections: &[SologSelection], last_index: usize) -> Vec<usize> {
  use SologSelection::*;
  if selections.contains(&All) {
    return (0..=last_index).collect();
  }
  let mut inds = FnvHashSet::default();

  for &s in selections {
    match s {
      Single(i) => {
        inds.insert(i);
      }
      UpTo(i) => {
        inds.extend(0..=i);
      }
      RangeInclusive { start, end } => inds.extend(start..=end),
      StartFrom(i) => inds.extend(i..=last_index),
      All => unreachable!(),
    }
  }

  let mut inds: Vec<_> = inds.into_iter().collect();
  inds.sort_unstable();
  inds
}

fn parse_selection_range(s: &str) -> anyhow::Result<SologSelection> {
  use SologSelection::*;
  const ERROR_MSG: &'static str =
    "Range patterns must be of the form A-B, A- or -B where A and B are non-negative integers.";

  if let Ok(i) = s.parse::<usize>() {
    return Ok(Single(i));
  }

  if s == "-" {
    return Ok(All);
  }

  let mut components = s.split('-');
  let start = components
    .next()
    .ok_or_else(|| anyhow::anyhow!(ERROR_MSG))?;
  let end = components
    .next()
    .ok_or_else(|| anyhow::anyhow!(ERROR_MSG))?;
  if !components.next().is_none() {
    anyhow::bail!(ERROR_MSG);
  }

  let selection = if start.is_empty() {
    let end: usize = end.parse()?;
    UpTo(end)
  } else if end.is_empty() {
    let start: usize = start.parse()?;
    StartFrom(start)
  } else {
    let start: usize = start.parse()?;
    let end: usize = end.parse()?;
    RangeInclusive { start, end }
  };
  Ok(selection)
}

#[derive(Debug, Clone, Parser)]
struct Args {
  index_file: PathBuf,
  /// Select subproblems from the solution log.  Assumes a '*-sollog.ndjson' file
  #[clap(short='s', long="select", parse(try_from_str=parse_selection_range))]
  selection: Vec<SologSelection>,
  /// View the final solution.  Assumes a '*-soln.json' file
  #[structopt(short = 'f', long = "final")]
  final_soln: bool,
  /// View the final solution.  Asssumes a '*-true_soln.json' file
  #[structopt(short = 't', long = "true")]
  true_soln: bool,
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

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum SolutionSource {
  Solog(usize),
  TrueSoln,
  FinalSoln,
}

struct OutputFiles {
  dir: PathBuf,
  source: SolutionSource,
  minor_index: u32,
}

impl OutputFiles {
  pub fn new(
    index_file: impl AsRef<Path>,
    index: usize,
    source: SolutionSource,
  ) -> anyhow::Result<Self> {
    let mut dir = index_file.as_ref().parent().unwrap().to_path_buf();

    match source {
      SolutionSource::Solog(i) => {
        dir.push(format!("{}-subproblems", index));
        dir.push(i.to_string());
      }
      SolutionSource::FinalSoln | SolutionSource::TrueSoln => {
        dir.push(format!("{}-soln", index));
      }
    }

    std::fs::create_dir_all(&dir).write_context(&dir)?;

    Ok(OutputFiles {
      dir,
      source,
      minor_index: 0,
    })
  }

  pub fn inc_minor(&mut self) {
    self.minor_index += 1;
  }

  pub fn get_filepath(&self, suffix: &str) -> String {
    let name = match self.source {
      SolutionSource::Solog(_) => format!("{}{}", self.minor_index, suffix),
      SolutionSource::TrueSoln => format!("true_{}{}", self.minor_index, suffix),
      SolutionSource::FinalSoln => format!("final_{}{}", self.minor_index, suffix),
    };
    self.dir.join(name).into_os_string().into_string().unwrap()
  }
}

fn sawmill_graph(
  out: &OutputFiles,
  lookups: &Lookups,
  inf_model: &InferenceModel,
  sol: &MpSolution,
  constraints: Option<&Set<SpConstr>>,
) -> anyhow::Result<()> {
  let active_vars: Set<_> = sol.mp_vars().collect();
  let active_cons = inf_model.implied_constraints(&active_vars);

  let cover = constraints.map(|constraints| {
    if !constraints.is_subset(&active_cons) {
      let diff = constraints.difference(&active_cons).collect::<Set<_>>();
      std::fs::write(
        "dump.json",
        serde_json::to_string(&inf_model.constraints().collect::<Vec<_>>()).unwrap(),
      )
      .unwrap();
      panic!("inactive iis constraints: {:?}", diff);
    }
    inf_model.cover(&active_vars, &constraints)
  });

  let mut viz = inf_model
    .viz()
    .active_clauses(&active_vars)
    .active_constraints(&active_cons);

  if let Some(cover) = cover {
    let l = inf_model.lift_cover(&cover, &mut apvrp::model::sp::CoverLift::new(lookups));
    viz = viz.cover(&cover).lifted_cover(&l);
  }
  viz.render(&out.get_filepath("-inference.svg"))?;
  Ok(())
}

fn process_solution(
  mut out: OutputFiles,
  lookups: &Lookups,
  inf_model: &InferenceModel,
  sol: SerialisableSolution,
) -> anyhow::Result<()> {
  let sol = sol.to_solution(&lookups);
  let mp_vars: Set<_> = sol.mp_vars().collect();
  let sp_cons = {
    let mut s = inf_model.implied_constraints(&mp_vars);
    inf_model.remove_dominated_constraints(&mut s);
    s
  };

  let mut graph = GraphModel::build(&lookups, &sp_cons, sol.sp_objective_tasks());

  for _ in 0.. {
    graph.model.write_debug(out.get_filepath(".txt"))?;

    let (status, iis) = match graph.solve() {
      SpStatus::Optimal(_) => {
        graph.model.compute_mrs()?;
        (Status::Optimal, None)
      }
      SpStatus::Infeasible => {
        let iis = graph.extract_and_remove_iis();
        let status = match graph.inf_kind.unwrap() {
          InfKind::Path => Status::PathIis,
          InfKind::Cycle => Status::Cycle,
        };
        (status, Some(iis))
      }
    };

    let info = SolutionInformation { dag_status: status };

    std::fs::write(
      out.get_filepath(".json"),
      serde_json::to_string_pretty(&info)?,
    )?;

    let var_names = |var: daggylp::Var| match graph.var_to_task.get(&var) {
      Some(t) => format!("{:?}", t),
      None => "ODepot".to_string(),
    };

    let filename = out.get_filepath("-sp.svg");
    graph.model.viz().fmt_vars(&var_names).render(&filename)?;
    info!(%filename, "wrote");

    sawmill_graph(
      &out,
      lookups,
      inf_model,
      &sol,
      iis.as_ref().map(|i| i.constraints()),
    )?;
    if iis.is_none() {
      break;
    }
    out.inc_minor();
  }
  Ok(())
}

fn process_single_file(
  lookups: &Lookups,
  inf_model: &InferenceModel,
  solution_file: impl AsRef<Path>,
  out: OutputFiles,
) -> anyhow::Result<()> {
  let solution_file = solution_file.as_ref();
  let contents = std::fs::read_to_string(solution_file).read_context(solution_file)?;
  let sol: SerialisableSolution = serde_json::from_str(&contents)?;
  process_solution(out, lookups, inf_model, sol)?;
  Ok(())
}

fn main() -> anyhow::Result<()> {
  let logfile = if cfg!(debug_assertions) {
    Some("solog.ndjson")
  } else {
    None
  };
  let _g = apvrp::logging::init_logging(logfile, true)?;
  let args = Args::parse();
  let exp = experiment::ApvrpExp::from_index_file(&args.index_file)?;

  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;
  let inf_model = apvrp::model::sp::build_inference_graph(&lookups);

  if args.final_soln {
    let out = OutputFiles::new(
      &args.index_file,
      exp.inputs.index,
      SolutionSource::FinalSoln,
    )?;
    let solution_file = args
      .index_file
      .with_file_name(format!("{}-soln.json", exp.inputs.index));
    process_single_file(&lookups, &inf_model, solution_file, out)?;
  }

  if args.true_soln {
    let out = OutputFiles::new(&args.index_file, exp.inputs.index, SolutionSource::TrueSoln)?;
    let solution_file = args
      .index_file
      .with_file_name(format!("{}-true_soln.json", exp.inputs.index));
    process_single_file(&lookups, &inf_model, solution_file, out)?;
  }

  if !args.selection.is_empty() {
    let solog = args.index_file.with_file_name(&exp.outputs.solution_log);
    let solog = std::fs::read_to_string(&solog).read_context(&solog)?;
    let lines: Vec<_> = solog.lines().collect();

    let indices = get_indices(&args.selection, lines.len() - 1);
    let bar = indicatif::ProgressBar::new(indices.len() as u64)
      .with_message("rendering...")
      .with_style(indicatif::ProgressStyle::default_bar().template("{msg} {bar} {pos}/{len}"));

    for k in indices.into_iter().progress_with(bar) {
      let sol: SerialisableSolution = serde_json::from_str(lines[k])?;
      let out = OutputFiles::new(&args.index_file, exp.inputs.index, SolutionSource::Solog(k))?;
      process_solution(out, &lookups, &inf_model, sol)?;
    }
  }

  Ok(())
}
