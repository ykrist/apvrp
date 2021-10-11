use structopt::*;
use std::path::{PathBuf, Path};
use apvrp::*;
use apvrp::solution::{iter_solution_log, MpSolution};
use slurm_harray::Experiment;
use daggylp::{SolveStatus, InfKind};
use daggylp::viz::GraphViz;
use serde::{Serialize, Deserialize};
use sawmill::InferenceModel;
use apvrp::model::mp::MpVar;
use apvrp::model::sp::{SpConstr};
use apvrp::model::sp::dag::GraphModel;
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
  dag: &GraphModel,
  sol: &MpSolution,
  iis: Option<&daggylp::Iis>,
) -> anyhow::Result<()> {
  use apvrp::model::sp::*;
  let active_vars: Set<_> = sol.mp_vars().collect();
  let active_cons = inf_model.implied_constraints(&active_vars);

  let cover = iis.map(|iis| {
    let mut iis_cons = Set::default();
    for (v1, v2) in iis.iter_edge_constraints() {
      let pt1 = dag.var_to_task[&v1];
      let pt2 = dag.var_to_task[&v2];
      let t1 = IdxTask::from(pt1.index());
      let t2 = IdxTask::from(pt2.index());
      for c in inf_model.constraints() {
        if let SpConstr::Delta(s1, s2, _) = c {
          if s1 == &t1 && s2 == &t2 {
            iis_cons.insert(c.clone());
          }
        }
      }
    }

    if let Some((lb_var, ub_var)) = iis.bounds() {
      let lb_pt = dag.var_to_task[&lb_var];
      let ub_pt = dag.var_to_task[&ub_var];
      iis_cons.insert(SpConstr::Lb(lb_pt.index().into(), lb_pt.t_release));
      let ub_t = ub_pt.index().into();
      iis_cons.insert(SpConstr::Ub(ub_t, ub_pt.t_deadline - ub_pt.tt));

      println!("{:#?}", &lookups.tasks.task_to_pvtasks[&lookups.tasks.by_index[&ub_t]]);
    }
    inf_model.remove_dominated_constraints(&mut iis_cons);
    println!("{:?}\n{:?}", &iis_cons, &active_cons);
    if !iis_cons.is_subset(&active_cons) {
      let diff = iis_cons.difference(&active_cons).collect::<Set<_>>();
      std::fs::write("dump.json", serde_json::to_string(&inf_model.constraints().collect::<Vec<_>>()).unwrap()).unwrap();
      panic!("inactive iis constraints: {:?}", diff);

    }
    inf_model.cover_default(&active_vars, &iis_cons)
  });


  let mut viz = inf_model.viz()
    .active_clauses(&active_vars)
    .active_constraints(&active_cons);

  if let Some(cover) = cover {
    let l = inf_model.lift_cover(&cover, &mut apvrp::model::sp::CoverLift::new(lookups));
    viz = viz.cover(&cover).lifted_cover(&l);
  }
  viz.render_svg(&out.get_filepath("-inference.svg"))?;
  Ok(())
}

fn process_solution(mut out: OutputFiles,
                    lookups: &Lookups,
                    inf_model: &InferenceModel<MpVar, SpConstr>,
                    sol: MpSolution,
) -> anyhow::Result<()> {
  let mut graph = apvrp::model::sp::dag::GraphModel::build(&lookups, &sol)?;

  for _ in 0.. {
    graph.model.write_debug(out.get_filepath(".txt"))?;

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
      .save_svg(&filename);
    info!(%filename, "wrote");

    match iis.as_ref() {
      Some(iis) => {
        graph.model.remove_iis(iis);
        println!("infeasible")
      }
      None => {}
    }

    sawmill_graph(&out, lookups, inf_model, &graph, &sol, iis.as_ref())?;
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