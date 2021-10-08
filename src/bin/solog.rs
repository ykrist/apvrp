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

fn sawmill_graph(lookups: &Lookups,
                 inf_model: &InferenceModel<MpVar, SpConstr>,
                 output_dir: &Path,
                 dag: &GraphModel,
                 sol: &MpSolution,
                 iis: Option<&daggylp::Iis>,
                 idx: (i32, i32),
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

    if let Some((ub_var, lb_var)) = iis.bounds() {
      let lb_pt = dag.var_to_task[&lb_var];
      let ub_pt = dag.var_to_task[&ub_var];
      iis_cons.insert(SpConstr::Lb(lb_pt.index().into(), lb_pt.t_release));
      iis_cons.insert(SpConstr::Ub(ub_pt.index().into(), ub_pt.t_deadline));
    }
    inf_model.remove_dominated_constraints(&mut iis_cons);
    println!("{:?}\n{:?}", &iis_cons, &active_cons);
    if !iis_cons.is_subset(&active_cons) {
      let diff = iis_cons.difference(&active_cons).collect::<Set<_>>();
      panic!("inactive iis constraints: {:?}", diff);
    }
    assert!(iis_cons.is_subset(&active_cons));
    inf_model.cover_default(&active_vars, &iis_cons)
  });


  let mut viz = inf_model.viz()
    .active_clauses(&active_vars)
    .active_constraints(&active_cons);

  if let Some(cover) = cover {
    let l = inf_model.lift_cover(&cover, &mut apvrp::model::sp::CoverLift::new(lookups));
    viz = viz.cover(cover).lifted_cover(l);
  }
  viz.render_svg(output_dir.join(format!("inference{}-{}.svg", idx.0, idx.1))
    .to_str().unwrap()
  )?;
  Ok(())
}

fn process_solution(lookups: &Lookups,
                    inf_model: &InferenceModel<MpVar, SpConstr>,
                    output_dir: &Path,
                    sol: MpSolution,
                    sol_id: usize) -> anyhow::Result<()> {

  let mut graph = apvrp::model::sp::dag::GraphModel::build(&lookups, &sol)?;

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

    match iis.as_ref() {
      Some(iis) => {
        graph.model.remove_iis(iis);
        println!("infeasible")
      }
      None => {},
    }

    sawmill_graph(lookups, inf_model, output_dir, &graph, &sol, iis.as_ref(),
                  (sol_id as i32, j as i32))?;

    if iis.is_none() {
      break
    }
  }
  Ok(())
}

fn main() -> anyhow::Result<()> {
  let _g = apvrp::logging::init_logging(None::<&str>, true)?;
  let args: Args = Args::from_args();
  let exp = experiment::ApvrpExp::from_index_file(&args.index_file)?;

  let sol_log = args.index_file.with_file_name(&exp.outputs.solution_log);
  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;
  let inf_model = apvrp::model::sp::build_inference_graph(&lookups);

  let output_dir = args.index_file.with_file_name(format!("{}-sp", exp.inputs.index));
  std::fs::create_dir_all(&output_dir)?;

  for (k, sol) in iter_solution_log(&sol_log)?.enumerate() {
    process_solution(&lookups, &inf_model, output_dir.as_path(), sol.to_solution(&lookups), k)?;
  }

  Ok(())
}