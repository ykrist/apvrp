use anyhow::Result;
use tracing::{info, error};
use grb::prelude::*;
use slurm_harray::{Experiment, handle_slurm_args};
use apvrp::model::{Phase, mp::{ObjWeights, TaskModelMaster}, sp};
use std::io::Write;
use instances::dataset::apvrp::LocSetStarts;
use apvrp::*;
use apvrp::test::test_data_dir;
use apvrp::experiment::*;
use apvrp::solution::MpSolution;
use std::time::Duration;
use sawmill::InferenceModel;
use apvrp::model::mp::MpVar;
use apvrp::model::sp::SpConstr;


fn infeasibility_analysis(mp: &mut TaskModelMaster) -> Result<()> {
  mp.model.update()?;
  mp.model.write("debug.lp")?;
  mp.model.optimize()?;


  match mp.model.status()? {
    Status::Infeasible => {
      let _s = tracing::error_span!("infeasible_model").entered();
      mp.model.set_param(param::IISMethod, 1)?;
      assert_eq!(mp.model.get_param(param::IISMethod)?, 1);
      mp.model.compute_iis()?;

      let constrs = mp.model.get_constrs()?;
      let iis_constrs: Vec<_> = constrs.iter()
        .copied()
        .zip(mp.model.get_obj_attr_batch(attr::IISConstr, constrs.iter().copied())?)
        .filter(|(_, is_iis)| *is_iis > 0)
        .map(|(c, _)| c)
        .collect();

      for name in mp.model.get_obj_attr_batch(attr::ConstrName, iis_constrs)? {
        error!(constr=%name, "iis constr");
      }

      // for var in mp.model.get_vars()? {
      //   if mp.model.get_obj_attr(attr::IISLB, var)? > 0 {
      //     let name = mp.model.get_obj_attr(attr::VarName, var)?;
      //     let lb = mp.model.get_obj_attr(attr::LB, var)?;
      //     error!(var=%name, ?lb, "iis bound");
      //   }
      // }
    }
    status => {
      if status == Status::Optimal {
        print_obj_breakdown(mp)?;
      }
      let msg = "model was not infeasible";
      error!(?status, "{}", msg);
      anyhow::bail!("{}", msg)
    }
  }
  Ok(())
}

fn get_total_cost(mp: &TaskModelMaster, vars: impl Iterator<Item=Var>) -> Result<f64> {
  let mut total = 0.;
  let obj_constr = mp.model.get_constr_by_name("Obj")?.unwrap();

  for var in vars {
    let x = mp.model.get_obj_attr(attr::X, &var)?;
    let c = mp.model.get_coeff(&var, &obj_constr)?.abs();
    total += x * c;
  }

  Ok(total)
}

fn print_obj_breakdown(mp: &TaskModelMaster) -> Result<Cost> {
  let pv_costs = get_total_cost(mp, mp.vars.x.values().copied())?;
  let av_costs = get_total_cost(mp, mp.vars.y.values().copied())?;
  let theta_sum = get_total_cost(mp, mp.vars.theta.values().copied())?;

  let obj = mp.model.get_attr(attr::ObjVal)?.round() as Cost;
  println!("Obj = {}", obj);
  println!("PV Costs = {:.2}", pv_costs);
  println!("AV Costs = {:.2}", av_costs);
  println!("Theta total = {:.2}", theta_sum);
  Ok(obj)
}

fn evalutate_full_objective(
  lookups: &Lookups,
  mp: &TaskModelMaster,
  inference_model: &InferenceModel<MpVar, SpConstr>,
  obj_weights: &ObjWeights,
  sol: &MpSolution
) -> Result<Cost> {
  let mp_vars : Set<_> = sol.mp_vars().collect();
  let mut sp_cons = inference_model.implied_constraints(&mp_vars);
  inference_model.remove_dominated_constraints(&mut sp_cons);

  let mut subproblem = sp::dag::GraphModel::build(&lookups, &sp_cons, sol.sp_objective_tasks());
  let sp_obj: Time = subproblem.solve_for_obj()?;
  let true_cost = mp.obj_val()? + sp_obj as Cost * obj_weights.av_finish_time;
  Ok(true_cost)
}

fn run(exp: ApvrpExp) -> Result<()> {
  #[allow(non_snake_case)]
    let MIN_BP_FORBID = grb::parameter::Undocumented::new("GURO_PAR_MINBPFORBID")?;

  info!(inputs=?exp.inputs, params=?exp.parameters);

  let mut stopwatch = Stopwatch::new();
  let time_deadline = Deadline::start(
    Duration::from_secs(exp.parameters.timelimit)
  );
  let mut phase_info = Vec::new();

  stopwatch.start(String::from("task_generation"));

  let lookups = {
    let mut lu = Lookups::load_data_and_build(exp.inputs.index)?;
    if exp.parameters.pvcg {
      lu.generate_pv_routes()
    }
    lu
  };

  stopwatch.lap(String::from("inf_model_build"));
  let inference_model = model::sp::build_inference_graph(&lookups);

  stopwatch.lap(String::from("mp_build"));
  let obj_weights = ObjWeights::default();
  let mut mp = model::mp::TaskModelMaster::build(&lookups)?;
  mp.set_objective(&lookups,
                   if exp.parameters.two_phase { obj_weights.without_av_finish_time() } else { obj_weights })?;

  mp.model.update()?;

  let true_soln = solution::load_michael_soln(
    test_data_dir().join(format!("soln/{}.json", exp.inputs.index)),
    &lookups,
    &LocSetStarts::new(lookups.data.n_passive, lookups.data.n_req),
  )
    .map_err(|e| error!(error=%e, "unable to load solution"))
    .ok();

  mp.model.set_obj_attr_batch(attr::BranchPriority, mp.vars.u.values().map(|&u| (u, 100)))?;
  mp.model.set_obj_attr_batch(attr::UB, mp.vars.u.values().map(|&u| (u, 0.0)))?;

  mp.model.set_param(&MIN_BP_FORBID, 1)?;
  mp.model.set_param(param::BranchDir, 1)?;
  mp.model.set_param(param::Threads, exp.parameters.cpus as i32)?;
  mp.model.set_param(param::LazyConstraints, 1)?;
  mp.apply_gurobi_parameters(&exp)?;

  mp.model.update()?;
  mp.model.write("master_problem.lp")?;
  mp.model.set_obj_attr_batch(attr::Sense, mp.cons.num_av.values().map(|&c| (c, ConstrSense::Equal)))?;
  mp.model.update()?;
  phase_info.push(PhaseInfo::new_init(&mp.model)?);

  let mut callback = {
    let initial_phase = if exp.parameters.two_phase { Phase::NoAvTTCost } else { Phase::Final };
    model::cb::Cb::new(&lookups, &exp, &mp, &inference_model, initial_phase, obj_weights)?
  };


  let mut solution = None;
  while !time_deadline.reached() {
    stopwatch.lap(callback.phase.name());
    mp.model.set_param(param::TimeLimit, time_deadline.sec_remaining().max(0.0))?;
    let mut bounds = Bounds::new();
    mp.relax_integrality()?;
    mp.model.update()?;
    debug_assert_eq!(mp.model.get_attr(attr::IsMIP)?, 0);
    mp.model.optimize()?;
    bounds.record_root_lb(&mp)?;
    mp.enforce_integrality()?;

    mp.model.optimize_with_callback(&mut callback)?;

    let optimal = match mp.model.status()? {
      Status::Infeasible => {
        callback.flush_cut_cache(&mut mp.model)?;
        infeasibility_analysis(&mut mp)?;
        anyhow::bail!("bugalug")
      }
      Status::Optimal => true,
      Status::TimeLimit => false,
      status => anyhow::bail!("unexpected master problem status: {:?}", status)
    };

    let solution_found = mp.model.get_attr(attr::SolCount)? > 0;

    bounds.record_final_bounds(&mp)?;

    if solution_found {
      println!();
      print_obj_breakdown(&mp)?;
    }

    println!();
    callback.stats.print_cut_counts();
    println!();

    if solution_found {
      let sol = match MpSolution::from_mp(&mp) {
        Ok(sol) => sol,
        Err(e) => {
          callback.flush_cut_cache(&mut mp.model)?;
          mp.model.write(&exp.get_output_path_prefixed("model_debug.lp").to_str().unwrap())?;
          return Err(e);
        }
      };

      if matches!(&callback.phase, Phase::NoAvTTCost) {
        bounds.record_ub_full_obj(evalutate_full_objective(&lookups, &mp, &inference_model, &obj_weights, &sol)?);
      }
      solution = Some(sol);
    }

    phase_info.push(PhaseInfo::new_post_mip(
      callback.phase.name(),
      &mp.model,
      bounds.finish(),
      callback.reset_stats(),
    )?);

    callback.flush_cut_cache(&mut mp.model)?;
    mp.model.update()?;
    if exp.aux_params.model_file {
      mp.model.write(exp.get_output_path_prefixed("master_problem.lp").to_str().unwrap())?;
    }

    match callback.phase {
      Phase::Final => {
        break;
      }
      Phase::NoAvTTCost => {
        if optimal {
          println!("-------------------------------------------------------------------------------------------------");
          #[cfg(debug_assertions)] {
            if callback.cached_solution.is_none() {
              error!("solved to optimality but no solution has been saved");
              panic!("bugalug")
            }
          }
        }
        mp.set_objective(&lookups, obj_weights)?;
        callback.phase.set_next();
      }
    }
  };

  stopwatch.stop();

  println!();
  stopwatch.print_laps();

  let info = experiment::Info::new(phase_info, stopwatch.into_laps());
  info.to_json_file(exp.get_output_path(&exp.outputs.info))?;


  if let Some(sol) = solution {
    let json_sol = sol.to_serialisable();
    if let Some(sol_log) = callback.sol_log.as_mut() {
      serde_json::to_writer(&mut *sol_log, &json_sol)?; // need to re-borrow here
      write!(sol_log, "\n")?;
    }
    json_sol.to_json_file(exp.get_output_path(&format!("{}-soln.json", exp.inputs.index)))?;

    let sol_with_times = sol.solve_for_times(&lookups)?;
    sol_with_times.print_objective_breakdown(&lookups, &obj_weights);

    if matches!(mp.model.status()?, Status::Optimal) {
      if let Some(true_soln) = true_soln {
        let obj = sol.objective.unwrap();
        let true_obj = true_soln.objective.unwrap();

        if obj != true_obj {
          true_soln.to_serialisable().to_json_file(exp.get_output_path_prefixed("true_soln.json"))?;
          error!(correct = true_obj, obj, "objective mismatch");
          true_soln.solve_for_times(&lookups)?.print_objective_breakdown(&lookups, &obj_weights);
          mp.fix_solution(&true_soln)?;
          infeasibility_analysis(&mut mp)?;
          anyhow::bail!("bugalug");
        }
      }
    }
  }
  Ok(())
}

#[tracing::instrument(level = "error")]
fn main() -> Result<()> {
  apvrp::check_commit_hash()?;
  let exp: experiment::ApvrpExp = handle_slurm_args()?;
  exp.write_index_file()?;
  exp.write_parameter_file()?;
  let _g = logging::init_logging(Some(exp.get_output_path(&exp.outputs.trace_log)), !exp.aux_params.quiet)?;

  let result = run(exp);
  if let Err(err) = result.as_ref() {
    error!("{}", err);
  }
  result
}
