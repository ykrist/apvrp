use anyhow::Result;
use tracing::{info, error};
use grb::prelude::*;
use slurm_harray::{Experiment, handle_slurm_args};
use apvrp::model::{Phase, mp::{ObjWeights, TaskModelMaster}, sp};
use std::io::Write;
use instances::dataset::apvrp::LocSetStarts;
use apvrp::*;
use apvrp::test::test_data_dir;
use apvrp::experiment::{PhaseInfo, Bounds};
use apvrp::solution::Solution;
use std::time::Duration;


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

fn evalutate_full_objective(lookups: &Lookups, mp: &TaskModelMaster, obj_weights: &ObjWeights, sol: &Solution) -> Result<Cost> {
  let dummy_theta = Default::default();
  let mut subproblem = sp::dag::GraphModel::build(&lookups, sol, &dummy_theta)?;
  let sp_obj: Time = subproblem.solve_for_obj()?;
  let y_obj_tt : Time = subproblem.second_last_tasks
    .iter()
    .map(|t| lookups.travel_time_to_ddepot(t))
    .sum();

  let true_cost = mp.obj_val()? + (sp_obj + y_obj_tt ) as Cost * obj_weights.av_finish_time;
  Ok(true_cost)
}


#[tracing::instrument]
fn main() -> Result<()> {
  #[allow(non_snake_case)]
    let MIN_BP_FORBID = grb::parameter::Undocumented::new("GURO_PAR_MINBPFORBID")?;

  let exp: experiment::ApvrpExp = handle_slurm_args()?;
  exp.write_index_file()?;
  exp.write_parameter_file()?;
  let _g = logging::init_logging(Some(exp.get_output_path(&exp.outputs.trace_log)), Some("apvrp.logfilter"));
  info!(inputs=?exp.inputs, params=?exp.parameters);

  let mut stopwatch = crate::stopwatch::Stopwatch::new();
  let time_deadline = crate::stopwatch::Deadline::start(
    Duration::from_secs(exp.parameters.timelimit)
  );
  let mut phase_info = Vec::new();

  stopwatch.start(String::from("task_generation"));
  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;

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
  mp.model.update()?;
  mp.model.write("master_problem.lp")?;
  mp.model.set_obj_attr_batch(attr::Sense, mp.cons.num_av.values().map(|&c| (c, ConstrSense::Equal)))?;
  mp.model.update()?;
  phase_info.push(PhaseInfo::new_init(&mp.model)?);

  let mut callback = {
    let initial_phase = if exp.parameters.two_phase { Phase::NoAvTTCost } else { Phase::Final };
    model::cb::Cb::new(&lookups, &exp, &mp, initial_phase)?
  };


  let mut solution = None;
  loop {
    if time_deadline.reached() {
      break;
    }
    stopwatch.lap(callback.phase.name());
    mp.model.set_param(param::TimeLimit, time_deadline.sec_remaining())?;
    let mut bounds = Bounds::new();
    mp.relax_integrality()?;
    mp.model.optimize()?;
    bounds.record_root_lb(&mp)?;
    mp.enforce_integrality()?;

    mp.model.optimize_with_callback(&mut callback)?;

    match mp.model.status()? {
      Status::Infeasible => {
        callback.flush_cut_cache(&mut mp.model)?;
        infeasibility_analysis(&mut mp)?;
        anyhow::bail!("bugalug")
      }
      Status::Optimal | Status::TimeLimit => {}
      status => anyhow::bail!("unexpected master problem status: {:?}", status)
    }

    bounds.record_final_bounds(&mp)?;

    println!();
    print_obj_breakdown(&mp)?;
    println!();
    callback.stats.print_cut_counts();
    println!();

    let sol = Solution::from_mp(&mp)?;
    if matches!(&callback.phase, Phase::NoAvTTCost) {
      bounds.record_ub_full_obj(evalutate_full_objective(&lookups, &mp, &obj_weights, &sol)?);
    }
    solution = Some(sol);

    phase_info.push(PhaseInfo::new_post_mip(
      callback.phase.name(),
      &mp.model,
      bounds.finish(),
      callback.reset_stats(),
    )?);

    callback.flush_cut_cache(&mut mp.model)?;

    match callback.phase {
      Phase::Final => {
        break;
      },
      Phase::NoAvTTCost => {
        callback.phase.set_next();
        mp.set_objective(&lookups, obj_weights)?;
      }
    }
  };

  stopwatch.stop();
  mp.model.update()?;
  mp.model.write(exp.get_output_path_prefixed("master_problem.lp").to_str().unwrap())?;

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

  println!();
  stopwatch.print_laps();

  let info = experiment::Info {
    time: stopwatch.into_laps(),
    info: phase_info,
  };

  info.to_json_file(exp.get_output_path(&exp.outputs.info))?;

  Ok(())
}