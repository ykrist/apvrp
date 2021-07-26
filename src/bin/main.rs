use anyhow::Result;
use tracing::{info, error};
use grb::prelude::*;
use slurm_harray::{Experiment, handle_slurm_args};
use apvrp::model::{Phase, mp::{ObjWeights, TaskModelMaster}};
use std::io::Write;
use instances::dataset::apvrp::LocSetStarts;
use apvrp::*;
use apvrp::test::test_data_dir;


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




#[tracing::instrument]
fn main() -> Result<()> {
  #[allow(non_snake_case)]
    let MIN_BP_FORBID = grb::parameter::Undocumented::new("GURO_PAR_MINBPFORBID")?;

  let exp: experiment::ApvrpExp = handle_slurm_args()?;
  exp.write_index_file()?;
  exp.write_parameter_file()?;
  let _g = logging::init_logging(Some(exp.get_output_path(&exp.outputs.trace_log)), Some("apvrp.logfilter"));
  info!(inputs=?exp.inputs, params=?exp.parameters);

  let lookups = Lookups::load_data_and_build(exp.inputs.index)?;

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
  mp.model.set_param(param::TimeLimit, exp.parameters.timelimit as f64)?;
  mp.model.update()?;
  mp.model.write("master_problem.lp")?;

  mp.model.set_obj_attr_batch(attr::Sense, mp.cons.num_av.values().map(|&c| (c, ConstrSense::Equal)))?;
  mp.model.update()?;


  let mut callback = {
    let initial_phase = if exp.parameters.two_phase { Phase::NoWaitCost } else { Phase::Final };
    model::cb::Cb::new(&lookups, &exp, &mp, initial_phase)?
  };

  let obj = loop {
    match mp.model.optimize_with_callback(&mut callback) {
      // match mp.model.optimize() {
      Err(e) => {
          #[cfg(debug_assertions)]
          match callback.error.take() {
            // errors handled
            // Some(model::cb::CbError::InvalidBendersCut { obj, mut solution, .. }) => {
            //   callback.flush_cut_cache(&mut mp.model)?;
            //   // solution.objective = None;  // FIXME
            //   // mp.fix_solution(&solution)?; // FIXME add back in
            //   mp.model.add_constr("debug", c!(mp.vars.theta.values().grb_sum() <= obj))?;
            //   infeasibility_analysis(&mut mp)?;
            //   anyhow::bail!("bugalug")
            // }
            // errors propagated
            _ => {
              tracing::error!(err=%e, "error during optimisation");
              return Err(e.into());
            }
          }

          #[cfg(not(debug_assertions))] {
            return Err(e.into())
          }
        }
      Ok(_) => {}
    };

    match mp.model.status()? {
      Status::Infeasible => {
        callback.flush_cut_cache(&mut mp.model)?;
        infeasibility_analysis(&mut mp)?;
        anyhow::bail!("bugalug")
      }
      Status::Optimal => {}
      status => anyhow::bail!("unexpected master problem status: {:?}", status)
    }

    let obj = print_obj_breakdown(&mp)?;
    callback.stats.print_cut_counts();


    match callback.phase {
      Phase::Final => break obj,
      Phase::NoWaitCost => {
        callback.flush_cut_cache(&mut mp.model)?;
        callback.phase.set_next();
        mp.set_objective(&lookups, obj_weights)?;
      }
    }
  };


  let sol = solution::Solution::from_mp(&mp)?;
  let json_sol = sol.to_serialisable();
  if let Some(sol_log) = callback.sol_log.as_mut() {
    serde_json::to_writer(&mut *sol_log, &json_sol)?; // need to re-borrow here
    write!(sol_log, "\n")?;
  }
  json_sol.to_json_file(exp.get_output_path(&format!("{}-soln.json", exp.inputs.index)))?;

  let sol = sol.solve_for_times(&lookups)?;
  sol.print_objective_breakdown(&lookups, &obj_weights);

  callback.flush_cut_cache(&mut mp.model)?;
  mp.model.update()?;
  mp.model.write(exp.get_output_path_prefixed("master_problem.lp").to_str().unwrap())?;

  if let Some(true_soln) = true_soln {
    if true_soln.objective != Some(obj) {
      true_soln.to_serialisable().to_json_file(exp.get_output_path_prefixed("true_soln.json"))?;
      error!(correct = true_soln.objective.unwrap(), obj, "objective mismatch");
      true_soln.solve_for_times(&lookups)?.print_objective_breakdown(&lookups, &obj_weights);
      mp.fix_solution(&true_soln)?;
      infeasibility_analysis(&mut mp)?;
      anyhow::bail!("bugalug");
    }
  }

  let info = experiment::Info {
    gurobi: experiment::GurobiInfo::new(&mp.model)?,
  };

  info.to_json_file(exp.get_output_path(&exp.outputs.info))?;

  Ok(())
}