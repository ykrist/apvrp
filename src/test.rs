use crate::experiment::*;
use crate::model::{
  mp::{ObjWeights, TaskModelMaster},
  Phase,
};
use crate::solution::{MpSolution, SpSolution};
use crate::*;
use grb::prelude::*;
use std::path::Path;
use std::time::Duration;

pub fn test_data_dir() -> &'static Path {
  Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/"))
}

pub fn test_output_dir() -> &'static Path {
  Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/outputs/"))
}

pub fn test_params(name: &str) -> Result<Params> {
  let mut path = Path::new(env!("CARGO_MANIFEST_DIR")).join("params/test/");
  path.push(name);
  path.set_extension("json");
  Params::from_json_file(&path).read_context(&path)
}

pub fn get_total_cost(mp: &TaskModelMaster, vars: impl Iterator<Item = Var>) -> Result<f64> {
  let mut total = 0.;
  let obj_constr = mp.model.get_constr_by_name("Obj")?.unwrap();

  for var in vars {
    let x = mp.model.get_obj_attr(attr::X, &var)?;
    let c = mp.model.get_coeff(&var, &obj_constr)?.abs();
    total += x * c;
  }

  Ok(total)
}

pub fn print_obj_breakdown(mp: &TaskModelMaster) -> Result<Cost> {
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

pub fn infeasibility_analysis(mp: &mut TaskModelMaster) -> Result<()> {
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
      let iis_constrs: Vec<_> = constrs
        .iter()
        .copied()
        .zip(
          mp.model
            .get_obj_attr_batch(attr::IISConstr, constrs.iter().copied())?,
        )
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

pub fn test_run(exp: ApvrpExp) -> Result<(Lookups, SpSolution)> {
  #[allow(non_snake_case)]
  let MIN_BP_FORBID = grb::parameter::Undocumented::new("GURO_PAR_MINBPFORBID")?;

  info!(inputs=?exp.inputs, params=?exp.parameters);

  let lookups = {
    let mut lu = Lookups::load_data_and_build(exp.inputs.index)?;
    if exp.parameters.pvcg {
      lu.generate_pv_routes()
    }
    lu
  };

  let inference_model = model::sp::build_inference_graph(&lookups);

  let obj_weights = ObjWeights::default();
  let mut mp = model::mp::TaskModelMaster::build(&lookups)?;
  mp.model.set_param(param::OutputFlag, 0)?;

  mp.set_objective(
    &lookups,
    if exp.parameters.two_phase {
      obj_weights.without_av_finish_time()
    } else {
      obj_weights
    },
  )?;

  mp.model.update()?;

  mp.model
    .set_obj_attr_batch(attr::BranchPriority, mp.vars.u.values().map(|&u| (u, 100)))?;
  mp.model
    .set_obj_attr_batch(attr::UB, mp.vars.u.values().map(|&u| (u, 0.0)))?;

  mp.model.set_param(&MIN_BP_FORBID, 1)?;
  mp.model.set_param(param::BranchDir, 1)?;
  mp.model
    .set_param(param::Threads, exp.parameters.cpus as i32)?;
  mp.model.set_param(param::LazyConstraints, 1)?;
  mp.apply_gurobi_parameters(&exp)?;

  mp.model.update()?;
  mp.model.write("master_problem.lp")?;
  mp.model.update()?;

  let mut callback = {
    let initial_phase = if exp.parameters.two_phase {
      Phase::NoAvTTCost
    } else {
      Phase::Final
    };
    model::cb::Cb::new(
      &lookups,
      &exp,
      &mp,
      &inference_model,
      initial_phase,
      obj_weights,
    )?
  };

  let solution = loop {
    mp.relax_integrality()?;
    mp.model.update()?;
    debug_assert_eq!(mp.model.get_attr(attr::IsMIP)?, 0);
    mp.model.optimize()?;

    if let Status::Infeasible = mp.model.status()? {
      callback.flush_cut_cache(&mut mp.model)?;
      infeasibility_analysis(&mut mp)?;
      anyhow::bail!("bugalug");
    }

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
      status => anyhow::bail!("unexpected master problem status: {:?}", status),
    };

    match callback.phase {
      Phase::Final => {
        break MpSolution::from_mp(&mp)?.solve_for_times(&lookups)?;
      }
      Phase::NoAvTTCost => {
        if optimal {
          if callback.cached_solution.is_none() {
            error!("solved to optimality but no solution has been saved");
            panic!("bugalug")
          }
        }
        callback.flush_cut_cache(&mut mp.model)?;
        mp.model.update()?;
        mp.set_objective(&lookups, obj_weights)?;
        callback.phase.set_next();
      }
    }
  };

  Ok((lookups, solution))
}

#[cfg(feature = "expensive-tests")]
mod expensive {
  use super::*;
  use rayon::prelude::*;
  use std::{fmt::Debug, panic::RefUnwindSafe};

  pub fn batch_test<I, T, F>(inputs: I, test: F) -> Result<()>
  where
    I: IntoParallelIterator<Item = T> + Send,
    T: Debug + Send + Clone + RefUnwindSafe,
    F: Send + Sync + RefUnwindSafe + Fn(T) -> Result<()>,
  {
    let pool = rayon::ThreadPoolBuilder::new()
      .num_threads(num_cpus::get_physical())
      .build()
      .unwrap();

    pool.install(move || {
      let error = inputs.into_par_iter().find_map_any(|i| {
        match std::panic::catch_unwind(|| test(i.clone())) {
          Ok(Ok(())) => None,
          Ok(err) => err.context(format!("test failed with input {:?}", i)).err(),
          Err(_) => Some(anyhow::anyhow!("test paniced with input {:?}", i)),
        }
      });

      match error {
        None => Ok(()),
        Some(e) => Err(e),
      }
    })
  }
}

#[cfg(feature = "expensive-tests")]
pub use expensive::batch_test;
