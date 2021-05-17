use apvrp::*;
use anyhow::Result;
use instances::dataset::{Dataset, apvrp::ApvrpInstance};
use tracing::{info, info_span};
use apvrp::model::mp::ObjWeights;
use instances::dataset::apvrp::LocSetStarts;
use tracing::{error, trace};
use grb::prelude::*;
use slurm_harray::{Experiment, handle_slurm_args};
use itertools::Itertools;

fn dataset(tilk_scale: f64) -> impl Dataset<Instance=ApvrpInstance> {
  use instances::{
    dataset::{self, apvrp::rescale_distances},
    modify::DSetModify,
  };
  dataset::DSetCollection::new()
    .push_owned(TILK_AB.map(move |data| rescale_distances(data, tilk_scale)))
    .push_ref(&*MEISEL_A)
    .finish()
}

#[tracing::instrument(level = "trace", skip(data))]
fn earliest_departures(data: &Data) -> Map<(Pv, Req), Time> {
  data.compat_req_passive.iter()
    .flat_map(|(&r, pvs)| {
      pvs.iter()
        .map(move |&p| {
          let rp = Loc::ReqP(r);
          let po = Loc::Po(p);
          trace!(?rp, ?po);
          let t = std::cmp::max(
            data.start_time[&rp],
            data.travel_time[&(Loc::Ao, po)] + data.travel_time[&(po, rp)] + data.srv_time[&rp],
          );

          ((p, r), t)
        })
    })
    .collect()
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

  let data = {
    let mut d = dataset(0.5).load_instance(exp.inputs.index)?;
    let _span = info_span!("preprocess", idx=exp.inputs.index, id=%d.id).entered();
    info!("loaded instance");
    preprocess::pv_req_timing_compat(&mut d);
    let lss = LocSetStarts::new(d.n_passive, d.n_req);
    let d = preprocess::av_grouping(d, &lss);
    info!(num_av_groups = d.av_groups.len(), num_av = d.n_active, "preprocessing finished");
    d
  };

  let sets = Sets::new(&data);

  // `pv_req_t_start` is the earliest time we can *depart* from request r's pickup with passive vehicle p
  let pv_req_t_start = earliest_departures(&data);
  let tasks = Tasks::generate(&data, &sets, &pv_req_t_start);

  info!(num_tasks = tasks.all.len(), "task generation finished");

  let mut mp = model::mp::TaskModelMaster::build(&data, &sets, &tasks, ObjWeights::default())?;
  mp.model.update()?;

  let true_soln = solution::load_michael_soln(
    format!("/home/yannik/phd/src/apvrp/scrap/soln/{}.json", exp.inputs.index),
    &tasks,
    &LocSetStarts::new(data.n_passive, data.n_req))?;



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
  let mut callback = model::cb::Cb::new(&data, &exp, &sets, &tasks, &mp)?;
  let opt_res = mp.model.optimize_with_callback(&mut callback);
  match opt_res {
    Err(e) => tracing::error!(err=%e, "error during optimisation"),
    Ok(_) => {}
  };

  let sol = solution::Solution::from_mp(&mp)?;

  let sol = sol.solve_for_times(callback.sp_env(), &data, &tasks)?;
  sol.pretty_print(&data);


  for (cut_ty, num) in callback.stats.get_cut_counts() {
    println!("Num {:?} Cuts: {}", cut_ty, num);
  }

  let pv_costs : f64 = mp.model.get_obj_attr_batch(attr::X, mp.vars.x.values().copied())?.into_iter()
    .zip(mp.model.get_obj_attr_batch(attr::Obj, mp.vars.x.values().copied())?)
    .map(|(x, obj)| x*obj)
    .sum();

  let av_costs : f64 = mp.model.get_obj_attr_batch(attr::X, mp.vars.y.values().copied())?.into_iter()
    .zip(mp.model.get_obj_attr_batch(attr::Obj, mp.vars.y.values().copied())?)
    .map(|(x, obj)| x*obj)
    .sum();

  let theta_sum : f64 = mp.model.get_obj_attr_batch(attr::X, mp.vars.theta.values().copied())?.into_iter()
    .zip(mp.model.get_obj_attr_batch(attr::Obj, mp.vars.theta.values().copied())?)
    .map(|(x, obj)| x*obj)
    .sum();

  let obj = mp.model.get_attr(attr::ObjVal)?.round() as Cost;
  println!("Obj = {}", obj);
  println!("PV = {:.2}", pv_costs);
  println!("AV = {:.2}", av_costs);
  println!("Theta = {:.2}", theta_sum);


  callback.flush_cut_cache(&mut mp.model)?;
  mp.model.update()?;
  mp.model.write("master_problem.lp")?;


  if true_soln.objective != obj {
    error!(correct=true_soln.objective, obj, "objective mismatch");
    mp.fix_solution(&true_soln)?;
    mp.model.optimize()?;
    if mp.model.status()? == Status::Infeasible {
      let _s = tracing::error_span!("infeasible_model").entered();
      mp.model.compute_iis()?;

      let constrs = mp.model.get_constrs()?;
      let iis_constrs : Vec<_> = constrs.iter()
        .copied()
        .zip(mp.model.get_obj_attr_batch(attr::IISConstr, constrs.iter().copied())?)
        .filter(|(_, is_iis)| *is_iis > 0)
        .map(|(c, _)| c)
        .collect();

      for name in mp.model.get_obj_attr_batch(attr::ConstrName, iis_constrs)? {
        error!(constr=%name, "iis constr");
      }
    }
    panic!("bugalug");
  }

  // let tasks: Vec<RawPvTask> = tasks.all.into_iter().filter_map(RawPvTask::new).collect();
  // let task_filename = format!("scrap/tasks/{}.json", idx);
  // std::fs::write(task_filename, serde_json::to_string_pretty(&tasks)?)?;
  // FIXME index 62 is wrong

  Ok(())
}