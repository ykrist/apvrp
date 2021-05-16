use apvrp::*;
use anyhow::Result;
use instances::dataset::{Dataset, apvrp::ApvrpInstance};
use tracing::{info, info_span};
use apvrp::model::mp::ObjWeights;
use instances::dataset::apvrp::LocSetStarts;
use tracing::trace;
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

  let soln = solution::load_michael_soln(
    format!("/home/yannik/phd/src/apvrp/scrap/soln/{}.json", exp.inputs.index),
    &tasks,
    &LocSetStarts::new(data.n_passive, data.n_req))?;

  for (_, r) in &soln.pv_routes {
    mp.model.set_obj_attr_batch(attr::LB, r.iter().map(|t| (mp.vars.x[t], 1.0)))?;
  }

  for (a, r) in &soln.av_routes {
    for (t1, t2) in r.iter().tuple_windows() {
      if let Some(y) = mp.vars.y.get(&(*a, *t1, *t2)) {
        mp.model.set_obj_attr(attr::LB, y, 1.0)?;
      } else {
        tracing::error!(?a, ?t1, ?t2, "missing task-task connection");
        panic!("bugalug")
      }
    }
  }

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

  callback.flush_cut_cache(&mut mp.model)?;
  mp.model.update()?;
  // mp.model.write("master_problem.lp")?;

  for (cut_ty, num) in callback.stats.get_cut_counts() {
    println!("Num {:?} Cuts: {}", cut_ty, num);
  }


  // let tasks: Vec<RawPvTask> = tasks.all.into_iter().filter_map(RawPvTask::new).collect();
  // let task_filename = format!("scrap/tasks/{}.json", idx);
  // std::fs::write(task_filename, serde_json::to_string_pretty(&tasks)?)?;


  Ok(())
}