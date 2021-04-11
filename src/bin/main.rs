use apvrp::*;
use anyhow::Result;
use instances::dataset::{Dataset, apvrp::ApvrpInstance};
use tracing::{info, info_span};

fn dataset(tilk_scale: f64) -> impl Dataset<Instance=ApvrpInstance> {
  use instances::{
    dataset::{self, apvrp::rescale_distances},
    modify::DSetModify
  };
  dataset::DSetCollection::new()
    .push_owned(TILK_AB.map(move |data| rescale_distances(data, tilk_scale)))
    .push_ref(&*MEISEL_A)
    .finish()
}

#[tracing::instrument]
fn main() -> Result<()> {
  let _g = logging::init_logging(Some("log.ndjson"), Some("apvrp.logfilter"));
  let idx: usize = std::env::args().into_iter().skip(1).next().ok_or(anyhow::Error::msg("Expected integer argument"))?.parse()?;
  info!(idx);

  let data = {
    let mut d = dataset(0.5).load_instance(idx)?;
    let _span = info_span!("preprocess", idx, id=%d.id).entered();
    info!("loaded instance");
    preprocess::pv_req_timing_compat(&mut d);
    let d= preprocess::av_grouping(d);
    info!(num_av_groups = d.av_groups.len(), num_av = d.n_active, "preprocessing finished");
    d
  };

  let sets = Sets::new(&data);

  // `pv_req_t_start` is the earliest time we can *depart* from request r's pickup with passive vehicle p
  let pv_req_t_start: Map<_, _> = data.compat_req_passive.iter()
    .flat_map(|(&r, pvs)| {
      let data = &data;
      pvs.iter()
        .map(move |&p| (
          (p, r),
          std::cmp::max(
            data.start_time[&r],
            data.travel_time[&(data.odepot, p)] + data.travel_time[&(p, r)] + data.srv_time[&r],
          )
        ))
    })
    .collect();

  let tasks = Tasks::generate(&data, &sets, &pv_req_t_start);
  info!(num_tasks=tasks.all.len(), "task generation finished");

  // println!("{:#?}",&tasks.by_id);
  // let t = &tasks.by_id[&170];
  // println!("{:#?}", t);
  // println!("Successors: {:#?}", &tasks.succ[t]);
  // println!("Predecessors: {:#?}", &tasks.pred[t]);

  // let mut cnt = 0;
  // for t1 in &tasks.all {
  //   for t2 in &tasks.all {
  //     if task_incompat(t1, t2, &data).is_none() {
  //       cnt += 1;
  //     }
  //   }
  // }
  // println!("{}", cnt);

  let mut mp_model = model::mp::TaskModelMaster::build(&data, &sets, &tasks)?;
  mp_model.model.set_param(grb::param::LazyConstraints, 1)?;
  mp_model.model.update()?;
  mp_model.model.write("scrap.lp")?;
  let mut callback = model::cb::Cb::new(&data, &sets, &tasks, mp_model.vars.clone());
  mp_model.model.optimize_with_callback(&mut callback)?;


  let tasks: Vec<RawPvTask> = tasks.all.into_iter().filter_map(RawPvTask::new).collect();
  let task_filename = format!("scrap/tasks/{}.json", idx);
  std::fs::write(task_filename, serde_json::to_string_pretty(&tasks)?)?;


  Ok(())
}