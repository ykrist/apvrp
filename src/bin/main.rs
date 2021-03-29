use apvrp::*;

use anyhow::Result;

use instances::dataset::Dataset;

fn main() -> Result<()> {
  // dbg!(std::env::args());
  let idx : usize = std::env::args().into_iter().skip(1).next().ok_or(anyhow::Error::msg("Expected integer argument"))?.parse()?;
  let data = DSET.load_instance(idx)?;
  let sets = Sets::new(&data);

  // `pv_req_t_start` is the earliest time we can *depart* from request r's pickup with passive vehicle p
  let pv_req_t_start : Map<_, _> = data.compat_req_passive.iter()
    .flat_map(|(&r, pvs)| {
      let data = &data;
      pvs.iter()
        .map(move |&p| (
          (p,r),
          std::cmp::max(
            data.start_time[&r],
            data.travel_time[&(data.odepot, p)] + data.travel_time[&(p, r)] + data.srv_time[&r]
          )
        ))
    })
    .collect();

  let tasks = Tasks::generate(&data, &sets, &pv_req_t_start);
  // println!("{}", tasks.all.len());
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

  let mut mp_model = TaskModelMaster::build(&data, &sets, &tasks)?;
  mp_model.model.optimize()?;



  // let tasks : Vec<RawTask> = tasks.all.into_iter().map(RawTask::from).collect();
  // let task_filename = format!("scrap/tasks/{}.json", idx);
  // std::fs::write(task_filename, serde_json::to_string_pretty(&tasks)?)?;

  Ok(())
}