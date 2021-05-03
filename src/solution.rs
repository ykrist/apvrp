use crate::*;
use crate::tasks::{Task};
use crate::model::sp::TimingSubproblem;
use crate::model::cb::{AvPath, PvPath, get_var_values};
use serde::Deserialize;
use std::fs::read_to_string;
use std::path::Path;
use anyhow::Context;


#[derive(Clone)]
pub struct Solution {
  pub av_routes: Vec<(Avg, Vec<Task>)>,
  pub pv_routes: Vec<(Pv, Vec<Task>)>,
}


#[derive(Clone)]
pub struct SpSolution<'a> {
  pub av_routes: Vec<(Avg, Vec<(Task, Time)>)>,
  pub pv_routes: Vec<(Pv, Vec<(Task, Time)>)>,
  pub data: &'a Data,
}


impl<'a> SpSolution<'a> {
  pub fn from_sp(sp: &'a TimingSubproblem) -> Result<SpSolution<'a>> {
    let task_start_times: Map<_, _> = get_var_values(&sp.model, &sp.vars)?
      .map(|(task, time)| (task, time.round() as Time))
      .collect();

    let mut avr: Vec<_> = sp.av_routes.iter()
      .flat_map(|(&av, routes)| {
        let task_start_times = &task_start_times;
        routes.iter().map(move |route| {
          let mut sched: Vec<_> = route[..route.len() - 1].iter()
            .map(|task| (*task, *task_start_times.get(task).unwrap_or(&0)))
            .collect();

          let (second_last_task, t) = sched.last().unwrap().clone();
          let ddepot_task = route.last().unwrap().clone();
          debug_assert_eq!(ddepot_task.ty, TaskType::DDepot);
          sched.push((ddepot_task, t + second_last_task.tt + sp.data.travel_time[&(second_last_task.end, ddepot_task.start)]));
          (av, sched)
        })
      })
      .collect();

    avr.sort_by_key(|(av, _)| *av);

    let mut pvr: Vec<_> = sp.pv_routes.iter()
      .map(|(&pv, route)| {
        let sched = route.iter().map(|t| (*t, task_start_times[t])).collect();
        (pv, sched)
      })
      .collect();

    pvr.sort_by_key(|(pv, _)| *pv);

    Ok(SpSolution { av_routes: avr, pv_routes: pvr, data: &sp.data })
  }

  pub fn pretty_print(&self) {
    use prettytable::*;

    for (av, route) in &self.av_routes {
      let mut table = Table::new();
      let mut task_row = vec![cell!("Task")];
      let mut st_row = vec![cell!("ST")];
      let mut tt_time = vec![cell!("TT")];

      println!("Active Vehicle Group {}", av);
      for ((task1, t1), (task2, t2)) in route.iter().tuple_windows() {
        task_row.push(cell!(format!("{:?}", task1)));
        task_row.push(cell!(format!("(drive)")));
        st_row.push(cell!(format!("{:?}", t1)));
        st_row.push(cell!(format!("")));
        tt_time.push(cell!(format!("{:?}", task1.tt)));
        tt_time.push(cell!(self.data.travel_time[&(task1.end, task2.start)]));
      }

      if let Some((task, t)) = route.last() {
        task_row.push(cell!(format!("{:?}", task)));
        st_row.push(cell!(format!("{:?}", t)));
        tt_time.push(cell!(format!("{:?}", task.tt)));
      }

      table.add_row(Row::new(task_row));
      table.add_row(Row::new(st_row));
      table.add_row(Row::new(tt_time));
      table.printstd();
      // println!("{:?}", route)
    }

    for (pv, route) in &self.pv_routes {
      let mut table = Table::new();
      let mut task_row = vec![cell!("Task")];
      let mut st_row = vec![cell!("ST")];
      let mut tt_row = vec![cell!("TT")];
      // let mut et_row = vec![cell!("ET")];
      // let mut lt_row = vec![cell!("LT")];

      for (task, t) in route {
        task_row.push(cell!(format!("{:?}", task)));
        st_row.push(cell!(format!("{:?}", t)));
        tt_row.push(cell!(format!("{:?}", task.tt)));

        //
        // et_row.push(
        //   self.data.start_time.get(&task.start)
        //     .map(|t| cell!(format!("{}", t)))
        //     .unwrap_or_else(|| cell!(""))
        // )
      }

      table.add_row(Row::new(task_row));
      // table.add_row(Row::new(et_row));
      table.add_row(Row::new(st_row));
      table.add_row(Row::new(tt_row));

      println!("Passive Vehicle {}", pv);
      table.printstd();
    }
  }
}

#[derive(Deserialize, Copy, Clone, Debug, Hash, Eq, PartialEq)]
struct JsonTask {
  start: RawLoc,
  end: RawLoc,
  p: i32,
}

#[derive(Deserialize, Clone)]
struct JsonSolution {
  index: usize,
  av_routes: Map<String, Vec<Vec<JsonTask>>>,
  pv_routes: Map<String, Vec<JsonTask>>,
}


fn json_task_to_sh_task(lss: &LocSetStarts, st: &JsonTask) -> ShorthandTask {
  use ShorthandTask::*;

  let start = Loc::decode(st.start, lss);
  let end = Loc::decode(st.end, lss);
  let p = if st.p < 0 { None } else { Some(st.p as Pv) };

  match start {
    Loc::Ao => ODepot,
    Loc::Ad => DDepot,
    Loc::ReqP(r) => Request(p.unwrap(), r),
    Loc::Po(p) => {
      match end {
        Loc::Pd(p2) if p2 == p => Direct(p),
        Loc::ReqP(r) => Start(p, r),
        _ => unreachable!("unmatched end: start={:?}, end={:?}", start, end),
      }
    }
    Loc::ReqD(r1) => {
      match end {
        Loc::ReqP(r2) => Transfer(p.unwrap(), r1, r2),
        Loc::Pd(p) => End(p, r1),
        _ => unreachable!("unmatched end: start={:?}, end={:?}", start, end)
      }
    }
    _ => unreachable!("unmatched start: start={:?}, end={:?}", start, end)
  }
}

pub fn load_michael_soln(path: impl AsRef<Path>, tasks: &Tasks, lss: &LocSetStarts) -> Result<Solution> {
  let s = std::fs::read_to_string(path)?;
  let soln: JsonSolution = serde_json::from_str(&s)?;

  let mut av_routes = Vec::new();
  for (av, routes) in &soln.av_routes {
    let av: Avg = av.parse().context("parsing AV")?;
    for r in routes {
      let r: Vec<_> = r.iter()
        .map(|t| tasks.by_shorthand[&json_task_to_sh_task(lss, t)])
        .collect();
      av_routes.push((av, r));
    }
  }

  let mut pv_routes = Vec::new();
  for (pv, route) in &soln.pv_routes {
    let pv: Pv = pv.parse().context("parsing PV")?;
    let route = route.iter()
      .map(|t| tasks.by_shorthand[&json_task_to_sh_task(lss, t)])
      .collect();
    pv_routes.push((pv, route));
  }

  Ok(Solution { av_routes, pv_routes })
}