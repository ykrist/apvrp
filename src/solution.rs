use crate::*;
use crate::tasks::{Task};
use crate::model::sp::TimingSubproblem;
use crate::model::cb::{AvPath, PvPath, get_var_values};

#[derive(Clone)]
pub struct SpSolution<'a> {
  pub av_routes: Vec<(Avg, Vec<(Task, Time)>)>,
  pub pv_routes: Vec<(Pv, Vec<(Task, Time)>)>,
  pub data: & 'a Data,
}


impl<'a> SpSolution<'a> {
  pub fn from_sp(sp: &'a TimingSubproblem) -> Result<SpSolution<'a>> {
    let task_start_times : Map<_, _> = get_var_values(&sp.model, &sp.vars)?
      .map(|(task, time)| (task, time.round() as Time))
      .collect();

    let mut avr : Vec<_> = sp.av_routes.iter()
      .flat_map(|(&av, routes)| {
        let task_start_times = &task_start_times;
        routes.iter().map(move |route| {
          let mut sched: Vec<_> = route[..route.len()-1].iter()
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

    let mut pvr : Vec<_> = sp.pv_routes.iter()
      .map(|(&pv, route)| {
        let sched = route.iter().map(|t| (*t, task_start_times[t])).collect();
        (pv, sched)
      })
      .collect();

    pvr.sort_by_key(|(pv, _)| *pv);

    Ok(SpSolution{ av_routes: avr, pv_routes: pvr, data: &sp.data })
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