use crate::*;
use std::cmp::max;
use tracing::trace;

#[inline]
fn pv_forward_step(t: &mut Time, data: &Data, t1: &Task, t2: &Task) {
  let arrival = *t + t1.tt + *data.srv_time.get(&t1.end).unwrap_or(&0);
  debug_assert_eq!(t1.p, t2.p);
  debug_assert_eq!(t1.end, t2.start);
  *t = max(arrival, t2.t_release);
}

#[inline]
fn av_forward_step(t: &mut Time, data: &Data, t1: &Task, t2: &Task) {
  let mut arrival = *t + t1.tt + data.travel_time[&(t1.end, t2.start)];
  if t1.p == t2.p {
    arrival += *data.srv_time.get(&t1.end).unwrap_or(&0);
  }
  *t = max(arrival, t2.t_release);
}

#[tracing::instrument(level="trace", skip(data))]
pub fn check_pv_route(data: &Data, tasks: &[Task]) -> bool {
  let mut t = tasks[0].t_release;
  trace!(t);
  for (t1, t2) in tasks.iter().tuple_windows() {
    pv_forward_step(&mut t, data, t1, t2);
    if t + t2.tt > t2.t_deadline {
      trace!(t, "{:#?} to {:#?}", t1, t2);
      return false;
    }
    trace!(t, t2.end.srv_time=?data.srv_time.get(&t2.end), t2.t_release, t2.tt, t2.t_deadline);
  }
  true
}

pub fn pv_route(data: &Data, tasks: &[Task]) -> Vec<Time> {
  let mut t = tasks[0].t_release;
  let mut schedule = Vec::with_capacity(tasks.len());
  schedule.push(tasks[0].t_release);

  for (t1, t2) in tasks.iter().tuple_windows() {
    pv_forward_step(&mut t, data, t1, t2);
    schedule.push(t)
  }

  schedule
}

pub fn av_route(data: &Data, tasks: &[Task]) -> Vec<Time> {
  let mut t = tasks[0].t_release;
  let mut schedule = Vec::with_capacity(tasks.len());
  schedule.push(t);

  for (t1, t2) in tasks.iter().tuple_windows() {
    av_forward_step(&mut t, data, t1, t2);
    schedule.push(t)
  }

  schedule
}

pub fn check_av_route(data: &Data, tasks: &[Task]) -> bool {
  let mut t = tasks[0].t_release;
  for (t1, t2) in tasks.iter().tuple_windows() {
    av_forward_step(&mut t, data, t1, t2);
    if t + t2.tt > t2.t_deadline {
      return false;
    }
  }
  true
}

