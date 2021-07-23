use crate::*;
use std::cmp::max;
use tracing::trace;

#[inline]
fn pv_forward_step(t: &mut Time, data: &Data, t1: &PvTask, t2: &PvTask) {
  let arrival = *t + t1.tt + *data.srv_time.get(&t1.end).unwrap_or(&0);
  debug_assert_eq!(t1.end, t2.start);
  *t = max(arrival, t2.t_release);
}

#[inline]
fn av_forward_step(t: &mut Time, data: &Data, t1: &Task, t2: &Task) {
  let arrival = if t1.end == t2.start {
    // t1 and t2 must have the same passive vehicle by PV flow constraints
    // so we can add the service time
    *t + t1.tt + data.srv_time.get(&t1.end).copied().unwrap_or(0)
  } else {
    // Different passive vehicles (because we forbid task-task connections where t1.end != t2.start && t1.p == t2.p)
    *t + t1.tt + data.travel_time[&(t1.end, t2.start)]
  };
  *t = max(arrival, t2.t_release);
}

#[tracing::instrument(level="trace", skip(data))]
pub fn check_pv_route(data: impl AsRef<Data>, tasks: &[PvTask]) -> bool {
  let data = data.as_ref();
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

pub fn pv_route(data: impl AsRef<Data>, tasks: &[PvTask]) -> Vec<Time> {
  let data = data.as_ref();
  let mut t = tasks[0].t_release;
  let mut schedule = Vec::with_capacity(tasks.len());
  schedule.push(tasks[0].t_release);

  for (t1, t2) in tasks.iter().tuple_windows() {
    pv_forward_step(&mut t, data, t1, t2);
    schedule.push(t)
  }

  schedule
}

pub fn av_route(data: impl AsRef<Data>, tasks: &[Task]) -> Vec<Time> {
  let data = data.as_ref();
  let mut t = max(tasks[0].t_release, data.travel_time[&(Loc::Ao, tasks[0].start)]);

  let mut schedule = Vec::with_capacity(tasks.len());
  schedule.push(t);

  for (t1, t2) in tasks.iter().tuple_windows() {
    av_forward_step(&mut t, data, t1, t2);
    schedule.push(t)
  }

  schedule
}


/// Computes an underestimate of the earliest time this AV route can be back at the depot.  Expects `tasks[0] != ODp` and that
/// the last task is `DDp`
pub fn av_route_finish_time(data: impl AsRef<Data>, tasks: &[Task]) -> Time {
  let data = data.as_ref();
  debug_assert_ne!(tasks.last().unwrap().ty, TaskType::DDepot);
  debug_assert_ne!(tasks.first().unwrap().ty, TaskType::ODepot);
  let mut t = max(tasks[0].t_release, data.travel_time[&(Loc::Ao, tasks[0].start)]);
  for (t1, t2) in tasks.iter().tuple_windows() {
    av_forward_step(&mut t, data, t1, t2);
  }
  t
}

#[tracing::instrument(level="trace", skip(data))]
pub fn check_av_route(data: impl AsRef<Data>, tasks: &[Task]) -> bool {
  let data = data.as_ref();
  let mut t = tasks[0].t_release;
  for (t1, t2) in tasks.iter().tuple_windows() {
    av_forward_step(&mut t, data, t1, t2);
    if t + t2.tt > t2.t_deadline {
      trace!(?t1, ?t2, t, t2.tt, t2.t_deadline, "illegal");
      return false;
    }
    trace!(?t1, ?t2, t, "step");
  }
  true
}

/// For each Passive Vehicle-Request pair, computes the earliest time we can *leave* the pickup of the request.
#[tracing::instrument(level = "trace", skip(data))]
pub fn earliest_departures(data: impl AsRef<Data>) -> Map<(Pv, Req), Time> {
  let data = data.as_ref();
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