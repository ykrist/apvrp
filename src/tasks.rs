use crate::*;
use std::hash::{Hash, Hasher};
use std::ops::Deref;
use serde::{Serialize};

pub type TaskId = u32;

#[derive(Copy, Clone, Debug, Serialize)]
pub struct RawTask {
  pub start: Loc,
  /// End location
  pub end: Loc,
  /// Passive vehicle
  pub p: Pv,
}

impl From<Task> for RawTask {
  fn from(task: Task) -> RawTask {
    RawTask{ start: task.start, end: task.end, p: task.p }
  }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TaskType {
  /// Move from one request to another (delivery to pickup loc)
  Transfer,
  /// Move from the PV origin to a request pickup
  Start,
  /// Move from request delivery to the PV destination
  End,
  /// Move from the PV origin directly to the PV destination
  Direct,
  /// Fulfill as a request
  Req,
}

#[derive(Copy, Clone, Debug)]
pub struct Task {
  id: TaskId,
  pub ty: TaskType,
  /// Start location
  pub start: Loc,
  /// End location
  pub end: Loc,
  /// Passive vehicle
  pub p: Pv,
  /// Earliest time the active vehicle can leave `start`
  pub t_release: Time,
  /// Latest time the active vehicle can arrive at `end`
  pub t_deadline: Time,
  /// Travel time between start and end of this task
  pub tt: Time,
}

impl PartialEq for Task {
  fn eq(&self, other: &Task) -> bool { self.id() == other.id() }
}

impl Eq for Task {}

impl Hash for Task {
  fn hash<H: Hasher>(&self, state: &mut H) {
    state.write_u32(self.id())
  }
}

impl Task {
  fn next_id() -> TaskId {
    use std::sync::atomic::{Ordering, AtomicU32};
    static NEXT: AtomicU32 = AtomicU32::new(0);
    NEXT.fetch_add(1, Ordering::Relaxed)
  }

  pub fn new(ty: TaskType, start: Loc, end: Loc, p: Pv, t_release: Time, t_deadline: Time, tt: Time) -> Task {
    Task {
      id: Self::next_id(),
      ty,
      start,
      end,
      p,
      t_deadline,
      t_release,
      tt,
    }
  }

  #[inline(always)]
  pub fn id(&self) -> TaskId { self.id }
}

#[derive(Default, Debug, Clone)]
pub struct Tasks {
  pub all: Vec<Task>,
  pub by_id: Map<TaskId, Task>,
  pub by_start: Map<Loc, Vec<Task>>,
  pub by_end: Map<Loc, Vec<Task>>,
  pub by_cover: Map<Req, Vec<Task>>,
  pub succ: Map<Task, Vec<Task>>,
  pub pred: Map<Task, Vec<Task>>,
}

/// Returns true if an active vehicle can perform `t1` followed by `t2` with respect to travel time.
#[inline(always)]
pub fn active_vehicle_time_check(t1: &Task, t2: &Task) -> bool {
  t1.t_release + t1.tt + t2.tt <= t2.t_deadline
}

/// Returns true if an active vehicle can perform `t1` followed by `t2` with respect to all constraints (flow, cover and time)
#[inline(always)]
pub fn active_vehicle_check(t1: &Task, t2: &Task) -> bool {
  t1.end == t2.start && t1.start != t2.end && active_vehicle_time_check(t1, t2)
}

impl Tasks {
  pub fn generate(data: &ApvrpInstance, sets: &Sets, pv_req_t_start: &Map<(Pv, Req), Time>) -> Self {
    let mut all = Vec::with_capacity(500);
    let mut by_cover = map_with_capacity(data.n_req);

    // trivial tasks: PV origin to PV destination
    for po in sets.pv_origins() {
      let dp = data.n_passive + po;
      all.push(Task::new(
        TaskType::Direct,
        po,
        dp,
        po,
        data.travel_time[&(data.odepot, po)],
        data.tmax - data.travel_time[&(dp, data.ddepot)],
        data.travel_time[&(po, dp)],
      ))
    }


    // # tasks that go from PV origin to Req pickup
    for po in sets.pv_origins() {
      for &rp in &data.compat_passive_req[&po] {
        let rd = rp + data.n_req;
        all.push(Task::new(
          TaskType::Start,
          po,
          rp,
          po,
          data.travel_time[&(data.odepot, po)],
          data.end_time[&rd] - data.srv_time[&rp] - data.srv_time[&rd] - data.travel_time[&(rp, rd)],
          data.travel_time[&(po, rp)],
        ))
      }
    }


    // # tasks that go from Req delivery to PV dest
    for po in sets.pv_origins() {
      for &rp in &data.compat_passive_req[&po] {
        let rd = rp + data.n_req;
        let pd = po + data.n_passive;
        let av_dur = data.travel_time[&(rd, pd)];

        all.push(Task::new(
          TaskType::End,
          rd,
          pd,
          po,
          pv_req_t_start[&(po, rp)] + data.srv_time[&rd] + data.travel_time[&(rp, rd)],
          data.tmax - data.travel_time[&(pd, data.ddepot)],
          data.travel_time[&(rd, pd)],
        ))
      }
    }

    // # tasks that go from Req pickup to its delivery
    for po in sets.pv_origins() {
      for &rp in &data.compat_passive_req[&po] {
        let rd = rp + data.n_req;
        let task = Task::new(
          TaskType::Req,
          rp,
          rd,
          po,
          pv_req_t_start[&(po, rp)],
          data.end_time[&rd] - data.srv_time[&rd],
          data.travel_time[&(rp, rd)],
        );
        all.push(task.clone());
        by_cover.entry(rp).or_insert_with(Vec::new).push(task);
      }
    }

    // # tasks that go from a Req delivery to another Pickup
    for po in sets.pv_origins() {
      let requests = &data.compat_passive_req[&po];
      for &r1p in requests {
        let r1d = r1p + data.n_req;

        for &r2p in requests {
          if r1p == r2p {
            continue;
          }
          let r2d = r2p + data.n_req;
          let tt = data.travel_time[&(r1d, r2p)];
          let t_deadline = data.end_time[&r2d] - data.srv_time[&r2d] - data.srv_time[&r2p] - data.travel_time[&(r2p, r2d)];
          let t_release = pv_req_t_start[&(po, r1p)] + data.travel_time[&(r1p, r1d)] + data.srv_time[&r1d];

          if t_release + tt > t_deadline {
            continue;
          }

          all.push(Task::new(
            TaskType::Transfer,
            r1d,
            r2p,
            po,
            t_release,
            t_deadline,
            tt,
          ));
        }
      }
    }

    all.shrink_to_fit();

    // Lookups
    let mut by_start: Map<Loc, Vec<Task>> = map_with_capacity(data.n_loc);
    let mut by_end: Map<Loc, Vec<Task>> = map_with_capacity(data.n_loc);
    let mut by_id: Map<TaskId, Task> = map_with_capacity(all.len());
    let mut succ : Map <_, _ > = map_with_capacity(all.len());
    let mut pred : Map <_, _ > = map_with_capacity(all.len());

    for &t in &all {
      by_id.insert(t.id(), t);
      by_start.entry(t.start).or_default().push(t);
      by_end.entry(t.end).or_default().push(t);
    }

    for &t1 in &all {
      if !sets.pv_dests().contains(&t1.end) {
        let t1_succ = by_start[&t1.end].iter()
          .filter(|&t2| active_vehicle_check(&t1 ,t2))
          .copied()
          .collect();
        succ.insert(t1, t1_succ);
      }
      // TODO: add some debug assertions about which tasks have predecessors and which don't
      if !sets.pv_origins().contains(&t1.start) {
        let t1_pred = by_end[&t1.start].iter()
          .filter(|&t2| active_vehicle_check(t2, &t1))
          .copied()
          .collect();
        pred.insert(t1, t1_pred);
      }
    }

    Tasks { all, by_id, by_cover, by_end, by_start, succ, pred }
  }
}
