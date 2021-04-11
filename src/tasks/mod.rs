use crate::*;
use std::hash::{Hash, Hasher};
use std::ops::Deref;
use serde::{Serialize};
use itertools::max;
use fnv::FnvHashSet;
use std::fmt;
use grb::constr::IneqExpr;
use tracing::{error_span, trace, error};

pub type TaskId = u32;
pub type PVIRTaskId = u32;

mod checks;


#[derive(Copy, Clone, Debug, Serialize)]
pub struct RawPvTask {
  pub start: Loc,
  /// End location
  pub end: Loc,
  /// Passive vehicle
  pub p: Pv,
}

impl RawPvTask {
  pub fn new(task: Task) -> Option<RawPvTask> {
    task.p.map(|p| RawPvTask { start: task.start, end: task.end, p })
  }
}

impl From<Task> for RawPvTask {
  fn from(task: Task) -> RawPvTask {
    RawPvTask { start: task.start, end: task.end, p: task.p.unwrap() }
  }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum TaskType {
  /// Move from one request to another (delivery to pickup loc)
  Transfer,
  /// Move from the PV origin to a request pickup
  PvStart,
  /// Move from request delivery to the PV destination
  PvEnd,
  /// Move from the PV origin directly to the PV destination
  Direct,
  /// Fulfill as a request
  Req,
  /// Av origin depot (fake task)
  ODepot,
  /// Av destination depot (fake task)
  DDepot,
}

#[derive(Copy, Clone)]
pub struct Task {
  id: TaskId,
  pub ty: TaskType,
  /// Start location
  pub start: Loc,
  /// End location
  pub end: Loc,
  /// Passive vehicle, `None` only for active-vehicle depot tasks
  pub p: Option<Pv>,
  /// Earliest time the active vehicle can *depart* `start`
  pub t_release: Time,
  /// Latest time the active vehicle can *arrive* at `end`
  pub t_deadline: Time,
  /// Travel time between start and end of this task
  pub tt: Time,
}

#[derive(Copy, Clone, Debug)]
pub struct PVIRTask {
  id: PVIRTaskId,
  pub ty: TaskType,
  /// Start location
  pub start: Loc,
  /// End location
  pub end: Loc,
  /// Earliest time the active vehicle can *depart* `start`
  pub t_release: Time,
  /// Latest time the active vehicle can *arrive* at `end`
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

impl fmt::Debug for Task {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    use TaskType::*;
    use fmt::Debug;

    match self.ty {
      DDepot | ODepot => { return self.ty.fmt(f) }
      _ => {}
    }

    // Avoid using `f.debug_tuple`, tasks are small enough that they should be printed as atomic terms.
    match self.ty {
      Direct => {
        f.write_fmt(format_args!("{:?}({})", self.ty, self.p.unwrap()))
      },
      _ => {
        // DDepot and ODepot are covered above, so `p` is not `None`.
        f.write_fmt(format_args!("{:?}({},{},{})", self.ty, self.p.unwrap(), self.start, self.end))
      }
    }
  }
}

impl Task {
  fn next_id() -> TaskId {
    use std::sync::atomic::{Ordering, AtomicU32};
    static NEXT: AtomicU32 = AtomicU32::new(0);
    NEXT.fetch_add(1, Ordering::Relaxed)
  }

  pub fn new(ty: TaskType, start: Loc, end: Loc, p: Option<Pv>, t_release: Time, t_deadline: Time, tt: Time) -> Task {
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

// Optimisation(???): we could store Vec<TaskId> instead
#[derive(Debug, Clone)]
pub struct Tasks {
  pub all: Vec<Task>,
  pub compat_with_av: Map<Avg, FnvHashSet<TaskId>>, // FIXME is this correct?
  pub by_id: Map<TaskId, Task>,
  pub by_start: Map<Loc, Vec<TaskId>>,
  pub by_end: Map<Loc, Vec<TaskId>>,
  pub by_cover: Map<Req, Vec<TaskId>>,
  pub by_pv: Map<Pv, FnvHashSet<TaskId>>,
  pub succ: Map<TaskId, Vec<TaskId>>,
  pub pred: Map<TaskId, Vec<TaskId>>,
  pub odepot: Task,
  pub ddepot: Task,
}

impl Tasks {
  pub fn generate(data: &Data, sets: &Sets, pv_req_t_start: &Map<(Pv, Req), Time>) -> Self {
    let _span = error_span!("task generation").entered();

    let mut all = Vec::with_capacity(500);
    let mut by_cover = map_with_capacity(data.n_req);

    // trivial tasks: AV origin and destination depots
    let odepot = Task::new(
      TaskType::ODepot,
      data.odepot,
      data.odepot,
      None,
      0,
      data.tmax,
      0,
    );
    let ddepot = Task::new(
      TaskType::DDepot,
      data.ddepot,
      data.ddepot,
      None,
      0,
      data.tmax,
      0,
    );
    all.push(odepot);
    all.push(ddepot);

    // direct tasks: PV origin to PV destination
    for po in sets.pv_origins() {
      let dp = data.n_passive + po;
      all.push(Task::new(
        TaskType::Direct,
        po,
        dp,
        Some(po),
        data.travel_time[&(data.odepot, po)],
        data.tmax - data.travel_time[&(dp, data.ddepot)],
        data.travel_time[&(po, dp)],
      ))
    }


    // tasks that go from PV origin to Req pickup
    for po in sets.pv_origins() {
      for &rp in &data.compat_passive_req[&po] {
        let rd = rp + data.n_req;
        let t_release = data.travel_time[&(data.odepot, po)];
        let t_deadline = data.end_time[&rd] - data.srv_time[&rp] - data.srv_time[&rd] - data.travel_time[&(rp, rd)];
        let tt = data.travel_time[&(po, rp)];

        // Assumption: if po and rp are marked as compatible, they are time-compatible.  This is enforced with `preprocess::pv_req_timing_compat`
        all.push(Task::new(
          TaskType::PvStart,
          po,
          rp,
          Some(po),
          t_release,
          t_deadline,
          tt,
        ))
      }
    }


    // tasks that go from Req delivery to PV dest
    for po in sets.pv_origins() {
      for &rp in &data.compat_passive_req[&po] {
        let rd = rp + data.n_req;
        let pd = po + data.n_passive;
        let t_release = pv_req_t_start[&(po, rp)] + data.travel_time[&(rp, rd)] + data.srv_time[&rd];
        // let t_deadline = 2*data.tmax; //  FIXME doesn't seems like Michael is taking this tmax into account
        let t_deadline = data.tmax - data.travel_time[&(pd, data.ddepot)];
        let tt = data.travel_time[&(rd, pd)];
        if t_release + tt <= t_deadline {
          all.push(Task::new(
            TaskType::PvEnd,
            rd,
            pd,
            Some(po),
            t_release,
            t_deadline,
            tt,
          ));
        }
      }
    }

    // tasks that go from Req pickup to its delivery
    for po in sets.pv_origins() {
      for &rp in &data.compat_passive_req[&po] {
        let rd = rp + data.n_req;
        let t_release = pv_req_t_start[&(po, rp)];
        let tt = data.travel_time[&(rp, rd)];
        let t_deadline = data.end_time[&rd] - data.srv_time[&rd];
        if t_release + tt <= t_deadline {
          let task = Task::new(
            TaskType::Req,
            rp,
            rd,
            Some(po),
            t_release,
            t_deadline,
            tt,
          );
          all.push(task.clone());
          by_cover.entry(rp).or_insert_with(Vec::new).push(task.id());
        }
      }
    }

    // tasks that go from a Req delivery to another Pickup
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
          let t_deadline = data.end_time[&r2d] - data.srv_time[&r2d] - data.travel_time[&(r2p, r2d)] - data.srv_time[&r2p];
          let t_release = pv_req_t_start[&(po, r1p)] + data.travel_time[&(r1p, r1d)] + data.srv_time[&r1d];

          if t_release + tt <= t_deadline {
            all.push(Task::new(
              TaskType::Transfer,
              r1d,
              r2p,
              Some(po),
              t_release,
              t_deadline,
              tt,
            ));
          }
        }
      }
    }

    all.shrink_to_fit();

    #[cfg(debug_assertions)]
      {
        for t in &all {
          if t.t_release + t.tt > t.t_deadline {
            error!(task=?t, t_release=t.t_release, tt=t.tt, t_deadline=t.t_deadline,
              "BUG: time-impossible task: {} + {} > {}", t.t_release, t.tt, t.t_deadline);
            panic!("critical bug")
          }
        }
      }


    // Lookups
    let mut by_start: Map<Loc, Vec<_>> = map_with_capacity(data.n_loc);
    let mut by_end: Map<Loc, Vec<_>> = map_with_capacity(data.n_loc);
    let mut by_id: Map<_, _> = map_with_capacity(all.len());
    let mut succ: Map<_, _> = map_with_capacity(all.len());
    let mut pred: Map<_, _> = map_with_capacity(all.len());
    let mut by_pv: Map<_, _> = sets.pv_origins().map(|pv| (pv, FnvHashSet::default())).collect();

    for &t in &all {
      by_id.insert(t.id(), t);
      by_start.entry(t.start).or_default().push(t.id());
      by_end.entry(t.end).or_default().push(t.id());
      if let Some(pv) = &t.p {
        by_pv.get_mut(pv).unwrap().insert(t.id());
      }
    }

    for &t1 in &all {
      let t1_succ: Vec<_> = all.iter()
        .filter_map(|t2| match task_incompat(&t1, t2, data) {
          Some(_) => None,
          None => Some(t2.id())
        })
        .collect();

      for &t2 in &t1_succ {
        pred.entry(t2).or_insert_with(Vec::new).push(t1.id());
      }
      succ.insert(t1.id(), t1_succ);
    }

    let mut compat_with_av: Map<_, _> = sets.avs()
      .into_iter()
      .map(|av| {
        let mut s = FnvHashSet::default();
        s.insert(odepot.id());
        s.insert(ddepot.id());
        (av, s)
      }).collect();

    for (&pv, avs) in &data.compat_passive_active {
      for &t in &by_pv[&pv] {
        for &av in avs {
          compat_with_av.get_mut(&av).unwrap().insert(t);
        }
      }
    }

    Tasks { all, by_id, by_pv, by_cover, by_end, by_start, succ, pred, odepot, compat_with_av, ddepot }
  }
}

pub enum Incompatibility {
  /// Time-incompatibility
  Time,
  /// Conflict, possible indirect, in cover constraint
  Cover,
  /// Possibly legal, but optimisation says no
  Opt,
}

fn task_req(t: &Task, n_req: Loc) -> Req {
  use TaskType::*;
  match t.ty {
    Direct | Transfer | ODepot | DDepot => panic!("not valid for direct or transfer tasks"),
    PvStart => t.end,
    PvEnd => t.start - n_req,
    Req => t.start
  }
}

fn pv_schedule(locs: &[Loc], data: &Data) -> Option<Vec<Time>> {
  let mut schedule = Vec::with_capacity(locs.len());
  let mut i = locs[0];
  let mut t = data.travel_time[&(data.odepot, i)];
  schedule.push(t);

  for k in 1..locs.len() {
    let j = locs[k];
    t = data.travel_time[&(i, j)] + t + data.srv_time.get(&j).copied().unwrap_or(0);
    t = std::cmp::max(t, data.start_time.get(&j).copied().unwrap_or(0));
    if t > data.start_time.get(&j).copied().unwrap_or(2500) {
      return None;
    }
    i = j;
    schedule.push(t);
  }

  if t + data.travel_time[&(i, data.ddepot)] > data.tmax {
    return None;
  }

  Some(schedule)
}


/// Returns `None` if it is possible to complete `t2` after task `t1`, using the same active vehicle.
/// Otherwise, returns the reason for the incompatibility.
pub fn task_incompat(t1: &Task, t2: &Task, data: &Data) -> Option<Incompatibility> {
  if !checks::cover(t1, t2, data.n_req) {
    return Some(Incompatibility::Cover);
  }

  if t1.ty == TaskType::ODepot && t2.ty == TaskType::DDepot {
    return None;// TODO return Some(Optimisation)?
  }

  let t = t1.t_release + t1.tt + data.travel_time[&(t1.end, t2.start)] + t2.tt;

  if t > t2.t_deadline {
    return Some(Incompatibility::Time);
  }

  if t1.p == t2.p {
    if t + data.srv_time[&t1.end] + data.srv_time[&t2.start] > t2.t_deadline {
      return Some(Incompatibility::Time);
    }

    if t1.end != t2.start {
      // no point in having some other active vehicle do the implied task inbetween t1 and t2
      return Some(Incompatibility::Opt);
    }
  }

  None
}