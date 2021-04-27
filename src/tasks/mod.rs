use crate::*;
use std::hash::{Hash, Hasher};
use std::ops::Deref;
use serde::{Serialize};
use itertools::max;
use fnv::{FnvHashSet, FnvBuildHasher};
use std::fmt;
use grb::constr::IneqExpr;
use tracing::{error_span, trace, error, warn};
use std::collections::HashMap;

pub type TaskId = u32;
pub type PVIRTaskId = u32;

mod checks;
pub mod chain;


#[derive(Copy, Clone, Debug, Serialize)]
pub struct RawPvTask {
  pub start: RawLoc,
  /// End location
  pub end: RawLoc,
  /// Passive vehicle
  pub p: Pv,
}

impl RawPvTask {
  pub fn new(task: Task, lss: &LocSetStarts) -> Option<RawPvTask> {
    task.p.map(|p| RawPvTask { start: task.start.encode(lss), end: task.end.encode(lss), p })
  }
}


#[derive(Copy, Clone, Eq, PartialEq)]
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
  Request,
  /// Av origin depot (fake task)
  ODepot,
  /// Av destination depot (fake task)
  DDepot,
}

impl fmt::Debug for TaskType {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    use TaskType::*;
    let s = match self {
      Transfer => "Trn",
      Start => "Srt",
      End => "End",
      Direct => "Dir",
      Request => "Req",
      ODepot => "ODp",
      DDepot => "DDp"
    };
    f.write_str(s)
  }
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

    if f.alternate() {
      f.debug_struct("Task")
        .field("short_name", &format_args!("{:?}", &self))
        .field("t_release", &self.t_release)
        .field("tt", &self.tt)
        .field("t_deadline", &self.t_deadline)
        .finish()
    } else {
      match self.ty {
        DDepot | ODepot => { return self.ty.fmt(f); }
        _ => {}
      }

      // Avoid using `f.debug_tuple`, tasks are small enough that they should be printed as atomic terms.
      match self.ty {
        Direct => {
          f.write_fmt(format_args!("{:?}({})", self.ty, self.p.unwrap()))
        }
        Start => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.start.pv(), self.end.req()))
        }
        End => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.start.req(), self.end.pv()))
        }
        Transfer => {
          f.write_fmt(format_args!("{:?}({},{},{})", self.ty, self.p.unwrap(), self.start.req(), self.end.req()))
        }
        Request => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.p.unwrap(), self.start.req()))
        }
        ODepot | DDepot => {
          self.ty.fmt(f)
        }
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

  /// Is this task an AV-depot task?
  #[inline(always)]
  pub fn is_depot(&self) -> bool {
    self.ty == TaskType::ODepot || self.ty == TaskType::DDepot
  }
}

// Optimisation(???): we could store Vec<TaskId> instead
#[derive(Debug, Clone)]
pub struct Tasks {
  pub all: Vec<Task>,
  pub compat_with_av: Map<Avg, FnvHashSet<Task>>,
  // FIXME is this correct?
  pub by_id: Map<TaskId, Task>,
  pub by_start: Map<Loc, Vec<Task>>,
  pub by_end: Map<Loc, Vec<Task>>,
  pub by_locs: Map<(Loc, Loc), Vec<Task>>,
  pub by_cover: Map<Req, Vec<Task>>,
  pub by_pv: Map<Pv, FnvHashSet<Task>>,
  pub similar_tasks: Map<Task, Vec<Task>>,
  pub succ: Map<Task, Vec<Task>>,
  pub pred: Map<Task, Vec<Task>>,
  pub odepot: Task,
  pub ddepot: Task,
}


impl Tasks {
  pub fn generate(data: &Data, sets: &Sets, pv_req_t_start: &Map<(Pv, Req), Time>) -> Self {
    let _span = error_span!("task generation").entered();

    let mut all = Vec::with_capacity(500); // TODO improve this estimate
    let mut by_cover = map_with_capacity(data.n_req as usize);

    // trivial tasks: AV origin and destination depots
    let odepot = Task::new(
      TaskType::ODepot,
      Loc::Ao,
      Loc::Ao,
      None,
      0,
      data.tmax,
      0,
    );
    let ddepot = Task::new(
      TaskType::DDepot,
      Loc::Ad,
      Loc::Ad,
      None,
      0,
      data.tmax,
      0,
    );
    all.push(odepot);
    all.push(ddepot);

    // DIRECT tasks: PV origin to PV destination
    for po in sets.pv_origins() {
      trace!(?po);
      all.push(Task::new(
        TaskType::Direct,
        po,
        po.dest(),
        Some(po.pv()),
        data.travel_time[&(Loc::Ao, po)],
        data.tmax - data.travel_time[&(po.dest(), Loc::Ad)],
        data.travel_time[&(po, po.dest())],
      ))
    }

    // START tasks: go from PV origin to Req pickup
    for po in sets.pv_origins() {
      for &r in &data.compat_passive_req[&po.pv()] {
        let rp = Loc::ReqP(r);
        let t_release = data.travel_time[&(Loc::Ao, po)];
        let t_deadline = data.end_time[&rp.dest()] - data.srv_time[&rp] - data.srv_time[&rp.dest()] - data.travel_time[&(rp, rp.dest())];
        let tt = data.travel_time[&(po, rp)];

        // Assumption: if po and rp are marked as compatible, they are time-compatible.
        //  This is enforced with `preprocess::pv_req_timing_compat`
        all.push(Task::new(
          TaskType::Start,
          po,
          rp,
          Some(po.pv()),
          t_release,
          t_deadline,
          tt,
        ))
      }
    }

    // END tasks: go from Req delivery to PV dest
    for po in sets.pv_origins() {
      for rp in data.compat_passive_req[&po.pv()].iter().copied().map(Loc::ReqP) {
        let rd = rp.dest();
        let pd = po.dest();
        let t_release = pv_req_t_start[&(po.pv(), rp.req())] + data.travel_time[&(rp, rd)] + data.srv_time[&rd];
        // let t_deadline = 2*data.tmax; //  FIXME doesn't seems like Michael is taking this tmax into account
        let t_deadline = data.tmax - data.travel_time[&(pd, Loc::Ad)];
        let tt = data.travel_time[&(rd, pd)];
        if t_release + tt <= t_deadline {
          all.push(Task::new(
            TaskType::End,
            rd,
            pd,
            Some(po.pv()),
            t_release,
            t_deadline,
            tt,
          ));
        }
      }
    }

    // REQUEST tasks: go from Req pickup to its delivery
    for pv in sets.pvs() {
      for rp in data.compat_passive_req[&pv].iter().copied().map(Loc::ReqP) {
        let rd = rp.dest();
        let t_release = pv_req_t_start[&(pv, rp.req())];
        let tt = data.travel_time[&(rp, rd)];
        let t_deadline = data.end_time[&rd] - data.srv_time[&rd];
        trace!(pv, ?rp, t_release, tt, t_deadline);
        if t_release + tt <= t_deadline {
          let task = Task::new(
            TaskType::Request,
            rp,
            rd,
            Some(pv),
            t_release,
            t_deadline,
            tt,
          );
          all.push(task);
          by_cover.entry(rp.req()).or_insert_with(Vec::new).push(task);
        }
      }
    }

    // TRANSFER tasks: go from a Req delivery to another Pickup
    for pv in sets.pvs() {
      let requests = &data.compat_passive_req[&pv];
      for r1p in requests.iter().copied().map(Loc::ReqP) {
        let r1d = r1p.dest();

        for r2p in requests.iter().copied().map(Loc::ReqP) {
          if r1p == r2p {
            continue;
          }
          let r2d = r2p.dest();
          let tt = data.travel_time[&(r1d, r2p)];
          let t_deadline = data.end_time[&r2d] - data.srv_time[&r2d] - data.travel_time[&(r2p, r2d)] - data.srv_time[&r2p];
          let t_release = pv_req_t_start[&(pv, r1p.req())] + data.travel_time[&(r1p, r1d)] + data.srv_time[&r1d];

          if t_release + tt <= t_deadline {
            all.push(Task::new(
              TaskType::Transfer,
              r1d,
              r2p,
              Some(pv),
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
    let mut by_start: Map<Loc, Vec<_>> = map_with_capacity(data.n_loc as usize);
    let mut by_end: Map<Loc, Vec<_>> = map_with_capacity(data.n_loc as usize);
    let mut by_locs: Map<(Loc, Loc), Vec<_>> = map_with_capacity(data.travel_time.len());
    let mut by_id: Map<_, _> = map_with_capacity(all.len());
    let mut succ: Map<_, _> = map_with_capacity(all.len());
    let mut pred: Map<_, _> = map_with_capacity(all.len());
    let mut by_pv: Map<_, _> = sets.pvs().map(|pv| (pv, FnvHashSet::default())).collect();
    let mut similar_tasks: Map<_, Vec<_>> = map_with_capacity(all.len());

    for &t in &all {
      by_id.insert(t.id(), t);
      by_start.entry(t.start).or_default().push(t);
      by_end.entry(t.end).or_default().push(t);
      by_locs.entry((t.start, t.end)).or_default().push(t);
      if let Some(pv) = &t.p {
        by_pv.get_mut(pv).unwrap().insert(t);
      }
    }

    for &t1 in &all {
      let t1_succ: Vec<_> = all.iter()
        .filter(|t2| {
          let incompat = task_incompat(&t1, t2, data);
          trace!(?t1, ?t2, ?incompat);
          incompat.is_none()
        })
        .copied()
        .collect();

      for &t2 in &t1_succ {
        pred.entry(t2).or_insert_with(Vec::new).push(t1);
      }
      if t1.is_depot() {
        trace!(?t1, t1_succ=?&t1_succ);
      }
      succ.insert(t1, t1_succ);
    }

    let mut compat_with_av: Map<_, _> = sets.avs()
      .into_iter()
      .map(|av| {
        let mut s = FnvHashSet::default();
        s.insert(odepot);
        s.insert(ddepot);
        (av, s)
      }).collect();

    for (&pv, avs) in &data.compat_passive_active {
      for &t in &by_pv[&pv] {
        for &av in avs {
          compat_with_av.get_mut(&av).unwrap().insert(t);
        }
      }
    }

    for &t1 in &all {
      if t1.ty == TaskType::Request || t1.ty == TaskType::Transfer {
        let candidates = &by_start[&t1.start];
        let similar = similar_tasks.entry(t1).or_insert(Vec::with_capacity(candidates.len()));
        for &t2 in candidates {
          if t1.end == t2.end {
            if t1.t_release == t2.t_release && t1.t_deadline == t2.t_deadline {
              similar.push(t2);
            } else {
              warn!(?t1, ?t2, t1.t_release, t2.t_release, t1.t_deadline, t2.t_deadline,
                "interesting timing tasks: similar tasks with different time values");
            }
          }
        }
        similar.shrink_to_fit();
      } else {
        similar_tasks.insert(t1, vec![t1]);
      }
    }

    Tasks { all, by_id, by_pv, by_cover, by_locs, by_end, by_start, similar_tasks, succ, pred, odepot, compat_with_av, ddepot }
  }
}

#[derive(Copy, Clone, Debug)]
pub enum Incompatibility {
  /// Time-incompatibility
  Time,
  /// Conflict, possible indirect, in cover constraint
  Cover,
  /// Possibly legal, but optimisation says no
  Opt,
}

fn task_req(t: &Task) -> Req {
  use TaskType::*;
  match t.ty {
    Direct | Transfer | ODepot | DDepot => panic!("not valid for direct, AV depot or transfer tasks"),
    Start => t.end.req(),
    End => t.start.req(),
    Request => t.start.req()
  }
}


/// Returns `None` if it is possible to complete `t2` after task `t1`, using the same active vehicle.
/// Otherwise, returns the reason for the incompatibility.
pub fn task_incompat(t1: &Task, t2: &Task, data: &Data) -> Option<Incompatibility> {
  if !checks::cover(t1, t2) {
    return Some(Incompatibility::Cover);
  }

  if t1.ty == TaskType::ODepot && t2.ty == TaskType::DDepot {
    return Some(Incompatibility::Opt); // necessary to avoid trivial AV routes
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


pub fn update_implied_cover_before(visited: &mut FnvHashSet<Loc>, task: &Task) -> bool {
  use Loc::*;
  use TaskType::*;

  let cover_violation = match task.ty {
    ODepot | DDepot => false,

    Start | Direct =>
      visited.insert(task.start)
        && visited.insert(task.end),

    End =>
      visited.insert(task.end.origin())
        && visited.insert(task.start.origin())
        && visited.insert(task.start)
        && visited.insert(task.end),

    Transfer =>
      visited.insert(Po(task.p.unwrap()))
        && visited.insert(task.start.origin())
        && visited.insert(task.start)
        && visited.insert(task.end),

    Request =>
      visited.insert(Po(task.p.unwrap()))
        && visited.insert(task.start)
        && visited.insert(task.end),
  };

  !cover_violation
}




pub fn update_implied_cover_after(visited: &mut FnvHashSet<Loc>, task: &Task) -> bool {
  use Loc::*;
  use TaskType::*;

  let cover_violation = match task.ty {
    ODepot | DDepot => false,

    Start =>
      visited.insert(task.start)
        && visited.insert(task.end)
        && visited.insert(task.end.dest())
        && visited.insert(task.start.dest()),

    End | Direct =>
      visited.insert(task.start)
        && visited.insert(task.end),

    Transfer =>
      visited.insert(task.start)
        && visited.insert(task.end)
        && visited.insert(task.end.dest())
        && visited.insert(Pd(task.p.unwrap())),

    Request =>
      visited.insert(task.start)
        && visited.insert(task.end)
        && visited.insert(Pd(task.p.unwrap())),
  };

  !cover_violation
}

// FIXME: I don't think this works with the similar-task lifting because the Req-PV pairing for tasks
//  will change.
// /// Update the Request-Passive vehicle pairings.  If a conflict occurs, the pairing is not
// /// modified and `false` is returned.  Otherwise, the pairings are updated and `true` is returned.
// pub fn update_req_pv_pairing(pairing: &mut HashMap<Req, Pv>, task: &Task) -> bool {
//   use Loc::*;
//   use TaskType::*;
//   use std::collections::hash_map::Entry;
//
//   #[inline]
//   fn update_one(pairing: &mut HashMap<Req, Pv>, r: Req, pv: Pv) -> bool {
//     match pairing.entry(r) {
//       Entry::Occupied(e) => e.get() == &pv,
//       Entry::Vacant(e) => {
//         e.insert(pv);
//         true
//       }
//     }
//   }
//
//   match task.ty {
//     ODepot | DDepot | Direct => true,
//     Start => update_one(pairing, task.end.req(), task.p.unwrap()),
//     End | Request => update_one(pairing, task.start.req(), task.p.unwrap()),
//     Transfer => update_one(pairing, task.start.req(), task.p.unwrap())
//       && update_one(pairing, task.end.req(), task.p.unwrap()),
//   }
// }

