use crate::*;
use std::hash::{Hash, Hasher};
use std::ops::Deref;
use serde::{Serialize, Deserialize};
use itertools::max;
use fnv::{FnvHashSet, FnvBuildHasher};
use std::fmt;
use grb::constr::IneqExpr;
use tracing::{error_span, trace, error, warn};
use std::collections::HashMap;
use std::str::FromStr;
use std::cmp::Ordering;

pub type TaskId = u32;

mod checks;
pub mod chain;

mod shorthand;

pub use shorthand::*;

#[derive(Copy, Clone, Debug, Serialize)]
pub struct RawPvTask {
  pub start: RawLoc,
  /// End location
  pub end: RawLoc,
  /// Passive vehicle
  pub p: Pv,
}

impl RawPvTask {
  pub fn new(task: PvTask, lss: &LocSetStarts) -> Option<RawPvTask> {
    // FIXME: what is this even used for
    Some(RawPvTask { start: task.start.encode(lss), end: task.end.encode(lss), p: task.p })
  }
}


#[derive(Copy, Clone, Eq, PartialEq, Hash)]
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
pub struct PvTask {
  id: TaskId,
  pub ty: TaskType,
  /// Start location
  pub start: Loc,
  /// End location
  pub end: Loc,
  /// Passive vehicle, `None` only for active-vehicle depot tasks
  pub p: Pv,
  /// Earliest time the active vehicle can *depart* `start`
  pub t_release: Time,
  /// Latest time the active vehicle can *arrive* at `end`
  pub t_deadline: Time,
  /// Travel time between start and end of this task
  pub tt: Time,
}


impl fmt::Debug for PvTask {
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
      // Avoid using `f.debug_tuple`, tasks are small enough that they should be printed as atomic terms.
      match self.ty {
        Direct => {
          f.write_fmt(format_args!("{:?}({})", self.ty, self.p))
        }
        Start => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.p, self.end.req()))
        }
        End => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.p, self.start.req()))
        }
        Transfer => {
          f.write_fmt(format_args!("{:?}({},{},{})", self.ty, self.p, self.start.req(), self.end.req()))
        }
        Request => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.p, self.start.req()))
        }
        ODepot | DDepot => {
          self.ty.fmt(f)
        }
      }
    }
  }
}

macro_rules! impl_task_shared {

    ($t:ty) => {
      impl PartialEq for $t {
        fn eq(&self, other: &Self) -> bool { self.id() == other.id() }
      }

      impl Eq for $t {}

      impl Ord for $t {
        fn cmp(&self, other: &Self) -> Ordering { self.id().cmp(&other.id()) }
      }

      impl PartialOrd for $t {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(&other)) }
      }

      impl Hash for $t {
        fn hash<H: Hasher>(&self, state: &mut H) {
          state.write_u32(self.id())
        }
      }

      impl LocPair for $t {
        fn ty(&self) -> TaskType { self.ty }
        fn start(&self) -> Loc { self.start }
        fn end(&self) -> Loc { self.end }
      }

      impl $t {
        fn next_id() -> TaskId {
          use std::sync::atomic::{Ordering, AtomicU32};
          static NEXT: AtomicU32 = AtomicU32::new(0);
          NEXT.fetch_add(1, Ordering::Relaxed)
        }

        #[inline(always)]
        pub fn id(&self) -> TaskId { self.id }

        /// Is this task an AV-depot task?
        #[inline(always)]
        pub fn is_depot(&self) -> bool {
          matches!(&self.ty, TaskType::ODepot | TaskType::DDepot)
        }
      }
    };
}

impl_task_shared!(PvTask);

impl PvTask {
  pub fn new(ty: TaskType, start: Loc, end: Loc, p: Pv, t_release: Time, t_deadline: Time, tt: Time) -> PvTask {
    PvTask {
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
}

#[derive(Copy, Clone)]
pub struct Task {
  id: TaskId,
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

impl_task_shared!(Task);

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
      // Avoid using `f.debug_tuple`, tasks are small enough that they should be printed as atomic terms.
      match self.ty {
        Direct => {
          f.write_fmt(format_args!("{:?}({})", self.ty, self.infer_pv().unwrap()))
        }
        Start => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.start.pv(), self.end.req()))
        }
        End => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.end.pv(), self.start.req()))
        }
        Transfer => {
          f.write_fmt(format_args!("{:?}({},{})", self.ty, self.start.req(), self.end.req()))
        }
        Request => {
          f.write_fmt(format_args!("{:?}({})", self.ty, self.start.req()))
        }
        ODepot | DDepot => {
          self.ty.fmt(f)
        }
      }
    }
  }
}

impl Task {
  pub fn new(ty: TaskType, start: Loc, end: Loc, t_release: Time, t_deadline: Time, tt: Time) -> Task {
    Task {
      id: Self::next_id(),
      ty,
      start,
      end,
      t_deadline,
      t_release,
      tt,
    }
  }

  pub fn infer_pv(&self) -> Option<Pv> {
    match self.ty {
      TaskType::ODepot => None,
      TaskType::DDepot => None,
      TaskType::Transfer => None,
      TaskType::Request => None,
      TaskType::Start | TaskType::Direct => Some(self.start.pv()),
      TaskType::End => Some(self.end.pv()),
    }
  }
}

#[derive(Debug)]
pub enum VistedReq {
  None,
  One(Req),
  Transfer(Req, Req)
}

pub trait LocPair {
  fn ty(&self) -> TaskType;
  fn start(&self) -> Loc;
  fn end(&self) -> Loc;

  fn req_visited(&self) -> VistedReq {
    match self.ty() {
      TaskType::Direct | TaskType::DDepot | TaskType::ODepot => VistedReq::None,
      TaskType::Start | TaskType::Request => VistedReq::One(self.end().req()),
      TaskType::End => VistedReq::One(self.start().req()),
      TaskType::Transfer => VistedReq::Transfer(self.start().req(), self.end().req())
    }
  }

}

#[derive(Debug, Clone)]
pub struct Tasks {
  pub all: Vec<Task>,
  pub task_to_pvtasks: Map<Task, Vec<PvTask>>,
  pub pvtask_to_task: Map<PvTask, Task>,
  pub pvtask_to_similar_pvtask: Map<PvTask, Vec<PvTask>>,
  pub pvtask: Map<(Pv, Task), PvTask>,
  // pub pvs_by_task: Map<Task, Vec<Pv>>,
  pub compat_with_av: Map<Avg, Vec<Task>>,
  pub by_start: Map<Loc, Vec<Task>>,
  pub by_end: Map<Loc, Vec<Task>>,
  // pub by_locs: Map<(Loc, Loc), Vec<Task>>,
  pub by_cover: Map<Req, Vec<Task>>,
  // TODO maybe map to PvTasks?
  pub succ: Map<Task, Vec<Task>>,
  pub pred: Map<Task, Vec<Task>>,
  pub pv_succ: Map<PvTask, Vec<PvTask>>,
  pub pv_pred: Map<PvTask, Vec<PvTask>>,
  pub odepot: Task,
  pub ddepot: Task,
  pub by_shorthandpv: Map<ShorthandPvTask, PvTask>,
  pub by_shorthand: Map<ShorthandTask, Task>,
}

// Optimisation(???): we could store Vec<TaskId> instead
#[derive(Debug, Clone)]
struct PvTasks {
  pub all: Vec<PvTask>,
  pub by_start: Map<Loc, Vec<PvTask>>,
  pub by_end: Map<Loc, Vec<PvTask>>,
  pub by_shorthand: Map<ShorthandPvTask, PvTask>,
  pub av_task_conn: Vec<(PvTask, PvTask)>,
  pub pv_task_conn: Vec<(PvTask, PvTask)>,
}


impl PvTasks {
  pub fn generate(data: &Data, sets: &Sets, pv_req_t_start: &Map<(Pv, Req), Time>) -> Self {
    let _span = error_span!("task_gen").entered();

    let mut all = Vec::with_capacity(500); // TODO improve this estimate

    // DIRECT tasks: PV origin to PV destination
    for po in sets.pv_origins() {
      trace!(?po);
      all.push(PvTask::new(
        TaskType::Direct,
        po,
        po.dest(),
        po.pv(),
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
        all.push(PvTask::new(
          TaskType::Start,
          po,
          rp,
          po.pv(),
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
          all.push(PvTask::new(
            TaskType::End,
            rd,
            pd,
            po.pv(),
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
          let task = PvTask::new(
            TaskType::Request,
            rp,
            rd,
            pv,
            t_release,
            t_deadline,
            tt,
          );
          all.push(task);
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
            all.push(PvTask::new(
              TaskType::Transfer,
              r1d,
              r2p,
              pv,
              t_release,
              t_deadline,
              tt,
            ));
          }
        }
      }
    }

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

    for &t in &all {
      by_start.entry(t.start).or_default().push(t);
      by_end.entry(t.end).or_default().push(t);
    }

    let by_shorthand: Map<_, _> = all.iter()
      .copied()
      .map(|t| (ShorthandPvTask::from(t), t))
      .collect();

    let av_task_conn = build_av_task_connections(data, &all);
    let pv_task_conn = build_pv_task_connections(data, &all, &by_start, &by_end, &by_shorthand);

    PvTasks {
      all,
      by_start,
      by_end,
      by_shorthand,
      av_task_conn,
      pv_task_conn,
    }
  }
}


fn build_av_task_connections(data: &Data,
                             all: &Vec<PvTask>) -> Vec<(PvTask, PvTask)> {
  let mut av_task_connections = Vec::with_capacity(all.len() * 3);
  for t1 in all {
    for t2 in all {
      if let Some(incompat) = task_incompat(t1, t2, data) {
        trace!(?t1, ?t2, ?incompat);
      } else {
        av_task_connections.push((*t1, *t2));
      }
    }
  }
  av_task_connections
}

fn build_pv_task_connections(data: &Data,
                             all: &Vec<PvTask>,
                             by_start: &Map<Loc, Vec<PvTask>>,
                             by_end: &Map<Loc, Vec<PvTask>>,
                             by_shorthand: &Map<ShorthandPvTask, PvTask>,
) -> Vec<(PvTask, PvTask)> {
  use ShorthandPvTask::*;
  let mut connections = Vec::with_capacity(all.len());


  for &t1 in all {
    match t1.ty {
      TaskType::ODepot | TaskType::DDepot | TaskType::Direct | TaskType::End => {}
      TaskType::Start => {
        connections.push((t1, by_shorthand[&Request(t1.p, t1.end.req())]));
      }

      TaskType::Transfer => {
        connections.push((t1, by_shorthand[&Request(t1.p, t1.end.req())]));
      }

      TaskType::Request => {
        let p = t1.p;
        let r = t1.start.req();
        let dummy_task = t1;

        let mut path = [by_shorthand[&Start(p, r)], t1, dummy_task, dummy_task, dummy_task];

        for &t2 in by_start[&t1.end].iter().filter(|t2| t2.p == t1.p) {
          match t2.ty {
            TaskType::End => connections.push((t1, t2)),
            TaskType::Transfer => {
              let r_after = t2.end.req();
              path[2] = t2;
              path[3] = by_shorthand[&Request(p, r_after)];
              path[4] = by_shorthand[&End(p, r_after)];

              if schedule::check_pv_route(data, &path) {
                connections.push((t1, t2));
              }
            }
            _ => unreachable!()
          }
        }
      }
    }
  }
  connections
}

fn aggregate_pvtasks(pvtasks: &[PvTask]) -> Map<Task, Vec<PvTask>> {
  let mut pvtasks_grouped = map_with_capacity(pvtasks.len());
  for &pt in pvtasks {
    pvtasks_grouped.entry((pt.start, pt.end)).or_insert_with(Vec::new).push(pt);
  }

  pvtasks_grouped.into_values()
    .map(|group| {
      debug_assert!(group.len() == 1 || matches!(group[0].ty, TaskType::Transfer | TaskType::Request));
      let t_release = group.iter().map(|t| t.t_release).min().unwrap();
      let t_deadline = group.iter().map(|t| t.t_release).max().unwrap();

      let pt = group.first().unwrap();

      let t = Task::new(
        pt.ty,
        pt.start,
        pt.end,
        t_release,
        t_deadline,
        pt.tt,
      );
      (t, group)
    })
    .collect()
}

fn aggregate_similar_pvtasks(pvtasks: &[PvTask]) -> Map<PvTask, Vec<PvTask>> {
  let mut pvtasks_grouped = map_with_capacity(pvtasks.len());
  for &pt in pvtasks {
    pvtasks_grouped.entry((pt.start, pt.end, pt.t_deadline, pt.t_release)).or_insert_with(Vec::new).push(pt);
  }
  pvtasks_grouped.into_values()
    .flat_map(|group|
      group.clone().into_iter().map(move |t| (t, group.clone()))
    )
    .collect()
}


fn av_succ_and_pred(pvtask_to_task: &Map<PvTask, Task>, av_conn: &[(PvTask, PvTask)])
  -> (Map<Task, Vec<Task>>, Map<Task, Vec<Task>>)
{

  let av_conn: Set<_> = av_conn.iter()
    .map(|(pt1, pt2)| (pvtask_to_task[pt1], pvtask_to_task[pt2]) )
    .collect();

  let mut succ : Map<_, Vec<_>> = map_with_capacity(pvtask_to_task.len());
  let mut pred : Map<_, Vec<_>> = map_with_capacity(pvtask_to_task.len());

  for (t1, t2) in av_conn {
    succ.entry(t1).or_default().push(t2);
    pred.entry(t2).or_default().push(t1);
  }

  (succ, pred)
}


impl Tasks {
  pub fn generate(data: &Data, sets: &Sets, pv_req_t_start: &Map<(Pv, Req), Time>) -> Self {
    let pvtasks = PvTasks::generate(data, sets, pv_req_t_start);
    Self::build(data, sets, pvtasks)
  }

  fn build(data: &Data, sets: &Sets, pvtasks: PvTasks) -> Self {
    // trivial tasks: AV origin and destination depots
    let odepot = Task::new(
      TaskType::ODepot,
      Loc::Ao,
      Loc::Ao,
      0,
      data.tmax,
      0,
    );
    let ddepot = Task::new(
      TaskType::DDepot,
      Loc::Ad,
      Loc::Ad,
      0,
      data.tmax,
      0,
    );

    let depots = &[odepot, ddepot];

    let task_to_pvtasks = aggregate_pvtasks(&pvtasks.all);
    let pvtask_to_task : Map<_, _> = task_to_pvtasks.iter()
      .flat_map(|(&t, pts)| pts.iter().map(move |&pt| (pt, t)))
      .collect();
    let pvtask : Map<_, _> = pvtask_to_task.iter()
      .map(|(&pt, &t)| ((pt.p, t), pt))
      .collect();
    let all: Vec<_> = task_to_pvtasks.keys().chain(depots).copied().collect();
    let all_non_depot = &all[..all.len()-2];
    let pvtask_to_similar_pvtask = aggregate_similar_pvtasks(&pvtasks.all);

    let compat_with_av: Map<_, _> = sets.avs()
      .map(|av| {
        let pvs = &data.compat_active_passive[&av];
        let compat_passive_vehicles : Set<_> = pvs.iter().copied().collect();

        let compat_tasks : Vec<_> = task_to_pvtasks.iter()
          .filter_map(|(t, pvs)|
            if pvs.iter().any(|pt| compat_passive_vehicles.contains(&pt.p)) {
              Some(*t)
            }  else {
              None
            })
          .chain(depots.iter().copied())
          .collect();
        (av, compat_tasks)
      })
      .collect();


    let mut by_start: Map<_, Vec<_>> = map_with_capacity(data.n_loc as usize);
    let mut by_end: Map<_, Vec<_>> = map_with_capacity(data.n_loc as usize);
    let mut by_cover: Map<_, Vec<_>> = map_with_capacity(data.n_req as usize);

    for &t in &all {
      by_end.entry(t.end).or_default().push(t);
      by_start.entry(t.start).or_default().push(t);

      match t.req_visited() {
        VistedReq::None => {},
        VistedReq::One(r) => {
          by_cover.entry(r).or_default().push(t);
        }
        VistedReq::Transfer(r1, r2) => {
          by_cover.entry(r1).or_default().push(t);
          by_cover.entry(r2).or_default().push(t);
        }
      }
    }

    let (mut succ, mut pred) = av_succ_and_pred(&pvtask_to_task, &pvtasks.av_task_conn);
    debug_assert_eq!(all[all.len()-2], odepot);
    debug_assert_eq!(all[all.len()-1], ddepot);
    {
      for &t in all_non_depot {
        succ.entry(t).or_default().push(ddepot);
        pred.entry(t).or_default().push(odepot);
      }

      succ.insert(odepot, all_non_depot.to_vec());
      succ.insert(ddepot, Vec::new());

      pred.insert(odepot, Vec::new());
      pred.insert(ddepot, all_non_depot.to_vec());
    }

    let mut pv_succ : Map<_, Vec<_>> = map_with_capacity(pvtasks.all.len());
    let mut pv_pred : Map<_, Vec<_>> = map_with_capacity(pvtasks.all.len());
    for &(t1, t2) in &pvtasks.pv_task_conn {
      pv_succ.entry(t1).or_default().push(t2);
      pv_pred.entry(t2).or_default().push(t1);
    }

    let by_shorthand: Map<_, _> = all.iter()
      .map(|&t| (ShorthandTask::from(t), t))
      .collect();

    Tasks {
      all,
      task_to_pvtasks,
      pvtask_to_task,
      pvtask_to_similar_pvtask,
      pvtask,
      compat_with_av,
      by_start,
      by_end,
      by_cover,
      succ,
      pred,
      pv_succ,
      pv_pred,
      odepot,
      ddepot,
      by_shorthand,
      by_shorthandpv: pvtasks.by_shorthand
    }
  }
}

#[derive(Copy, Clone, Debug)]
pub enum Incompatibility {
  /// Time-incompatibility
  Time,
  /// Conflict, possible indirect, in cover constraint
  Cover,
  /// Possibly legal, but optimisation says no
  Optimisation,
}

fn task_req(t: &PvTask) -> Req {
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
pub fn task_incompat(t1: &PvTask, t2: &PvTask, data: &Data) -> Option<Incompatibility> {
  if !checks::cover(t1, t2) {
    return Some(Incompatibility::Cover);
  }

  if t1.ty == TaskType::ODepot && t2.ty == TaskType::DDepot {
    return Some(Incompatibility::Optimisation); // avoid trivial AV routes
  }

  let t = t1.t_release + t1.tt + data.travel_time[&(t1.end, t2.start)] + t2.tt;

  if t > t2.t_deadline {
    return Some(Incompatibility::Time);
  }

  if t1.p == t2.p {
    if t1.end != t2.start {
      // no point in having some other active vehicle do the implied task between t1 and t2
      return Some(Incompatibility::Optimisation);
    }

    // t1.end == t2.start, so only add one service time. ss
    if t + data.srv_time[&t1.end] > t2.t_deadline {
      return Some(Incompatibility::Time);
    }
  }

  None
}

//
// pub fn update_implied_cover_before(visited: &mut FnvHashSet<Loc>, task: &PvTask) -> bool {
//   use Loc::*;
//   use TaskType::*;
//
//   let cover_violation = match task.ty {
//     ODepot | DDepot => false,
//
//     Start | Direct =>
//       visited.insert(task.start)
//         && visited.insert(task.end),
//
//     End =>
//       visited.insert(task.end.origin())
//         && visited.insert(task.start.origin())
//         && visited.insert(task.start)
//         && visited.insert(task.end),
//
//     Transfer =>
//       visited.insert(Po(task.p.unwrap()))
//         && visited.insert(task.start.origin())
//         && visited.insert(task.start)
//         && visited.insert(task.end),
//
//     Request =>
//       visited.insert(Po(task.p.unwrap()))
//         && visited.insert(task.start)
//         && visited.insert(task.end),
//   };
//
//   !cover_violation
// }
//
//
// pub fn update_implied_cover_after(visited: &mut FnvHashSet<Loc>, task: &PvTask) -> bool {
//   use Loc::*;
//   use TaskType::*;
//
//   let cover_violation = match task.ty {
//     ODepot | DDepot => false,
//
//     Start =>
//       visited.insert(task.start)
//         && visited.insert(task.end)
//         && visited.insert(task.end.dest())
//         && visited.insert(task.start.dest()),
//
//     End | Direct =>
//       visited.insert(task.start)
//         && visited.insert(task.end),
//
//     Transfer =>
//       visited.insert(task.start)
//         && visited.insert(task.end)
//         && visited.insert(task.end.dest())
//         && visited.insert(Pd(task.p.unwrap())),
//
//     Request =>
//       visited.insert(task.start)
//         && visited.insert(task.end)
//         && visited.insert(Pd(task.p.unwrap())),
//   };
//
//   !cover_violation
// }

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

