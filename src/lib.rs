#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_imports)]
pub use instances::dataset::apvrp::*;
pub use fnv::FnvHashMap as Map;
pub use anyhow::Result;

use itertools::Itertools;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Task {
  /// Start location
  pub start: Loc,
  /// End location
  pub end: Loc,
  /// Passive vehicle
  pub p: Pv,
  /// Earliest time the passive vehicle can leave `start`
  pub t_release: Time,
  /// Latest time the passive vehicle can arrive at `end`
  pub t_deadline: Time,
  /// Travel time between start and end of this task
  pub tt: Time,
}

#[derive(Default, Debug, Clone)]
pub struct Tasks {
  pub all: Vec<Task>
}

impl Tasks {
  pub fn generate(data: &ApvrpInstance, sets: &Sets, pv_req_t_start: &Map<(Pv, Req), Time>) -> Self {
    let mut all = Vec::with_capacity(500);

    // trivial tasks: PV origin to PV destination
    for &op in &sets.pv_o {
      let dp = data.n_passive + op;
      all.push(Task {
        start: op,
        end: dp,
        p: op,
        t_release: data.travel_time[&(sets.depot, op)],
        t_deadline: data.tmax - data.travel_time[&(dp, sets.depot)],
        tt: data.travel_time[&(op, dp)],
      })
    }


    // # tasks that go from PV origin to Req pickup
    for &op in &sets.pv_o {
      for &rp in &data.compat_passive_req[&op] {
        let rd = rp + data.n_req;
        all.push(Task {
          start: op,
          end: rp,
          p: op,
          t_release: data.travel_time[&(sets.depot, op)],
          t_deadline: data.end_time[&rd] - data.srv_time[&rp] - data.srv_time[&rd] - data.travel_time[&(rp, rd)],
          tt: data.travel_time[&(op, rp)],
        })
      }
    }



    // # tasks that go from Req delivery to PV dest
    for &po in &sets.pv_o {
      for &rp in &data.compat_passive_req[&po] {
        let rd = rp + data.n_req;
        let pd = po + data.n_passive;
        let av_dur = data.travel_time[&(rd, pd)];

        all.push(Task {
          start: rd,
          end: pd,
          p: po,
          t_release: pv_req_t_start[&(po, rp)] + data.srv_time[&rd] + data.travel_time[&(rp, rd)],
          t_deadline: data.tmax - data.travel_time[&(pd, sets.depot)],
          tt: data.travel_time[&(rd, pd)],
        })
      }
    }

    // # tasks that go from Req pickup to its delivery
    for &po in &sets.pv_o {
      for &rp in &data.compat_passive_req[&po] {
        let rd = rp + data.n_req;

        all.push(Task {
          start: rp,
          end: rd,
          p: po,
          t_release: pv_req_t_start[&(po, rp)],
          t_deadline: data.end_time[&rd] - data.srv_time[&rd],
          tt: data.travel_time[&(rp, rd)],
        })
      }
    }

    // # tasks that go from a Req delivery to another Pickup
    for &po in &sets.pv_o {
      let requests = &data.compat_passive_req[&po];
      for &r1p in requests {
        let r1d = r1p + data.n_req;

        for &r2p in requests {
          if r1p == r2p {
            continue
          }
          let r2d = r1p + data.n_req;
          let tt = data.travel_time[&(r1d, r2p)];
          let t_deadline = data.end_time[&r2d] - data.srv_time[&r2d] - data.srv_time[&r2p] -data.travel_time[&(r2p, r2d)];
          let t_release = pv_req_t_start[&(po, r1p)] + data.srv_time[&r2d] + data.travel_time[&(r1p,r1d)];

          if t_release + tt > t_deadline {
            continue
          }

          all.push(Task {
            start: r1d,
            end: r2p,
            p: po,
            t_release,
            t_deadline,
            tt,
          })
        }
      }
    }



    all.shrink_to_fit();

    Tasks{ all }
  }
}


#[derive(Debug, Clone)]
pub struct Sets {
  pub depot: Loc,
  /// Set of locations
  pub locs: Vec<Loc>,
  /// Set of passive vehicle locations
  pub pv: Vec<Pv>,
  /// Set of passive vehicles/passive vehicle origin locations
  pub pv_o: Vec<Pv>,
  /// Set of passive vehicle dest. locations
  pub pv_d: Vec<Pv>,
  /// Set of request locations
  pub r: Vec<Req>,
  /// Set of requests/request origin locations
  pub r_o: Vec<Req>,
  /// Set of request dest. locations
  pub r_d: Vec<Req>,
  /// Set of active vehicles
  pub av: Vec<Av>,
}

impl Sets {
  pub fn new(data: &ApvrpInstance) -> Self {
    let pv_o_srt = 1;
    let pv_o_end = data.n_passive;
    let pv_d_end = pv_o_end + data.n_passive;
    let r_o_end = pv_d_end + data.n_req;
    let r_d_end = r_o_end + data.n_req;

    Sets {
      depot: 0,
      locs: (0..=r_d_end).collect(),
      pv: (pv_o_srt..=pv_d_end).collect(),
      pv_o: (pv_o_srt..=pv_o_end).collect(),
      pv_d: (pv_o_end+1..=pv_d_end).collect(),
      r: (pv_d_end+1..=r_d_end).collect(),
      r_o: (pv_d_end+1..=r_o_end).collect(),
      r_d: (r_o_end+1..=r_d_end).collect(),
      av: (0..data.n_active).collect(),
    }
  }
}


