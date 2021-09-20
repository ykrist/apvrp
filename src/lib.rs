#![feature(generic_associated_types)]
#![feature(fn_traits)]
#![feature(assert_matches)]
#![allow(dead_code)]
#![allow(unused_imports)]
#![deny(unused_must_use)]
pub use fnv::{FnvHashMap as Map, FnvHashSet as Set};
pub use anyhow::Result;

pub(crate) use std::assert_matches::*;

use itertools::Itertools;
use std::fmt;

use instances::dataset::apvrp::{
  MEISEL_A,
  TILK_AB,
  DSET,
  LocSetStarts,
};
pub use instances::dataset::apvrp::{
  Av, Pv, Req, Time, Cost, Loc as RawLoc,
  ApvrpInstance,
};
pub use instances::dataset::Dataset;

/// Active Vehicle Group.
pub type Avg = Av;

#[derive(Debug, Clone)]
pub struct Data {
  pub id: String,
  pub n_req: RawLoc,
  pub n_passive: RawLoc,
  pub n_active: RawLoc,
  pub n_loc: RawLoc,
  pub tmax: Time,
  pub srv_time: Map<Loc, Time>,
  /// Earliest time we can leave Loc
  pub start_time: Map<Loc, Time>,
  /// Latest time we can finish service at Loc
  pub end_time: Map<Loc, Time>,
  pub compat_req_passive: Map<Req, Vec<Pv>>,
  pub compat_passive_req: Map<Pv, Vec<Req>>,
  pub compat_passive_active: Map<Pv, Vec<Avg>>,
  pub compat_active_passive: Map<Avg, Vec<Pv>>,
  /// The earliest time a PV may *leave* the Pickup of request
  pub pv_req_start_time: Map<(Pv, Req), Time>,
  pub travel_cost: Map<(Loc, Loc), Cost>,
  pub travel_time: Map<(Loc, Loc), Time>,
  pub av_groups: Map<Avg, Vec<Av>>,
}

impl Data {
  pub fn travel_time_to_ddepot(&self, t: &Task) -> Time {
    debug_assert!(!t.is_depot());
    t.tt + self.travel_time[&(t.end, Loc::Ad)]
  }
}


pub fn map_with_capacity<K,V>(capacity: usize) -> Map<K,V> {
  Map::with_capacity_and_hasher(capacity, fnv::FnvBuildHasher::default())
}


pub fn set_with_capacity<T>(capacity: usize) -> Set<T> {
  FnvHashSet::with_capacity_and_hasher(capacity, fnv::FnvBuildHasher::default())
}


#[derive(Hash, Copy, Clone, Eq, PartialEq)]
pub enum Loc {
  /// Passive vehicle origin (p)
  Po(u16),
  /// Passive vehicle destination (p)
  Pd(u16),
  /// Request pickup (r)
  ReqP(u16),
  /// Request delivery (r)
  ReqD(u16),
  /// AV origin depot
  Ao,
  /// AV dest depot
  Ad,
}

impl fmt::Debug for Loc {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Loc::Ao => f.write_str("Ao"),
      Loc::Ad => f.write_str("Ad"),
      Loc::Po(p) => f.write_fmt(format_args!("Po({})", p)),
      Loc::Pd(p) => f.write_fmt(format_args!("Pd({})", p)),
      Loc::ReqP(r) => f.write_fmt(format_args!("Rp({})", r)),
      Loc::ReqD(r) => f.write_fmt(format_args!("Rd({})", r)),
    }
  }
}

impl fmt::Display for Loc {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    fmt::Debug::fmt(self, f)
  }
}

impl Loc {
  /// Return the "destination" for Request pickups and origin depots
  pub fn dest(&self) -> Loc {
    match self {
      Loc::ReqP(r) => Loc::ReqD(*r),
      Loc::Ao => Loc::Ad,
      Loc::Po(p) => Loc::Pd(*p),
      _ => unimplemented!("only defined for request pickups, AV- and PV origins")
    }
  }

  /// Return the "origin" for Request deliverys and dest depots
  pub fn origin(&self) -> Loc {
    match self {
      Loc::ReqD(r) => Loc::ReqD(*r),
      Loc::Ad => Loc::Ao,
      Loc::Pd(p) => Loc::Po(*p),
      _ => unimplemented!("only defined for request deliveries, AV- and PV destinations")
    }
  }

  pub fn req(&self) -> Req {
    match self {
      Loc::ReqD(r) | Loc::ReqP(r) => *r,
      _ => unimplemented!("only defined for request pickups and deliveries")
    }
  }

  pub fn pv(&self) -> Pv {
    match self {
      Loc::Po(p) | Loc::Pd(p) => *p,
      _ => unimplemented!("only defined for passive vehicle origins and destinations")
    }
  }
}

pub trait LocSetStartsExt {
  fn decode(&self, loc: RawLoc) -> Loc;
  fn encode(&self, loc: Loc) -> RawLoc;
}

impl LocSetStartsExt for LocSetStarts {
  fn decode(&self, loc: RawLoc) -> Loc {
    if loc == self.avo {
      Loc::Ao
    } else if loc < self.pv_d {
      Loc::Po(loc - self.pv_o)
    } else if loc < self.req_p {
      Loc::Pd(loc - self.pv_d)
    } else if loc < self.req_d {
      Loc::ReqP(loc - self.req_p)
    } else if loc < self.avd {
      Loc::ReqD(loc - self.req_d)
    } else if loc == self.avd {
      Loc::Ad
    } else {
      panic!("{} is out of range (max {})", loc, self.avd)
    }
  }

  fn encode(&self, loc: Loc) -> RawLoc {
    match loc {
      Loc::Ao => self.avo,
      Loc::Ad => self.avd,
      Loc::Po(p) => self.pv_o + p,
      Loc::Pd(p) => self.pv_d + p,
      Loc::ReqP(r) => self.req_p + r,
      Loc::ReqD(r) => self.req_d + r,
    }
  }
}

#[derive(Debug, Clone)]
pub struct Lookups {
  pub data: Data,
  pub sets: Sets,
  pub tasks: Tasks,
  pub pv_routes: Option<PvRoutes>
}

impl Lookups {
  pub fn load_data_and_build(data_index: usize) -> Result<Self> {
    let data = DSET
      .load_instance(data_index)
      .map(preprocess::full_pipeline)?;

    let sets = Sets::new(&data);
    let tasks = Tasks::generate(&data, &sets);

    info!(num_tasks = tasks.all.len(), "task generation finished");
    Ok(Lookups { data, sets, tasks, pv_routes: None })
  }

  pub fn generate_pv_routes(&mut self) {
    if self.pv_routes.is_some() {
      panic!("Routes have already been generated.")
    }
    self.pv_routes = Some(PvRoutes::new(&self.sets, &self.data, &self.tasks))
  }

}

impl AsRef<Data> for Lookups {
  fn as_ref(&self) -> &Data {
    &self.data
  }
}

impl AsRef<Sets> for Lookups {
  fn as_ref(&self) -> &Sets {
    &self.sets
  }
}

impl AsRef<Tasks> for Lookups {
  fn as_ref(&self) -> &Tasks {
    &self.tasks
  }
}

impl AsRef<Data> for Data {
  fn as_ref(&self) -> &Data { self }
}



mod tasks;
pub use tasks::*;

use logging::*;

mod sets;
pub use sets::Sets;

mod utils;
pub use utils::{iter_cycle, Json};

mod constants;
pub use constants::*;


pub mod model;
pub mod preprocess;
pub mod graph;
pub mod logging;
pub mod solution;
use fnv::FnvHashSet;
use crate::colgen::PvRoutes;
use anyhow::Context;

pub mod schedule;
pub mod experiment;
pub mod test;
pub mod colgen;
// TODO tests for encode and decode.

pub const COMMIT_HASH : &'static str = env!("COMMIT_HASH");


fn commit_hash() -> Result<String> {
  let output = std::process::Command::new("git").args(["rev-parse", "HEAD"]).output()?;
  assert!(output.status.success());
  Ok(std::str::from_utf8(&output.stdout)?.trim().to_string())
}

pub fn check_commit_hash() -> Result<()> {
  let hash = commit_hash().context("Unable to retrieve commit hash")?;
  if hash != COMMIT_HASH {
    let msg = "build is out of date";
    tracing::error!(build_commit=%COMMIT_HASH, current_commit=%hash, "{}", msg);
    anyhow::bail!("{}", msg);
  }
  tracing::info!(git_commit=%hash);
  Ok(())
}