#![feature(generic_associated_types)]
#![feature(fn_traits)]
#![feature(assert_matches)]
#![feature(custom_test_frameworks)]
#![allow(dead_code)]
#![allow(unused_imports)]
#![deny(unused_must_use)]

pub use anyhow::Result;
pub use fnv::{FnvHashMap as Map, FnvHashSet as Set};

pub(crate) use std::assert_matches::*;
use std::path::Path;

use itertools::Itertools;
use std::fmt;

pub use instances::dataset::apvrp::{
  ApvrpInstance, Av as RawAv, Cost, Loc as RawLoc, Pv as RawPv, Req as RawReq, Time,
};
use instances::dataset::apvrp::{LocSetStarts, DSET, MEISEL_A, TILK_AB};

pub use wrapper_types::*;

mod wrapper_types {
  use super::*;
  use serde::{Deserialize, Serialize};
  use std::num::ParseIntError;
  use std::str::FromStr;

  macro_rules! new_wrapper_ty {
    ($t:ident = $inner:ty) => {
      #[derive(Copy, Clone, Hash, Eq, Ord, PartialOrd, PartialEq, Serialize, Deserialize)]
      #[repr(transparent)]
      #[serde(transparent)]
      pub struct $t(pub $inner);

      impl fmt::Display for $t {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
          fmt::Debug::fmt(self, f)
        }
      }

      impl fmt::Debug for $t {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
          fmt::Debug::fmt(&self.0, f)
        }
      }

      impl FromStr for $t {
        type Err = ParseIntError;
        fn from_str(s: &str) -> Result<Self, Self::Err> {
          s.parse::<$inner>().map($t)
        }
      }
    };
  }

  new_wrapper_ty! { Av = RawAv }
  new_wrapper_ty! { Avg = RawAv }
  new_wrapper_ty! { Pv = RawPv }
  new_wrapper_ty! { Req = RawReq }
}

pub use instances::dataset::Dataset;

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
  /// The earliest time a PV may *leave* at the pickup location of a request
  pub pv_req_start_time: Map<(Pv, Req), Time>,
  /// The lates time a PV may *arrive* at the delivery location of a request
  pub pv_req_end_time: Map<(Pv, Req), Time>,
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

pub fn map_with_capacity<K, V>(capacity: usize) -> Map<K, V> {
  Map::with_capacity_and_hasher(capacity, fnv::FnvBuildHasher::default())
}

pub fn set_with_capacity<T>(capacity: usize) -> Set<T> {
  FnvHashSet::with_capacity_and_hasher(capacity, fnv::FnvBuildHasher::default())
}

#[derive(Hash, Copy, Clone, Eq, PartialEq)]
pub enum Loc {
  /// Passive vehicle origin (p)
  Po(Pv),
  /// Passive vehicle destination (p)
  Pd(Pv),
  /// Request pickup (r)
  ReqP(Req),
  /// Request delivery (r)
  ReqD(Req),
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
      _ => unimplemented!("only defined for request pickups, AV- and PV origins"),
    }
  }

  /// Return the "origin" for Request deliverys and dest depots
  pub fn origin(&self) -> Loc {
    match self {
      Loc::ReqD(r) => Loc::ReqD(*r),
      Loc::Ad => Loc::Ao,
      Loc::Pd(p) => Loc::Po(*p),
      _ => unimplemented!("only defined for request deliveries, AV- and PV destinations"),
    }
  }

  pub fn req(&self) -> Req {
    match self {
      Loc::ReqD(r) | Loc::ReqP(r) => *r,
      _ => unimplemented!("only defined for request pickups and deliveries"),
    }
  }

  pub fn pv(&self) -> Pv {
    match self {
      Loc::Po(p) | Loc::Pd(p) => *p,
      _ => unimplemented!("only defined for passive vehicle origins and destinations"),
    }
  }
}

pub trait LocSetStartsExt {
  fn odepot_to_pv(&self, i: RawLoc) -> Pv;
  fn pickup_to_req(&self, i: RawLoc) -> Req;
  fn decode(&self, loc: RawLoc) -> Loc;
  fn encode(&self, loc: Loc) -> RawLoc;
}

impl LocSetStartsExt for LocSetStarts {
  #[inline(always)]
  fn odepot_to_pv(&self, i: RawLoc) -> Pv {
    Pv(i - self.pv_o)
  }

  #[inline(always)]
  fn pickup_to_req(&self, i: RawLoc) -> Req {
    Req(i - self.req_p)
  }

  fn decode(&self, loc: RawLoc) -> Loc {
    if loc == self.avo {
      Loc::Ao
    } else if loc < self.pv_d {
      Loc::Po(self.odepot_to_pv(loc))
    } else if loc < self.req_p {
      Loc::Pd(Pv(loc - self.pv_d))
    } else if loc < self.req_d {
      Loc::ReqP(self.pickup_to_req(loc))
    } else if loc < self.avd {
      Loc::ReqD(Req(loc - self.req_d))
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
      Loc::Po(p) => self.pv_o + p.0,
      Loc::Pd(p) => self.pv_d + p.0,
      Loc::ReqP(r) => self.req_p + r.0,
      Loc::ReqD(r) => self.req_d + r.0,
    }
  }
}

#[derive(Debug, Clone)]
pub struct Lookups {
  pub data: Data,
  pub sets: Sets,
  pub tasks: Tasks,
  pub pv_routes: Option<PvRoutes>,
}

impl Lookups {
  pub fn load_data_and_build(data_index: usize) -> Result<Self> {
    let data = DSET
      .load_instance(data_index)
      .map(preprocess::full_pipeline)?;

    let sets = Sets::new(&data);
    let tasks = Tasks::generate(&data, &sets);

    info!(num_tasks = tasks.all.len(), "task generation finished");
    Ok(Lookups {
      data,
      sets,
      tasks,
      pv_routes: None,
    })
  }

  pub fn generate_pv_routes(&mut self) {
    if self.pv_routes.is_some() {
      panic!("Routes have already been generated.")
    }
    self.pv_routes = Some(PvRoutes::new(&self.sets, &self.data, &self.tasks))
  }

  pub fn iter_yvars<'a>(&'a self) -> impl Iterator<Item = (Avg, Task, Task)> + 'a {
    self
      .tasks
      .compat_with_av
      .iter()
      .flat_map(move |(&av, av_tasks)| {
        av_tasks
          .iter()
          .flat_map(move |t1| self.tasks.succ[t1].iter().map(move |t2| (t1, t2)))
          .filter(move |(_, t2)| av_tasks.contains(t2))
          .map(move |(&t1, &t2)| (av, t1, t2))
      })
  }

  // pub fn iter_xvars<'a>(&'a self) -> impl Iterator<Item=MpVar> + 'a {
  //   todo!()
  // }
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
  fn as_ref(&self) -> &Data {
    self
  }
}

mod tasks;

pub use tasks::*;

use logging::*;

mod sets;

pub use sets::Sets;

mod utils;

pub use utils::{iter_cycle, IoContext, Json};

mod constants;

pub use constants::*;

pub mod graph;
pub mod logging;
pub mod model;
pub mod preprocess;
pub mod solution;

use crate::colgen::PvRoutes;
use crate::model::mp::MpVar;
use crate::model::sp::SpConstr;
use anyhow::Context;
use fnv::FnvHashSet;
use std::fmt::Display;

pub mod colgen;
pub mod experiment;
pub mod schedule;
pub mod test;
// TODO tests for encode and decode.

pub const COMMIT_HASH: &'static str = env!("COMMIT_HASH");

fn commit_hash() -> Result<String> {
  let output = std::process::Command::new("git")
    .args(["rev-parse", "HEAD"])
    .output()?;
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

pub fn dataset_group(name: &str) -> Result<Vec<usize>> {
  let mut path = Path::new(env!("CARGO_MANIFEST_DIR")).join("data/group");
  path.push(name);
  let contents = std::fs::read_to_string(&path).read_context(&path)?;
  let mut inds = Vec::new();
  for l in contents.lines() {
    let i: usize = l.parse()?;
    inds.push(i);
  }
  Ok(inds)
}
