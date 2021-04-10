#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_imports)]

pub use instances::dataset::apvrp::{
  MEISEL_A,
  TILK_AB,
  DSET,
  Av, Pv, Time, Cost, Loc, Req,
};
/// Active Vehicle Group.
pub type Avg = Av;

pub struct Data {
  pub id: String,
  pub odepot: usize,
  pub ddepot: usize,
  pub n_req: usize,
  pub n_passive: usize,
  pub n_active: usize,
  pub n_loc: usize,
  pub tmax: Time,
  pub srv_time: Map<Req, Time>,
  pub start_time: Map<Req, Time>,
  pub end_time: Map<Req, Time>,
  pub compat_req_passive: Map<Req, Vec<Pv>>,
  pub compat_passive_req: Map<Pv, Vec<Req>>,
  pub compat_passive_active: Map<Pv, Vec<Avg>>,
  pub compat_active_passive: Map<Avg, Vec<Pv>>,
  pub travel_cost: Map<(usize, usize), Cost>,
  pub travel_time: Map<(usize, usize), Time>,
  pub av_groups: Map<Avg, Vec<Av>>,
}
pub use fnv::FnvHashMap as Map;
pub use anyhow::Result;

use itertools::Itertools;

mod tasks;
pub use tasks::*;

pub fn map_with_capacity<K,V>(capacity: usize) -> Map<K,V> {
  Map::with_capacity_and_hasher(capacity, fnv::FnvBuildHasher::default())
}

mod sets;
pub use sets::Sets;

pub mod model;
pub mod preprocess;
pub mod graph;
pub mod logging;
