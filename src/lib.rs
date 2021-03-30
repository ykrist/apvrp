#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(unused_imports)]

pub use instances::dataset::apvrp::{
  MEISEL_A,
  TILK_AB,
  DSET,
  ApvrpInstance,
  Av, Pv, Time, Cost, Loc, Req,
};
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

mod model;
pub mod preprocess;

pub use model::TaskModelMaster;
