use crate::{Loc, Data, Avg, Av, Pv, Req};
use std::ops::Range;
use std::iter::Iterator;

#[derive(Debug, Clone)]
pub struct Sets {
  n_active: Av,
  n_passive: Pv,
  n_req: Req,
  av_groups: Vec<Avg>,
}

impl Sets {
  pub fn new(data: &Data) -> Self {
    let mut av_groups : Vec<_> = data.av_groups.keys().copied().collect();
    av_groups.sort();

    Sets {
      n_active: data.n_active,
      n_passive: data.n_passive,
      n_req: data.n_req,
      av_groups
    }
  }



  /// Set of passive vehicles
  #[inline(always)]
  pub fn pvs(&self) -> Range<Pv> {
    0..self.n_passive
  }

  /// Set of passive vehicle origin locations
  #[inline(always)]
  pub fn pv_origins(&self) -> impl Iterator<Item=Loc> {
    self.pvs().map(Loc::Po)
  }

  /// Set of passive vehicle dest. locations
  #[inline(always)]
  pub fn pv_dests(&self) -> impl Iterator<Item=Loc> {
    self.pvs().map(Loc::Pd)
  }

  /// Set of requests
  #[inline(always)]
  pub fn reqs(&self) -> Range<Req> {
    0..self.n_req
  }

  /// Request origin locations
  #[inline(always)]
  pub fn req_pickups(&self) -> impl Iterator<Item=Loc>  {
    self.reqs().map(Loc::ReqP)
  }

  /// Set of request dest. locations
  #[inline(always)]
  pub fn req_deliveries(&self) -> impl Iterator<Item=Loc>  {
    self.reqs().map(Loc::ReqD)
  }

  // /// Set of active vehicles
  #[inline(always)]
  pub fn avs(&self) -> impl Iterator<Item=Avg> + '_ {
    self.av_groups.iter().copied()
  }
}
