use crate::{Loc, Data, Avg, Av, Pv, Req, RawAv, RawPvTask, RawPv, RawReq};
use std::ops::Range;
use std::iter::Iterator;

#[derive(Debug, Clone)]
pub struct Sets {
  n_active: RawAv,
  n_passive: RawPv,
  n_req: RawReq,
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

  #[inline(always)]
  pub fn num_pvs(&self) -> usize {
    self.n_passive as usize
  }

  /// Set of passive vehicles
  #[inline(always)]
  pub fn pvs(&self) -> impl Iterator<Item=Pv> {
    (0..self.n_passive).map(Pv)
  }

  /// Set of passive vehicle origin locations
  #[inline(always)]
  pub fn pv_origins(&self) -> impl Iterator<Item=Loc> {
    self.pvs().map(|p| Loc::Po(p))
  }

  /// Set of passive vehicle dest. locations
  #[inline(always)]
  pub fn pv_dests(&self) -> impl Iterator<Item=Loc> {
    self.pvs().map(Loc::Pd)
  }

  /// Set of requests
  #[inline(always)]
  pub fn reqs(&self) -> impl Iterator<Item=Req> {
    (0..self.n_req).map(Req)
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
  pub fn av_groups(&self) -> impl Iterator<Item=Avg> + Clone + '_ {
    self.av_groups.iter().copied()
  }
}
