use crate::{Loc, Data, Avg};
use std::ops::Range;
use std::iter::Iterator;

#[derive(Debug, Clone)]
pub struct Sets {
  pv_o_srt: usize,
  pv_d_srt: usize,
  r_o_srt: usize,
  r_d_srt: usize,
  end: usize,
  av_groups: Vec<Avg>,
}

impl Sets {
  pub fn new(data: &Data) -> Self {
    let pv_o_srt = 1;
    let pv_d_srt = pv_o_srt + data.n_passive;
    let r_o_srt = pv_d_srt + data.n_passive;
    let r_d_srt = r_o_srt + data.n_req;
    let end = r_d_srt + data.n_req;
    let mut av_groups : Vec<_> = data.av_groups.keys().copied().collect();
    av_groups.sort();

    Sets {
      pv_d_srt,
      pv_o_srt,
      r_o_srt,
      r_d_srt,
      end,
      av_groups
    }
  }


  /// Set of passive vehicles/passive vehicle origin locations
  #[inline(always)]
  pub fn pv_origins(&self) -> Range<Loc> {
    self.pv_o_srt..self.pv_d_srt
  }

  /// Set of passive vehicle dest. locations
  #[inline(always)]
  pub fn pv_dests(&self) -> Range<Loc> {
    self.pv_d_srt..self.r_o_srt
  }

  /// Set of passive vehicle locations
  #[inline(always)]
  pub fn pv_locs(&self) -> Range<Loc> {
    self.pv_o_srt..self.r_o_srt
  }

  /// Set of requests/request origin locations
  #[inline(always)]
  pub fn req_pickups(&self) -> Range<Loc> {
    self.r_o_srt..self.r_d_srt
  }

  /// Set of request dest. locations
  #[inline(always)]
  pub fn req_deliveries(&self) -> Range<Loc> {
    self.r_d_srt..self.end
  }

  /// Set of request locations
  #[inline(always)]
  pub fn req_locs(&self) -> Range<Loc> {
    self.r_o_srt..self.end
  }

  /// Set of locations
  #[inline(always)]
  pub fn locs(&self) -> Range<Loc> {
    0..(self.end + 1)
  }

  // /// Set of active vehicles
  #[inline(always)]
  pub fn avs(&self) -> impl Iterator<Item=Avg> + '_ {
    self.av_groups.iter().copied()
  }
}
