use itertools::Itertools;
use grb::expr::{Expr, LinExpr, QuadExpr};
use grb::constr::IneqExpr;
use grb::{attr, Var};
use std::fmt;
use std::fmt::Write;
use std::collections::HashMap;
use std::hash::BuildHasher;

pub fn iter_cycle<'a, T>(vals: &'a [T]) -> impl Iterator<Item=(&'a T, &'a T)> + 'a {
  let n = vals.len();
  assert!(n >= 2);
  let last = unsafe { vals.get_unchecked(n - 1) };
  let first = unsafe { vals.get_unchecked(0) };
  vals.iter().tuple_windows()
    .chain(std::iter::once((last, first)))
}

#[derive(Hash)]
enum Loc {
  /// Passive vehicle origin (p)
  Po(u16),
  /// Passive vehicle destination (p)
  Pd(u16),
  /// Request pickup (r)
  ReqP(u16),
  /// Request delivery (r)
  ReqD(u16),
  /// Request delivery (d)
  Ao,
  Ad,
}

impl Loc {
  pub fn destination(&self) -> Loc {
    match self {
      Loc::ReqP(r) => Loc::ReqD(*r),
      Loc::Ao => Loc::Ad,
      Loc::Po(p) => Loc::Pd(*p),
      _ => unimplemented!("only defined for request pickups, AV- and PV origins")
    }
  }

  pub fn origin(&self) -> Loc {
    match self {
      Loc::ReqD(r) => Loc::ReqD(*r),
      Loc::Ad => Loc::Ao,
      Loc::Pd(p) => Loc::Po(*p),
      _ => unimplemented!("only defined for request deliveries, AV- and PV destinations")
    }
  }

  pub fn decode(loc: usize, n_active: usize, n_passive: usize, n_req: usize) -> Loc {
    todo!()
  }

  pub fn encode(&self, loc: usize, n_active: usize, n_passive: usize, n_req: usize) -> usize {
    todo!()
  }

}


#[cfg(test)]
mod tests {
  use super::*;
  use grb::prelude::*;
  use anyhow::Result;
  use crate::Map;

  #[test]
  fn loc_size() {
    assert_eq!(std::mem::size_of::<Loc>(), 4);
  }
}