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