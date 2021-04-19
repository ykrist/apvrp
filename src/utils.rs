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

pub trait QueryVarName {
  fn write_name(&self, var: &Var, f: &mut fmt::Formatter<'_>) -> fmt::Result;
}

impl<V, S> QueryVarName for HashMap<Var, V, S>
  where
    S: BuildHasher,
    V: AsRef<str>,
{
  fn write_name(&self, var: &Var, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let name = self.get(var).ok_or(fmt::Error)?.as_ref();
    f.write_str(name)
  }
}

impl QueryVarName for grb::Model
{
  fn write_name(&self, var: &Var, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    let name = self.get_obj_attr(grb::attr::VarName, var).map_err(|_| fmt::Error)?;
    f.write_str(&name)
  }
}

impl<F> QueryVarName for F
where
  F: Fn(&Var, &mut fmt::Formatter<'_>) -> fmt::Result
{
  fn write_name(&self, var: &Var, f: &mut fmt::Formatter<'_>) -> fmt::Result {
   self(var, f)
  }
}


pub struct NameMapped<'a, T: ?Sized, N> {
  writer: &'a N,
  inner: &'a T
}

pub trait AttachNameMap {
  fn with_names<'a, N: QueryVarName>(&'a self, name_map: &'a N) -> NameMapped<'a, Self, N> {
    NameMapped { inner: &self, writer: name_map }
  }
}

impl AttachNameMap for LinExpr {}
impl AttachNameMap for QuadExpr {}
impl AttachNameMap for Expr {}
impl AttachNameMap for Var {}
impl AttachNameMap for IneqExpr {}

impl<W: QueryVarName> fmt::Debug for NameMapped<'_, IneqExpr, W> {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    use grb::ConstrSense::*;
    let cmp = match self.inner.sense {
      Less => "≤",
      Greater => "≥",
      Equal => "=",
    };

    self.inner.lhs.with_names(self.writer).fmt(f)?;
    f.write_fmt(format_args!(" {} ", cmp))?;
    self.inner.rhs.with_names(self.writer).fmt(f)?;
    Ok(())
  }
}




fn float_fmt_helper(x: f64, ignore_val: f64) -> (Option<f64>, bool) {
  let positive = x > -f64::EPSILON;
  if (x - ignore_val).abs() < f64::EPSILON {
    (None, positive)
  } else if positive {
    (Some(x), positive)
  } else {
    (Some(-x), positive)
  }
}


impl<W> fmt::Debug for NameMapped<'_, LinExpr, W>
where
  W: QueryVarName
{
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    if self.inner.is_empty() {
      return f.write_str("<empty LinExpr>");
    }

    let (offset, positive) = float_fmt_helper(self.inner.get_offset(), 0.0);

    let mut is_first_term = false;
    if let Some(offset) = offset {
      f.write_fmt(format_args!("{}", if positive { offset } else { -offset }))?;
    } else {
      is_first_term = true;
    }

    for (var, &coeff) in self.inner.iter_terms() {
      let (coeff, positive) = float_fmt_helper(coeff, 1.0);

      // write the operator with the previous term
      if !is_first_term {
        f.write_str(if positive { " + " } else { " - " })?;
      } else {
        is_first_term = false;
        if !positive {
          f.write_char('-')?;
        }
      }
      if let Some(coeff) = coeff {
        f.write_fmt(format_args!("{} ", coeff))?;
      }
      self.writer.write_name(var, f)?;
    }
    Ok(())
  }
}


impl<W: QueryVarName> fmt::Debug for NameMapped<'_, Expr, W> {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    use self::Expr::*;
    match &self.inner {
      Constant(a) => {
        f.write_fmt(format_args!("{}", a))?;
      },
      Term(a, x) => {
        if (a - 1.0).abs() > f64::EPSILON {
          f.write_fmt(format_args!("{} ", a))?;
        }
        self.writer.write_name(x, f)?;
      }
      QTerm(a, x, y) => {
        if (a - 1.0).abs() > f64::EPSILON {
          f.write_fmt(format_args!("{} ", a))?;
        }
        self.writer.write_name(x, f)?;
        f.write_char('*')?;
        self.writer.write_name(y, f)?;
      }
      Linear(e) => {
        e.with_names(self.writer).fmt(f)?;
      },
      Quad(e) => unimplemented!(),
    }
    Ok(())
  }
}

impl<W: QueryVarName> fmt::Debug for NameMapped<'_, Var, W> {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    self.writer.write_name(self.inner, f)
  }
}





#[cfg(test)]
mod tests {
  use super::*;
  use grb::prelude::*;
  use anyhow::Result;
  use crate::Map;

  #[test]
  fn name_map_hashmap() -> Result<()> {
    let mut m = Model::new("")?;
    let x = add_ctsvar!(m, name: "X")?;
    let c = Expr::from(x + 1).into_linexpr()?;
    let lookup : Map<_, _> = vec![(x, "X".to_string())].into_iter().collect();

    println!("{:?}", c.with_names(&lookup));
    Ok(())
  }

  #[test]
  fn name_map_closure() -> Result<()> {
    let mut m = Model::new("")?;
    let x = add_ctsvar!(m, name: "X")?;
    let xname = "X".to_string();
    let c = Expr::from(x + 1).into_linexpr()?;

    println!("{:?}", c.with_names(&|_ : &Var, f: &mut fmt::Formatter<'_>| f.write_str(&xname)));
    Ok(())
  }


  #[test]
  fn name_map_model() -> Result<()> {
    let mut m = Model::new("")?;
    let x = add_ctsvar!(m, name: "X")?;
    m.update()?;
    let c = Expr::from(x + 1).into_linexpr()?;
    println!("{:?}", c.with_names(&m));
    Ok(())
  }
}