use itertools::Itertools;
use grb::expr::{Expr, LinExpr, QuadExpr};
use grb::constr::IneqExpr;
use grb::{attr, Var};
use std::fmt;
use std::fmt::{Write, Debug};
use std::collections::HashMap;
use std::hash::{Hash, BuildHasher};
use std::cell::{RefCell, Ref};
use std::rc::Rc;
use std::path::{Path, PathBuf};
use std::iter::Peekable;
use std::ops::AddAssign;
use serde::Serialize;
use serde::de::DeserializeOwned;
use anyhow::Context;

pub fn iter_cycle<'a, T>(vals: &'a [T]) -> impl Iterator<Item=(&'a T, &'a T)> + 'a {
  let n = vals.len();
  assert!(n >= 2);
  let last = unsafe { vals.get_unchecked(n - 1) };
  let first = unsafe { vals.get_unchecked(0) };
  vals.iter().tuple_windows()
    .chain(std::iter::once((last, first)))
}

// pub struct CyclicPermutator<T> {
//   objects: Rc<Vec<T>>,
//   idx: usize,
//   len: usize,
// }
//
// impl <T: Clone> CyclicPermutator<T> {
//   pub fn new(objects: &[T]) -> CyclicPermutator<T> {
//     let len = objects.len();
//     let mut v = Vec::with_capacity(2*objects.len() - 1);
//     v.extend_from_slice(objects);
//     v.extend_from_slice(&objects[..(len-1)]);
//     CyclicPermutator{ objects: Rc::new(v), len, idx: 0}
//   }
// }
//
// impl<T> Iterator for CyclicPermutator<T> {
//   type Item = Rc<Vec<T>>;
//
//   fn next(&mut self) -> Option<Self::Item> {
//     if self.idx < self.len {
//
//       self.idx += 1;
//     } else {
//       None
//     }
//   }
// }

pub trait PermuteSliceClone {
  type Item;
  fn permutations(&self) -> Permutator<Self::Item>;
}

/// # Safety:
/// Slice must have a length of 2
unsafe fn map_window2<T>(window: &[T]) -> (&T, &T) {
  (window.get_unchecked(0), window.get_unchecked(1))
}

pub fn iter_pairs<'a, T>(slice: &'a [T]) -> impl Iterator<Item=(&'a T, &'a T)> {
  slice.windows(2)
    .map(|pair| (
      unsafe { pair.get_unchecked(0) },
      unsafe { pair.get_unchecked(1) },
    ))
}

impl<T: Clone + Debug> PermuteSliceClone for &[T] {
  type Item = T;
  fn permutations(&self) -> Permutator<Self::Item> {
    Permutator::new(self.to_vec())
  }
}

pub struct Permutator<T> {
  objects: Rc<Vec<T>>,
  k: usize,
  c: Vec<usize>,
  i: usize,
  perm0: bool,
}


impl<T: Debug> Permutator<T> {
  pub fn new(objects: Vec<T>) -> Permutator<T> {
    let n = objects.len();
    let c = if n > 1 { vec![0; n - 1] } else { vec![] };
    Permutator { objects: Rc::new(objects), k: n, c, i: 1, perm0: false }
  }

  pub fn finish(self) -> Vec<T> {
    Rc::try_unwrap(self.objects)
      .expect("All refs must be dropped first")
  }
}

impl<T: Debug + Clone> Iterator for Permutator<T> {
  type Item = Rc<Vec<T>>;

  fn next(&mut self) -> Option<Self::Item> {
    if !self.perm0 {
      self.perm0 = true;
      return Some(self.objects.clone());
    } else {
      if self.i < self.objects.len() {
        if self.c[self.i - 1] < self.i {
          let objects = Rc::get_mut(&mut self.objects)
            .expect("refs must be dropped before the next .next() call");
          if self.i % 2 == 0 {
            objects.swap(0, self.i);
          } else {
            objects.swap(self.c[self.i - 1], self.i);
          }

          self.c[self.i - 1] += 1;
          self.i = 1;

          return Some(self.objects.clone());
        } else {
          self.c[self.i - 1] = 0;
          self.i += 1;
          return self.next();
        }
      } else {
        None
      }
    }
  }
}

pub fn factorial(n: usize) -> usize {
  if n <= 1 { 1 } else { n * factorial(n - 1) }
}

pub trait HashMapExt<K, V> {
  fn retain_ok<F, E>(&mut self, filter: F) -> std::result::Result<(), E>
    where
      F: FnMut(&K, &mut V) -> std::result::Result<bool, E>;
}


impl<K, V, S> HashMapExt<K, V> for HashMap<K, V, S>
  where
    K: Hash + Eq,
    S: BuildHasher,
{
  fn retain_ok<F, E>(&mut self, mut f: F) -> std::result::Result<(), E>
    where
      F: FnMut(&K, &mut V) -> std::result::Result<bool, E>
  {
    let mut ret = Ok(());

    let wrapper = |key: &K, val: &mut V| -> bool {
      if ret.is_err() {
        true
      } else {
        match f(key, val) {
          Ok(keep) => keep,
          Err(e) => {
            ret = Err(e);
            true // keep on error
          }
        }
      }
    };

    self.retain(wrapper);
    ret
  }
}

pub trait VarIterExt {
  fn sum_into(self, expr: &mut Expr);

  fn sum_into_count(self, expr: &mut Expr) -> usize;
}

impl<I, T> VarIterExt for I
  where
    I: Iterator<Item=T>,
    Expr: AddAssign<T>,
{
  fn sum_into(self, expr: &mut Expr) {
    for elem in self {
      *expr += elem;
    }
  }

  fn sum_into_count(self, expr: &mut Expr) -> usize {
    let mut count = 0;
    for elem in self {
      *expr += elem;
      count += 1;
    }
    count
  }
}

pub trait PeekableExt: Iterator {
  fn for_each_except_last<F>(self, f: F) -> Option<Self::Item> where F: FnMut(Self::Item);
}

impl<I: Iterator> PeekableExt for Peekable<I> {
  fn for_each_except_last<F>(mut self, mut f: F) -> Option<Self::Item>
    where
      F: FnMut(Self::Item)
  {
    while let Some(item) = self.next() {
      if self.peek().is_none() {
        return Some(item);
      } else {
        f(item)
      }
    }
    None
  }
}

pub trait Json: Serialize + DeserializeOwned {
  fn from_json_file(path: impl AsRef<Path>) -> anyhow::Result<Self> {
    let f = std::io::BufReader::new(std::fs::File::open(path)?);
    Ok(serde_json::from_reader(f)?)
  }

  fn to_json_file(&self, path: impl AsRef<Path>) -> anyhow::Result<()> {
    let f = std::io::BufWriter::new(std::fs::File::create(path)?);
    serde_json::to_writer_pretty(f, self)?;
    Ok(())
  }
}

impl<T: Serialize + DeserializeOwned> Json for T {}


#[cfg(test)]
pub(crate) fn test_data_file(filename: &str) -> PathBuf {
  let mut path = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data"));
  path.push(filename);
  path
}

#[cfg(test)]
mod tests {
  use super::*;
  use grb::prelude::*;
  use anyhow::Result;
  use crate::Loc;

  #[test]
  fn loc_size() {
    assert_eq!(std::mem::size_of::<Loc>(), 4);
  }


  #[test]
  fn permutations() {
    check_permutations(0);
    check_permutations(1);
    check_permutations(2);
    check_permutations(3);
    check_permutations(4);
    check_permutations(5);
    check_permutations(6);
    check_permutations(7);
    check_permutations(8);
  }


  fn check_permutations(n: usize) {
    use std::collections::HashSet;

    let perms: HashSet<_> = Permutator::new((0..n).collect())
      .map(|p| {
        let p = &*p;
        println!("{:?}", p);
        p.clone()
      })
      .collect();

    assert_eq!(perms.len(), factorial(n))
  }

  #[test]
  fn permutator_finish() {
    let mut permutator = Permutator::new(vec![0, 1, 2]);
    let q = permutator.next();
    drop(q);
    permutator.finish();
  }


  #[test]
  #[should_panic]
  fn permutator_finish_fail() {
    let mut permutator = Permutator::new(vec![0, 1, 2]);
    let q = permutator.next();
    permutator.finish();
  }


  #[test]
  #[should_panic]
  fn permutator_next_fail() {
    let mut permutator = Permutator::new(vec![0, 1, 2]);
    let x = permutator.next();
    let y = permutator.next();
  }

  #[test]
  fn permutator_next() {
    let mut permutator = Permutator::new(vec![0, 1, 2]);
    let x = permutator.next();
    drop(x);
    let y = permutator.next();
  }
}

pub trait IoContext<T, E>: Context<T, E> + Sized {
  fn read_context(self, p: impl AsRef<Path>) -> anyhow::Result<T> {
    self.with_context(|| format!( "Failed to read file {:?}", p.as_ref()))
  }

  fn write_context(self, p: impl AsRef<Path>) -> anyhow::Result<T> {
    self.with_context(|| format!("Failed to write to file {:?}", p.as_ref()))
  }
}

impl<R, T, E> IoContext<T, E> for R where R: Context<T, E> + Sized {}