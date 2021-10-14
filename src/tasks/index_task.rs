use crate::*;
use std::str::FromStr;
use serde::{Deserialize, Serialize};
use std::ops::Deref;

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize, Ord, PartialOrd)]
pub enum IdxPvTask {
  Transfer(Pv, Req, Req),
  Start(Pv, Req),
  Request(Pv, Req),
  End(Pv, Req),
  Direct(Pv),
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize, Ord, PartialOrd)]
pub enum IdxTask {
  Transfer(Req, Req),
  Start(Pv, Req),
  Request(Req),
  End(Pv, Req),
  Direct(Pv),
  ODepot,
  DDepot,
}

impl IdxTask {
  pub fn infer_pv(&self) -> Option<Pv> {
    use IdxTask::*;
    match self {
      Direct(p) | Start(p, _) | End(p, _) => Some(*p),
      _ => None,
    }
  }
}

impl From<(Pv, IdxTask)> for IdxPvTask {
  fn from((p, t): (Pv, IdxTask)) -> Self {
    debug_assert!(t.infer_pv().is_none() || t.infer_pv() == Some(p));
    match t {
      IdxTask::Request(r) => IdxPvTask::Request(p, r),
      IdxTask::Start(_, r) => IdxPvTask::Start(p, r),
      IdxTask::End(_, r) => IdxPvTask::End(p, r),
      IdxTask::Direct(_) => IdxPvTask::Direct(p),
      IdxTask::Transfer(r1, r2) => IdxPvTask::Transfer(p, r1, r2),
      IdxTask::DDepot | IdxTask::ODepot => panic!("not valid for AV depot tasks")
    }
  }
}

impl From<IdxPvTask> for IdxTask {
  fn from(t: IdxPvTask) -> Self {
    match t {
      IdxPvTask::Request(_, r) => IdxTask::Request(r),
      IdxPvTask::Transfer(_, r1, r2) => IdxTask::Transfer(r1, r2),
      IdxPvTask::Start(p, r) => IdxTask::Start(p, r),
      IdxPvTask::End(p, r) => IdxTask::End(p, r),
      IdxPvTask::Direct(p) => IdxTask::Direct(p),
    }
  }
}

pub trait TaskIndex {
  type Index;

  fn index(&self) -> Self::Index;
}

impl TaskIndex for Task {
  type Index = IdxTask;

  fn index(&self) -> Self::Index {
    match self.ty {
      TaskType::ODepot => IdxTask::ODepot,
      TaskType::DDepot => IdxTask::DDepot,
      TaskType::Request => IdxTask::Request(self.start.req()),
      TaskType::Transfer => IdxTask::Transfer(self.start.req(), self.end.req()),
      TaskType::Start => IdxTask::Start(self.start.pv(), self.end.req()),
      TaskType::End => IdxTask::End(self.end.pv(), self.start.req()),
      TaskType::Direct => IdxTask::Direct(self.start.pv()),
    }
  }
}

impl TaskIndex for PvTask {
  type Index = IdxPvTask;

  fn index(&self) -> Self::Index {
    match self.ty {
      TaskType::Request => IdxPvTask::Request(self.p, self.start.req()),
      TaskType::Transfer => IdxPvTask::Transfer(self.p, self.start.req(), self.end.req()),
      TaskType::Start => IdxPvTask::Start(self.p, self.end.req()),
      TaskType::End => IdxPvTask::End(self.p, self.start.req()),
      TaskType::Direct => IdxPvTask::Direct(self.p),
      _ => unreachable!(),
    }
  }
}

impl<T> TaskIndex for [T]
  where
    T: TaskIndex,
{
  type Index = Vec<T::Index>;

  fn index(&self) -> Self::Index {
    self.iter().map(|t| t.index()).collect()
  }
}


mod parsers {
  use super::*;
  use regex::Regex;
  macro_rules! parser {
    ($name: ident : $re:literal) => {
      pub fn $name(s: &str) -> Option<()> {
        lazy_static::lazy_static! {
           static ref RE : Regex = Regex::new($re).unwrap();
        }
        if RE.is_match(s) {
          Some(())
        } else {
          None
        }
      }
    };

    ($name: ident : $re:literal => $($t:path),+) => {
      #[allow(unused_parens)]
      pub fn $name(s: &str) -> Option<($($t),*)> {
        lazy_static::lazy_static! {
           static ref RE : Regex = Regex::new($re).unwrap();
        }
        let matches = RE.captures(s)?;
        let mut match_iter = matches.iter().skip(1);
        return Some((
        $(match_iter.next()??.as_str().parse::<$t>().expect("regexp captured invalid string")),*
        ))
      }
    };
  }

  parser!{   odp : r"ODp" }
  parser!{   ddp : r"DDp" }
  parser!{   dir : r"Dir\((\d+)\)" => Pv }
  parser!{   trn : r"Trn\((\d+),(\d+)\)" => Req, Req }
  parser!{   req : r"Req\((\d+)\)" => Req }
  parser!{   end : r"End\((\d+),(\d+)\)" => Pv, Req }
  parser!{   srt : r"Srt\((\d+),(\d+)\)" => Pv, Req }
  parser!{ trn_p : r"Trn\((\d+),(\d+),(\d+)\)" => Pv, Req, Req }
  parser!{ req_p : r"Req\((\d+),(\d+)\)" => Pv, Req }

  macro_rules! one_of {
      ($target:ident => $([$($branch:tt)+])+) => {
        $(
          one_of!{ @IF $target $($branch)* }
        )*
      };

      (@IF $target:ident $p:ident => $e:ident) => {
        if $p($target).is_some() {
          return Ok($e)
        }
      };

      (@IF $target:ident $p:ident => $($v:ident),+ => $e:ident) => {
        #[allow(unused_parens)]
        if let Some(($($v),+)) = $p($target) {
          return Ok($e($($v),*))
        }
      };
  }

  impl FromStr for IdxPvTask {
    type Err = anyhow::Error;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
      use IdxPvTask::*;

      one_of!(s =>
        [trn_p => p, r1, r2 => Transfer]
        [req_p => p, r => Request]
        [srt => p, r => Start]
        [end => p, r => End]
        [dir => p => Direct]
      );

      anyhow::bail!("unable to parse: {}", s)
    }
  }


  impl FromStr for IdxTask {
    type Err = anyhow::Error;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
      use IdxTask::*;

      one_of!(s =>
        [req => p => Request]
        [trn => p, r => Transfer]
        [srt => p, r => Start]
        [end => p, r => End]
        [dir => p => Direct]
        [odp => ODepot]
        [ddp => DDepot]
      );

      anyhow::bail!("unable to parse: {}", s)
    }
  }
}



// TODO add some parsing tests