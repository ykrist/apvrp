use crate::*;
use std::str::FromStr;
use serde::{Deserialize, Serialize};

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum ShorthandPvTask {
  Transfer(Pv, Req, Req),
  Start(Pv, Req),
  Request(Pv, Req),
  End(Pv, Req),
  Direct(Pv),
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash, Serialize, Deserialize)]
pub enum ShorthandTask {
  Transfer(Req, Req),
  Start(Pv, Req),
  Request(Req),
  End(Pv, Req),
  Direct(Pv),
  ODepot,
  DDepot,
}


impl From<PvTask> for ShorthandPvTask {
  fn from(t: PvTask) -> Self {
    match t.ty {
      TaskType::Request => ShorthandPvTask::Request(t.p, t.start.req()),
      TaskType::Transfer => ShorthandPvTask::Transfer(t.p, t.start.req(), t.end.req()),
      TaskType::Start => ShorthandPvTask::Start(t.p, t.end.req()),
      TaskType::End => ShorthandPvTask::End(t.p, t.start.req()),
      TaskType::Direct => ShorthandPvTask::Direct(t.p),
      _ => unreachable!(),
    }
  }
}


impl From<ShorthandPvTask> for ShorthandTask {
  fn from(t: ShorthandPvTask) -> Self {
    match t {
      ShorthandPvTask::Request(_, r) => ShorthandTask::Request(r),
      ShorthandPvTask::Transfer(_, r1, r2) => ShorthandTask::Transfer(r1, r2),
      ShorthandPvTask::Start(p, r) => ShorthandTask::Start(p, r),
      ShorthandPvTask::End(p, r) => ShorthandTask::End(p, r),
      ShorthandPvTask::Direct(p) => ShorthandTask::Direct(p),
    }
  }
}

impl From<Task> for ShorthandTask {
  fn from(t: Task) -> Self {
    match t.ty {
      TaskType::ODepot => ShorthandTask::ODepot,
      TaskType::DDepot => ShorthandTask::DDepot,
      TaskType::Request => ShorthandTask::Request(t.start.req()),
      TaskType::Transfer => ShorthandTask::Transfer(t.start.req(), t.end.req()),
      TaskType::Start => ShorthandTask::Start(t.start().pv(), t.end.req()),
      TaskType::End => ShorthandTask::End(t.end.pv(), t.start.req()),
      TaskType::Direct => ShorthandTask::Direct(t.start.pv()),
    }
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

  impl FromStr for ShorthandPvTask {
    type Err = anyhow::Error;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
      use ShorthandPvTask::*;

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


  impl FromStr for ShorthandTask {
    type Err = anyhow::Error;
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
      use ShorthandTask::*;

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