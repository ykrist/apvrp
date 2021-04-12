use super::{Task, TaskType, task_req};
use crate::{Req};

/// Returns `true` if completing `t1` and then `t2` does not violate the cover constraint
pub fn cover(t1: &Task, t2: &Task, n: Req) -> bool {
  use TaskType::*;
  match (t1.ty, t2.ty) {
    (ODepot, ty) => {
      !matches!(ty, ODepot)
    },

    (ty, DDepot) => {
      !matches!(ty, DDepot)
    }

    (_, ODepot) => { false }

    (DDepot, _) => { false }

    // o+(a), o-(a) --> o+(b), o-(b)
    // where
    // a != b
    (Direct, Direct) => {
      t1.p != t2.p
    },

    // o+(a), o-(a) --> o+(b), p(s), d(s), o-(b)
    // where
    // a != b
    (Direct, Start) => {
      t1.p != t2.p
    },

    // o+(a), o-(a) --> d(s), o-(b)
    // where
    // a != b
    (Direct, End) => {
      t1.p != t2.p
    },

    // o+(a), o-(a) --> p(s), d(s), o-(b)
    // where
    // a != b
    (Direct, Request) => {
      t1.p != t2.p
    },

    // o+(a), o-(a) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // a != b
    (Direct, Transfer) => {
      t1.p != t2.p
    },

    // o+(a), p(r) --> o+(b), o-(b)
    // where
    // a != b
    (Start, Direct) => {
      t1.p != t2.p
    },

    // o+(a), p(r) --> o+(b), p(s), d(s), o-(b)
    // where
    // a != b
    // r != s
    (Start, Start) => {
      t1.end != t2.end
        && t1.p != t2.p
    },

    // o+(a), p(r) --> d(s), o-(b)
    // where
    // r != s OR a == b
    (Start, End) => {
      t1.p == t2.p || t1.end != t2.start - n
    },

    // o+(a), p(r) --> p(s), d(s), o-(b)
    // where
    // r != s
    (Start, Request) => {
      t1.end != t2.start
    },

    // o+(a), p(r) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // r != s0 OR a == b
    // r != s1
    (Start, Transfer) => {
      t1.end != t2.end
        && ( t1.p == t2.p || t1.end != t2.start - n)
    },

    // o+(a), p(r), d(r), o-(a) --> o+(b), o-(b)
    // where
    // a != b
    (End, Direct) => {
      t1.p != t2.p
    },

    // o+(a), p(r), d(r), o-(a) --> o+(b), p(s), d(s), o-(b)
    // where
    // a != b
    // r != s
    (End, Start) => {
      t1.start - n != t2.end
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r), o-(a) --> d(s), o-(b)
    // where
    // a != b
    // r != s
    (End, End) => {
      t1.start - n != t2.start - n
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r), o-(a) --> p(s), d(s), o-(b)
    // where
    // a != b
    // r != s
    (End, Request) => {
      t1.start - n != t2.start
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r), o-(a) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // a != b
    // r != s0
    // r != s1
    (End, Transfer) => {
      t1.start - n != t2.start - n
        && t1.start - n != t2.end
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r) --> o+(b), o-(b)
    // where
    // a != b
    (Request, Direct) => {
      t1.p != t2.p
    },

    // o+(a), p(r), d(r) --> o+(b), p(s), d(s), o-(b)
    // where
    // a != b
    // r != s
    (Request, Start) => {
      t1.start != t2.end
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r) --> d(s), o-(b)
    // where
    // r != s
    (Request, End) => {
      t1.start != t2.start - n
    },

    // o+(a), p(r), d(r) --> p(s), d(s), o-(b)
    // where
    // r != s
    (Request, Request) => {
      t1.start != t2.start
    },

    // o+(a), p(r), d(r) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // r != s0
    // r != s1
    (Request, Transfer) => {
      t1.start != t2.start - n
        && t1.start != t2.end
    },

    // o+(a), p(r0), d(r0), p(r1) --> o+(b), o-(b)
    // where
    // a != b
    (Transfer, Direct) => {
      t1.p != t2.p
    },

    // o+(a), p(r0), d(r0), p(r1) --> o+(b), p(s), d(s), o-(b)
    // where
    // a != b
    // r0 != s
    // r1 != s
    (Transfer, Start) => {
      t1.start - n != t2.end
        && t1.end != t2.end
        && t1.p != t2.p
    },

    // o+(a), p(r0), d(r0), p(r1) --> d(s), o-(b)
    // where
    // r1 != s OR a == b
    // r0 != s
    (Transfer, End) => {
      t1.start - n != t2.start - n
        && ( t1.p == t2.p || t1.end != t2.start - n)
    },

    // o+(a), p(r0), d(r0), p(r1) --> p(s), d(s), o-(b)
    // where
    // r0 != s
    // r1 != s
    (Transfer, Request) => {
      t1.start - n != t2.start
        && t1.end != t2.start
    },

    // o+(a), p(r0), d(r0), p(r1) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // r1 != s0 OR a == b
    // r0 != s0
    // r0 != s1
    // r1 != s1~
    (Transfer, Transfer) => {
      t1.start - n != t2.start - n
        && t1.start - n != t2.end
        && t1.end != t2.end
        && ( t1.p == t2.p || t1.end != t2.start - n)
    },


  }
}