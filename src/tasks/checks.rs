use super::{Task, TaskType, task_req};
use crate::*;

/// Returns `true` if completing `t1` and then `t2` does not violate the cover constraint
pub fn cover(t1: &Task, t2: &Task) -> bool {
  use TaskType::*;
  match (t1.ty, t2.ty) {
    (ODepot, ty) => {
      ty != ODepot
    },

    (ty, DDepot) => {
      ty != DDepot
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
      t1.end.req() != t2.end.req()
        && t1.p != t2.p
    },

    // o+(a), p(r) --> d(s), o-(b)
    // where
    // r != s OR a == b
    (Start, End) => {
      t1.p == t2.p || t1.end.req() != t2.start.req()
    },

    // o+(a), p(r) --> p(s), d(s), o-(b)
    // where
    // r != s
    (Start, Request) => {
      t1.end.req() != t2.start.req()
    },

    // o+(a), p(r) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // r != s0 OR a == b
    // r != s1
    (Start, Transfer) => {
      t1.end.req() != t2.end.req()
        && (t1.p == t2.p || t1.end.req() != t2.start.req())
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
      t1.start.req() != t2.end.req()
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r), o-(a) --> d(s), o-(b)
    // where
    // a != b
    // r != s
    (End, End) => {
      t1.start.req() != t2.start.req()
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r), o-(a) --> p(s), d(s), o-(b)
    // where
    // a != b
    // r != s
    (End, Request) => {
      t1.start.req() != t2.start.req()
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r), o-(a) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // a != b
    // r != s0
    // r != s1
    (End, Transfer) => {
      t1.start.req() != t2.start.req()
        && t1.start.req() != t2.end.req()
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
      t1.start.req() != t2.end.req()
        && t1.p != t2.p
    },

    // o+(a), p(r), d(r) --> d(s), o-(b)
    // where
    // r != s
    (Request, End) => {
      t1.start.req() != t2.start.req()
    },

    // o+(a), p(r), d(r) --> p(s), d(s), o-(b)
    // where
    // r != s
    (Request, Request) => {
      t1.start.req() != t2.start.req()
    },

    // o+(a), p(r), d(r) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // r != s0
    // r != s1
    (Request, Transfer) => {
      t1.start.req() != t2.start.req()
        && t1.start.req() != t2.end.req()
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
      t1.start.req() != t2.end.req()
        && t1.end.req() != t2.end.req()
        && t1.p != t2.p
    },

    // o+(a), p(r0), d(r0), p(r1) --> d(s), o-(b)
    // where
    // r1 != s OR a == b
    // r0 != s
    (Transfer, End) => {
      t1.start.req() != t2.start.req()
        && (t1.p == t2.p || t1.end.req() != t2.start.req())
    },

    // o+(a), p(r0), d(r0), p(r1) --> p(s), d(s), o-(b)
    // where
    // r0 != s
    // r1 != s
    (Transfer, Request) => {
      t1.start.req() != t2.start.req()
        && t1.end.req() != t2.start.req()
    },

    // o+(a), p(r0), d(r0), p(r1) --> d(s0), p(s1), d(s1), o-(b)
    // where
    // r1 != s0 OR a == b
    // r0 != s0
    // r0 != s1
    // r1 != s1
    (Transfer, Transfer) => {
      t1.start.req() != t2.start.req()
        && t1.start.req() != t2.end.req()
        && t1.end.req() != t2.end.req()
        && (t1.p == t2.p || t1.end.req() != t2.start.req())
    },

  }
}


/// Returns `true` if chain does not visit the same two locations (except AV depots) more than once.
/// Returns `false` otherwise.
pub fn av_chain_cover(chain: &[Task]) -> bool { impl_chain_cover_and_pred(chain, true) }

/// Returns `true` if chain does not visit the same two locations (except AV depots) more than once
/// AND there is no conflict between the locations visited before and after the chain.
/// Returns `false` otherwise.
pub fn av_chain_cover_pred(chain: &[Task]) -> bool { impl_chain_cover_and_pred(chain, false) }

/// Returns `true` if chain is legal:
///   1. Chain does not visit the same two locations (except AV depots) more than once
///   2. There are no conflicts between the locations visited before and after the chain.
///   3. An AV-feasible schedule exists for the chain.
///
/// Returns `false` otherwise.
#[inline]
pub fn av_chain_full(data: &Data, chain: &[Task]) -> bool {
  schedule::check_pv_route(data, chain) && av_chain_cover_pred(chain)
}

fn impl_chain_cover_and_pred(chain: &[Task], cover_only: bool) -> bool {
  use TaskType::*;

  let mut locs_visited_by_chain = set_with_capacity(chain.len()*2);

  for t in chain {
    // chain[k].end may be equal to chain[k+1].start
    locs_visited_by_chain.insert(t.start);
    if !locs_visited_by_chain.insert(t.end) {
      // Direct cover violation
      return false;
    }
  }
  if cover_only {
    return true
  }

  let mut locs_visited_before_chain = FnvHashSet::default();
  let mut locs_visited_after_chain = FnvHashSet::default();

  let mut implied_before = |loc: Loc|
    if locs_visited_by_chain.contains(&loc) {
      // TODO we could walk the chain backwards for check for precedence within the chain
      true
    } else if locs_visited_after_chain.contains(&loc) {
      false
    } else {
      locs_visited_before_chain.insert(loc);
      true
    };

  let mut implied_after = |loc: Loc|
    if locs_visited_by_chain.contains(&loc) {
      // TODO we could walk the chain for check for precedence within the chain
      true
    } else if locs_visited_before_chain.contains(&loc) {
      false
    } else {
      locs_visited_after_chain.insert(loc);
      true
    };


  // implied cover before and after chain
  for t in chain {
    let precedence_is_consistent = match t.ty {
      ODepot | Direct | DDepot => { true }

      Start =>
        implied_after(t.start.dest())
          && implied_after(t.end.dest()),

      End =>
        implied_before(t.start.origin())
          && implied_before(t.end.origin()),

      Request =>
        implied_before(Loc::Po(t.p.unwrap()))
          && implied_after(Loc::Pd(t.p.unwrap())),

      Transfer =>
        implied_before(Loc::Po(t.p.unwrap()))
          && implied_before(t.start.origin())
          && implied_after(t.end.dest())
          && implied_after(Loc::Pd(t.p.unwrap())),
    };
    if !precedence_is_consistent {
      return false;
    }
  }


  true
}
