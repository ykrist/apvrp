use crate::*;
use tracing::{trace_span, trace};

/// Returns `true` if chain does not visit the same two locations more than once.
/// Returns `false` otherwise.
pub fn chain_cover(chain: &[impl LocPair]) -> bool {
  cover_check_impl(chain).is_some()
}

/// Returns `true` if chain does not visit the same two locations (except AV depots) more than once
/// AND there is no conflict between the locations visited before and after the chain.
/// Returns `false` otherwise.
pub fn av_chain_cover_pred(chain: &[Task]) -> bool {
  match cover_check_impl(chain) {
    Some(l) => av_precedence_check(chain, l),
    None => false,
  }
}

/// Returns `true` if chain does not visit the same two locations (except AV depots) more than once
/// AND there is no conflict between the locations visited before and after the chain.
/// Returns `false` otherwise.
pub fn pv_chain_cover_pred(chain: &[PvTask]) -> bool {
  match cover_check_impl(chain) {
    Some(l) => pv_precedence_check(chain, l),
    None => false,
  }
}

/// Returns `true` if chain is legal:
///   1. Chain does not visit the same two locations (except AV depots) more than once
///   2. There are no conflicts between the locations visited before and after the chain.
///   3. An AV-feasible schedule exists for the chain.
///
/// Returns `false` otherwise. Does **not** check PV-Req pairings.
#[inline]
pub fn av_chain_full(data: &Data, chain: &[Task]) -> bool {
  schedule::check_av_route(data, chain) && av_chain_cover_pred(chain)
}


struct LocsVisitedByChain {
  before: Set<Loc>,
  after: Set<Loc>,
  during: Set<Loc>,
}

impl LocsVisitedByChain {
  /// Declare that `loc` must be visited before or during the chain, returning `false` if there is a precedence conflict
  /// and `true` otherwise.
  pub fn implied_before(&mut self, loc: Loc) -> bool {
    if self.during.contains(&loc) {
      // TODO we could walk the chain backwards for check for precedence within the chain
      true
    } else if self.after.contains(&loc) {
      false
    } else {
      self.before.insert(loc);
      true
    }
  }

  /// Declare that `loc` must be visited after or during the chain, returning `false` if there is a precedence conflict
  /// and `true` otherwise.
  pub fn implied_after(&mut self, loc: Loc) -> bool {
    if self.during.contains(&loc) {
      // TODO we could walk the chain for check for precedence within the chain
      true
    } else if self.before.contains(&loc) {
      false
    } else {
      self.after.insert(loc);
      true
    }
  }
}

pub fn pv_chain_full(data: &Data, chain: &[PvTask]) -> bool {
  schedule::check_pv_route(data, chain) && pv_chain_cover_pred(chain)
}

fn cover_check_impl<T: LocPair>(chain: &[T]) -> Option<Set<Loc>> {
  let mut locs_visited_by_chain = set_with_capacity(chain.len() * 2);
  for t in chain {
    // if !t.is_depot() { // TODO dont think this is needed
    //   // chain[k].end may be equal to chain[k+1].start.  For AV depot tasks, t.start = t.end
    //   locs_visited_by_chain.insert(t.start());
    // }
    if !locs_visited_by_chain.insert(t.end()) {
      // Direct cover violation
      trace!("cover violation");
      return None;
    }
  }
  Some(locs_visited_by_chain)
}

fn av_precedence_check(chain: &[Task], locs_visited_by_chain: Set<Loc>) -> bool {
  use TaskType::*;
  let mut lv = LocsVisitedByChain { before: Set::default(), after: Set::default(), during: locs_visited_by_chain };

  for t in chain {
    let precedence_is_consistent = match t.ty {
      ODepot | Direct | DDepot | Request => { true }

      Start =>
        lv.implied_after(t.start.dest())
          && lv.implied_after(t.end.dest()),

      End =>
        lv.implied_before(t.start.origin())
          && lv.implied_before(t.end.origin()),

      Transfer =>
          lv.implied_before(t.start.origin())
          && lv.implied_after(t.end.dest())
    };
    if !precedence_is_consistent {
      trace!("precedence violation");
      return false;
    }
  }
  true
}

fn pv_precedence_check(chain: &[PvTask], locs_visited_by_chain: Set<Loc>) -> bool {
  use TaskType::*;
  let mut lv = LocsVisitedByChain { before: Set::default(), after: Set::default(), during: locs_visited_by_chain };

  for t in chain {
    let precedence_is_consistent = match t.ty {
      ODepot | Direct | DDepot => { true }

      Start =>
        lv.implied_after(t.start.dest())
          && lv.implied_after(t.end.dest()),

      End =>
        lv.implied_before(t.start.origin())
          && lv.implied_before(t.end.origin()),

      Request =>
        lv.implied_before(Loc::Po(t.p))
          && lv.implied_after(Loc::Pd(t.p)),

      Transfer =>
        lv.implied_before(Loc::Po(t.p))
          && lv.implied_before(t.start.origin())
          && lv.implied_after(t.end.dest())
          && lv.implied_after(Loc::Pd(t.p)),
    };
    if !precedence_is_consistent {
      trace!("precedence violation");
      return false;
    }
  }
  true
}

pub trait ChainExt {
  type Iter<'a>: Iterator + 'a;

  fn legal_before<'a>(&self, lu: &'a Lookups) -> Self::Iter<'a>;
  fn legal_after<'a>(&self, lu: &'a Lookups) -> Self::Iter<'a>;
}

impl ChainExt for &[Task] {
  type Iter<'a> = AvLegalEndsIter<'a>;

  fn legal_before<'a>(&self, lu: &'a Lookups) -> Self::Iter<'a> {
    AvLegalEndsIter::new_before(self, lu)
  }

  fn legal_after<'a>(&self,  lu: &'a Lookups) -> Self::Iter<'a> {
    AvLegalEndsIter::new_after(self, lu)
  }
}
impl ChainExt for &[PvTask] {
  type Iter<'a> = PvLegalEndsIter<'a>;

  fn legal_before<'a>(&self,  lu: &'a Lookups) -> Self::Iter<'a> {
    PvLegalEndsIter::new_before(self, lu)
  }

  fn legal_after<'a>(&self,  lu: &'a Lookups) -> Self::Iter<'a> {
    PvLegalEndsIter::new_after(self, lu)
  }
}

// pub fn av_legal_before<'a>(data: &'a Data, tasks: &'a Tasks, chain: &[Task]) -> AvLegalEndsIter<'a> {
//   AvLegalEndsIter::new_before(chain, data, tasks)
// }
//
// pub fn av_legal_after<'a>(data: &'a Data, tasks: &'a Tasks, chain: &[Task]) -> AvLegalEndsIter<'a> {
//   AvLegalEndsIter::new_after(chain, data, tasks)
// }
//
// pub fn pv_legal_before<'a>(data: &'a Data, tasks: &'a Tasks, chain: &[PvTask]) -> PvLegalEndsIter<'a> {
//   PvLegalEndsIter::new_before(chain, data, tasks)
// }
//
// pub fn pv_legal_after<'a>(data: &'a Data, tasks: &'a Tasks, chain: &[PvTask]) -> PvLegalEndsIter<'a> {
//   PvLegalEndsIter::new_after(chain, data, tasks)
// }

pub struct AvLegalEndsIter<'a> {
  replace_idx: usize,
  chain_template: Vec<Task>,
  candidate_iter: std::slice::Iter<'a, Task>,
  lu: &'a Lookups,
}

impl<'a> AvLegalEndsIter<'a> {
  pub fn new_after(chain: &[Task], lu: &'a Lookups) -> Self {
    let candidate_iter = lu.tasks.succ[chain.last().unwrap()].iter();
    let mut chain_template = Vec::with_capacity(chain.len() + 1);
    chain_template.extend_from_slice(chain);
    chain_template.push(lu.tasks.ddepot); // placeholder
    // replace_idx is a valid index since Vec is non-empty
    AvLegalEndsIter { replace_idx: chain_template.len() - 1, chain_template, candidate_iter, lu }
  }

  pub fn new_before(chain: &[Task], lu: &'a Lookups) -> Self {
    let candidate_iter = lu.tasks.pred[chain.first().unwrap()].iter();
    let mut chain_template = Vec::with_capacity(chain.len() + 1);
    chain_template.push(lu.tasks.odepot); // placeholder
    chain_template.extend_from_slice(chain);
    // replace_idx is a valid index since Vec is non-empty
    AvLegalEndsIter { replace_idx: 0, chain_template, candidate_iter, lu }
  }
}

impl Iterator for AvLegalEndsIter<'_> {
  type Item = Task;

  fn next(&mut self) -> Option<Task> {
    let _s = trace_span!("av_legal_end_next", replace_idx=self.replace_idx).entered();
    while let Some(&task) = self.candidate_iter.next() {
      // Safety: replace_idx is a valid index because it is never modified, and is valid in the constructors.
      let task_to_replace = unsafe { self.chain_template.get_unchecked_mut(self.replace_idx) };
      *task_to_replace = task;
      if av_chain_full(&self.lu.data, &self.chain_template) {
        return Some(task)
      } else {
        trace!(?task, "illegal");
      }
    }
    None
  }
}


pub struct PvLegalEndsIter<'a> {
  replace_idx: usize,
  chain_template: Vec<PvTask>,
  candidate_iter: std::slice::Iter<'a, PvTask>,
  lu: &'a Lookups,
}


impl<'a> PvLegalEndsIter<'a> {
  pub fn new_after(chain: &[PvTask], lu: &'a Lookups) -> Self {
    let last_task = chain.last().unwrap();
    let candidate_iter = lu.tasks.pv_succ[last_task].iter();
    let mut chain_template = Vec::with_capacity(chain.len() + 1);
    chain_template.extend_from_slice(chain);
    chain_template.push(*last_task); // placeholder
    // replace_idx is a valid index since Vec is non-empty
    PvLegalEndsIter { replace_idx: chain_template.len() - 1, chain_template, candidate_iter, lu }
  }

  pub fn new_before(chain: &[PvTask], lu: &'a Lookups) -> Self {
    let first_task = chain.first().unwrap();
    let candidate_iter = lu.tasks.pv_pred[first_task].iter();
    let mut chain_template = Vec::with_capacity(chain.len() + 1);
    chain_template.push(*first_task); // placeholder
    chain_template.extend_from_slice(chain);
    // replace_idx is a valid index since Vec is non-empty
    PvLegalEndsIter { replace_idx: 0, chain_template, candidate_iter, lu }
  }
}


impl Iterator for PvLegalEndsIter<'_> {
  type Item = PvTask;

  fn next(&mut self) -> Option<Self::Item> {
    let _s = trace_span!("pv_legal_end_next", replace_idx=self.replace_idx).entered();
    while let Some(&task) = self.candidate_iter.next() {
      // Safety: replace_idx is a valid index because it is never modified, and is valid in the constructors.
      let task_to_replace = unsafe { self.chain_template.get_unchecked_mut(self.replace_idx) };
      *task_to_replace = task;
      if pv_chain_full(&self.lu.data, &self.chain_template) {
        return Some(task)
      }
    }
    None
  }
}
