use crate::*;

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


fn impl_chain_cover_and_pred(chain: &[Task], cover_only: bool) -> bool {
  use TaskType::*;

  let mut locs_visited_by_chain = set_with_capacity(chain.len() * 2);

  for t in chain {
    // chain[k].end may be equal to chain[k+1].start
    locs_visited_by_chain.insert(t.start);
    if !locs_visited_by_chain.insert(t.end) {
      // Direct cover violation
      return false;
    }
  }

  if cover_only {
    return true;
  }

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
        lv.implied_before(Loc::Po(t.p.unwrap()))
          && lv.implied_after(Loc::Pd(t.p.unwrap())),

      Transfer =>
        lv.implied_before(Loc::Po(t.p.unwrap()))
          && lv.implied_before(t.start.origin())
          && lv.implied_after(t.end.dest())
          && lv.implied_after(Loc::Pd(t.p.unwrap())),
    };
    if !precedence_is_consistent {
      return false;
    }
  }

  true
}

use crate::utils::Permutator;
pub trait ChainIterExt {
  // pub fn legal_before(&self, data: &Data, tasks: &Tasks) -> ;
  fn permutations(&self) -> Permutator<Task>;
  fn legal_after<'a>(&'a self, data: &'a Data, tasks: &'a Tasks) -> LegalEndsIter<'a>;
  fn legal_before<'a>(&'a self, data: &'a Data, tasks: &'a Tasks) -> LegalEndsIter<'a>;
}

impl ChainIterExt for &[Task] {
  fn permutations(&self) -> Permutator<Task> {
    Permutator::new(self.to_vec())
  }

  fn legal_after<'a>(&'a self, data: &'a Data, tasks: &'a Tasks) -> LegalEndsIter<'a> {
    LegalEndsIter::new_after(self, data, tasks)
  }

  fn legal_before<'a>(&'a self, data: &'a Data, tasks: &'a Tasks) -> LegalEndsIter<'a> {
    LegalEndsIter::new_before(self, data, tasks)
  }
}

pub struct LegalEndsIter<'a> {
  replace_idx: usize,
  chain_template: Vec<Task>,
  candidate_iter: std::slice::Iter<'a, Task>,
  data: &'a Data,
}

impl<'a> LegalEndsIter<'a> {
  pub fn new_after(chain: &[Task], data: &'a Data, tasks: &'a Tasks) -> Self {
    let candidate_iter = tasks.succ[chain.last().unwrap()].iter();
    let mut chain_template = Vec::with_capacity(chain.len() + 1);
    chain_template.extend_from_slice(chain);
    chain_template.push(tasks.ddepot); // placeholder
    // replace_idx is a valid index since Vec is non-empty
    LegalEndsIter { replace_idx: chain_template.len() - 1, chain_template, candidate_iter, data }
  }

  pub fn new_before(chain: &[Task], data: &'a Data, tasks: &'a Tasks) -> Self {
    let candidate_iter = tasks.pred[chain.first().unwrap()].iter();
    let mut chain_template = Vec::with_capacity(chain.len() + 1);
    chain_template.push(tasks.odepot); // placeholder
    chain_template.extend_from_slice(chain);
    // replace_idx is a valid index since Vec is non-empty
    LegalEndsIter { replace_idx: 0, chain_template, candidate_iter, data }
  }
}

impl Iterator for LegalEndsIter<'_> {
  type Item = Task;

  fn next(&mut self) -> Option<Task> {
    while let Some(&task) = self.candidate_iter.next() {
      // Safety: replace_idx is a valid index because it is never modified, and is valid in the constructors.
      let task_to_replace = unsafe { self.chain_template.get_unchecked_mut(self.replace_idx) };
      *task_to_replace = task;
      if av_chain_full(self.data, &self.chain_template) {
        return Some(task)
      }
    }
    None
  }
}

