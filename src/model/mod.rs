pub mod cb;
pub mod sp_lp;
pub mod mp;
pub mod sp_graph;

use crate::{Result, Task, Time, Sets};
use smallvec::SmallVec;
use crate::model::cb::CutType;
use crate::constants::NUM_AV_UB;
use itertools::Itertools;
use grb::prelude::*;
use grb::constr::IneqExpr;

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum SpConstr {
    AvTravelTime(Task, Task),
    Loading(Task),
    Unloading(Task),
    Ub(Task),
    Lb(Task),
}

pub type SpConstraints = SmallVec<[SpConstr; 10]>;
pub type MrsInfo = SmallVec<[SpConstraints; NUM_AV_UB]>;

#[derive(Debug)]
pub enum SpStatus<O,I> {
    Optimal(Time, O),
    Infeasible(I),
}

impl<O, I> SpStatus<O, I> {
    fn is_optimal(&self) -> bool {
        matches!(self, SpStatus::Optimal(..))
    }
}

pub trait SpSolve {
    type OptInfo;
    type InfInfo;

    fn solve(&mut self) -> Result<SpStatus<Self::OptInfo, Self::InfInfo>>;
    fn extract_and_remove_iis(&mut self, i: Self::InfInfo) -> Result<SpConstraints>;
    fn extract_mrs(&self, o: Self::OptInfo) -> Result<MrsInfo>;
}

pub fn build_infeasiblity_cut(cb: &cb::Cb, spc: &SpConstraints) -> IneqExpr {
    let (k, iis_sum) = cb.mp_vars.sum_responsible(cb.sets, spc);
    c!(iis_sum <= k - 1)
}


pub fn build_optimality_cut(cb: &cb::Cb, spc: &SpConstraints, sp_obj: Time) -> IneqExpr {
    let (k, mrs_sum) = cb.mp_vars.sum_responsible(cb.sets, spc);

    let theta_sum = spc.iter()
      .filter_map(|c| match c {
          SpConstr::AvTravelTime(last_task, dd) =>
              if dd.is_depot() { Some(last_task) }
              else { None },
          _ => None,
      })
      .cartesian_product(cb.sets.avs())
      .filter_map(|(&t, a)| cb.mp_vars.theta.get(&(a, t)))
      .grb_sum();
    c!( sp_obj*(mrs_sum - k + 1) <= theta_sum)
}


pub fn solve_subproblem_and_add_cuts<S: SpSolve>(solver: &mut S, cb: &mut cb::Cb) -> Result<()> {
    loop {
        match solver.solve()? {
            SpStatus::Optimal(sp_obj, o) => {
                for spc in solver.extract_mrs(o)? {
                    let cut = build_optimality_cut(cb,  &spc, sp_obj);
                    cb.enqueue_cut(cut, CutType::LpOpt);
                }
                break;
            }
            SpStatus::Infeasible(i) => {
                let iis = solver.extract_and_remove_iis(i)?;
                let cut = build_infeasiblity_cut(cb, &iis);
                cb.enqueue_cut(cut, CutType::LpFeas);
            }
        }
    }
    Ok(())
}