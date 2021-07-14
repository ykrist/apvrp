use crate::*;
use crate::solution::Solution;
use super::{cb, cb::CutType};
use smallvec::SmallVec;
use grb::constr::IneqExpr;
use crate::model::sp::XEdgeConstraint::Loading;
use crate::utils::iter_pairs;

pub mod lp;
pub mod dag;

#[derive(Debug, Clone)]
pub enum Iis {
  /// Includes first task LB, last task UB, tasks must be in time-order
  Path {
    lb: PvTask,
    ub: PvTask,
    path: SmallVec<[Task; 10]>,
  },
  /// Tasks should be ordered according to the cycle
  Cycle(SmallVec<[Task; 10]>),
}

#[derive(Debug)]
pub enum SpStatus<O, I> {
  Optimal(Time, O),
  Infeasible(I),
}

impl<O, I> SpStatus<O, I> {
  fn is_optimal(&self) -> bool {
    matches!(self, SpStatus::Optimal(..))
  }
}

pub trait Subproblem<'a>: Sized {
  // Additional information obtained when solving the subproblem to optimality
  type Optimal;
  // Additional information obtained when proving the subproblem to be infeasible
  type Infeasible;
  // A group of constraint sets representing one or more IIS
  type IisConstraintSets: IntoIterator<Item=Iis>;

  fn build(lu: &'a Lookups, sol: &'a Solution) -> Result<Self>;

  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus<Self::Optimal, Self::Infeasible>>;
  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self, i: Self::Infeasible) -> Result<Self::IisConstraintSets>;
  // Find and remove one or more MRS when optimal
  fn add_optimality_cuts(&self, cb: &mut cb::Cb, o: Self::Optimal) -> Result<()>;

  fn solve_subproblem_and_add_cuts(&mut self, cb: &mut cb::Cb, theta: &Map<(Avg, Task), Time>) -> Result<()> {
    loop {
      match self.solve()? {
        SpStatus::Optimal(sp_obj, o) => {
          self.add_optimality_cuts(cb, o)?;
          break;
        }
        SpStatus::Infeasible(i) => {
          for iis in self.extract_and_remove_iis(i)?.into_iter() {
            let cut = match iis {
              Iis::Path { lb, ub, path } =>
                build_path_infeasiblity_cut(cb, lb, ub, &path),
              Iis::Cycle(cycle) => build_cyclic_infeasiblity_cut(cb, &cycle),
            };
            cb.enqueue_cut(cut, CutType::LpFeas); // FIXME cuttype should be changed
          }
        }
      }
    }
    Ok(())
  }
}

pub fn build_path_infeasiblity_cut(cb: &cb::Cb, lb_task: PvTask, ub_task: PvTask, path: &[Task]) -> IneqExpr {
  // for (t1, t2) in
  todo!()
}

pub fn build_cyclic_infeasiblity_cut(cb: &cb::Cb, cycle: &[Task]) -> IneqExpr {
  todo!()
}

// FIXME: this should live in the sp_lp module, wont need it with daggylp
// pub fn build_optimality_cut(cb: &cb::Cb, spc: &SpConstraints, sp_obj: Time) -> IneqExpr {
// let (k, mrs_sum) = cb.mp_vars.sum_responsible(cb.sets, spc);
//
// let theta_sum = spc.iter()
//   .filter_map(|c| match c {
//       SpConstr::AvTravelTime(last_task, dd) =>
//           if dd.is_depot() { Some(last_task) }
//           else { None },
//       _ => None,
//   })
//   .cartesian_product(cb.sets.avs())
//   .filter_map(|(&t, a)| cb.mp_vars.theta.get(&(a, t)))
//   .grb_sum();
// c!( sp_obj*(mrs_sum - k + 1) <= theta_sum)
// todo!()
// }

#[derive(Copy, Clone, Debug)]
enum XEdgeConstraint<'a> {
  Loading(&'a PvTask, &'a PvTask),
  Unloading(&'a PvTask, &'a PvTask),
}

fn iter_edge_constraints<'a>(pv_route: &'a [PvTask]) -> impl Iterator<Item=XEdgeConstraint<'a>> + 'a {
  iter_pairs(pv_route).filter_map(|(t1, t2)| {
    if matches!(&t2.ty, TaskType::Request) {
      debug_assert_matches!(&t1.ty, TaskType::Start | TaskType::Transfer);
      Some(XEdgeConstraint::Loading(t1, t2))
    } else if matches!(&t1.ty, TaskType::Request) {
      debug_assert_matches!(&t2.ty, TaskType::End | TaskType::Transfer);
      Some(XEdgeConstraint::Unloading(t1, t2))
    } else {
      None
    }
  })
}
