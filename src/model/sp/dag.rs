use daggylp::{Var, Graph, SolveStatus};

use crate::*;
use crate::model::sp::{Iis, Subproblem, SpStatus};
use crate::model::cb;

struct GraphModel {
  vars: Map<Var, Task>,
  edges: Map<(Var, Var), (Task, Task)>,
  model: Graph,
}

impl Subproblem for GraphModel {
  // Additional information obtained when solving the subproblem to optimality
  type OptInfo = ();
  // Additional information obtained when proving the subproblem to be infeasible
  type InfInfo = ();
  // A group of constraint sets representing one or more IIS
  type IisConstraintSets = std::iter::Once<Iis>;

  // Solve the subproblem and return status
  fn solve(&mut self) -> Result<SpStatus<Self::OptInfo, Self::InfInfo>> {
    let s = match self.model.solve() {
      SolveStatus::Infeasible(_) => SpStatus::Infeasible(()),
      SolveStatus::Optimal => SpStatus::Optimal(self.model.compute_obj()? as Time, ())
    };
    Ok(s)
  }
  // Find and remove one or more IIS when infeasible
  fn extract_and_remove_iis(&mut self, i: Self::InfInfo) -> Result<Self::IisConstraintSets> {
    let iis = self.model.compute_iis(true);

    todo!()
  }

  // Find and remove one or more MRS when optimal
  fn add_optimality_cuts(&self, cb: &mut cb::Cb, o: Self::OptInfo) -> Result<()> {
    todo!()
  }
}