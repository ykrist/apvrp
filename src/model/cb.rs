use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use grb::callback::{Callback, Where, CbResult};
use fnv::FnvHashSet;
use super::sp::{BendersCut, TimingSubproblem};

pub struct CbStats {
  pub n_opt_bc: usize,
  pub n_feas_bc: usize,
}

pub struct Cb<'a> {
  data: &'a ApvrpInstance,
  sets: &'a Sets,
  tasks: &'a Tasks,
  mp: &'a TaskModelMaster,
  stats: CbStats,
}

pub fn construct_av_routes() -> Vec<Vec<TaskId>>{
  todo!()
}

pub fn construct_pv_routes() -> Vec<Vec<TaskId>> {
  todo!()
}


impl<'a> Callback for Cb<'a> {
  fn callback(&mut self, w: Where) -> CbResult {
    match w {
      Where::MIPSol(ctx) => {
        let av_routes = construct_av_routes();
        let pv_routes = construct_pv_routes();
        let sp = TimingSubproblem::build(self.data, self.tasks, &av_routes, &pv_routes)?;
        let cuts = sp.solve(self.tasks)?;

        debug_assert_eq!(cuts.len(), 1); // TODO remove this when multiple cuts are added
        for cut in cuts {
          match &cut {
            BendersCut::Optimality(_) => {self.stats.n_opt_bc += 1},
            BendersCut::Feasibility(_) => {self.stats.n_feas_bc += 1},
          }
          let cut = cut.into_ineq(self.sets, self.tasks, &self.mp.vars);
          ctx.add_lazy(cut)?;
        }
      }
      _ => {}
    }
    Ok(())
  }
}
