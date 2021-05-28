pub mod cb;
pub mod sp;
pub mod mp;
pub mod sp_graph;

use crate::{Result, Task};
use smallvec::SmallVec;
use grb::constr::IneqExpr;
use grb::Expr;
use crate::model::cb::CutType;
//
// #[derive(Debug, Copy, Clone)]
// pub enum SpConstr {
//     AvTravelTime(Task, Task),
//     Loading(Task),
//     Unloading(Task),
//     Ub(Task),
//     Lb(Task),
// }
//
// pub type SpConstraints = SmallVec<[SpConstraints; 10]>;
//
// pub enum SpStatus<O,I> {
//     Optimal(O),
//     Infeasible(I),
// }
//
// pub trait SpSolve {
//     type OptInfo;
//     type InfInfo;
//
//     fn solve(&mut self) -> Result<SpStatus<Self::OptInfo, Self::InfInfo>>;
//     fn extract_and_remove_iis(&mut self, i: Self::InfInfo) -> Result<SpConstraints>;
//     fn extract_mrs(&self, o: Self::OptInfo) -> Result<SpConstraints>;
// }
//
// // fn add_vars(vars: &mp::MpVars, )
//
// pub fn build_infeasiblity_cut(vars: &mp::MpVars, spc: &SpConstraints) -> IneqExpr {
//     let expr = Expr::default();
//
//     for c in spc {
//         match c {
//
//         }
//     }
//     todo!()
// }
//
//
// pub fn build_optimality_cut(vars: &mp::MpVars, spc: &SpConstraints) -> IneqExpr {
//     todo!()
// }
//
//
// pub fn solve_subproblem_and_add_cuts<S: SpSolve>(solver: &mut S, cb: &mut cb::Cb) -> Result<()> {
//     loop {
//         match solver.solve()? {
//             SpStatus::Optimal(o) => {
//                 let mrs = solver.extract_mrs(o)?;
//                 let cut = build_optimality_cut(&cb.mp_vars, &mrs);
//                 cb.enqueue_cut(cut, CutType::LpOpt);
//                 break;
//             }
//             SpStatus::Infeasible(i) => {
//                 let iis = solver.extract_and_remove_iis(i)?;
//                 let cut = build_infeasiblity_cut(&cb.mp_vars, &iis);
//                 cb.enqueue_cut(cut, CutType::LpFeas);
//             }
//
//         }
//     }
//     Ok(())
// }