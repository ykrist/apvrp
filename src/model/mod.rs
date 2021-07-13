pub mod cb;
pub mod mp;
pub mod sp;

use crate::{Result, Task, Time, Sets, Avg, Map, TaskType, PvTask};
use smallvec::SmallVec;
use crate::model::cb::CutType;
use crate::constants::NUM_AV_UB;
use itertools::Itertools;
use grb::prelude::*;
use grb::constr::IneqExpr;

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum EdgeConstrKind {
    AvTravelTime,
    Loading,
    Unloading,
}
//
//
// impl EdgeConstr {
//     fn x_var(&self) -> &Task {
//         todo!()
//     }
//     fn y_var(&self) -> (&Task, &Task) {
//         (&self.0, &self.1)
//     }
// }

pub fn edge_constr_kind(t1: &Task, t2: &Task) -> EdgeConstrKind {
    if t1.end == t2.start {
        match &t1.ty {
            TaskType::Start | TaskType::Transfer => {
                debug_assert!(matches!(&t2.ty, TaskType::Request));
                EdgeConstrKind::Loading
            },
            TaskType::Request => {
                debug_assert!(matches!(&t2.ty, TaskType::End | TaskType::Transfer));
                EdgeConstrKind::Unloading
            },
            other => panic!("should not have a successor: task type {:?}", other)
        }
    } else {
        EdgeConstrKind::AvTravelTime
    }
}
