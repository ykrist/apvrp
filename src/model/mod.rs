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


pub fn edge_constr_kind(t1: &Task, t2: &Task) -> EdgeConstrKind {
    if &t2.ty == &TaskType::Request {
        EdgeConstrKind::Loading
    } else if &t1.ty == &TaskType::Request {
        EdgeConstrKind::Unloading
    } else {
        EdgeConstrKind::AvTravelTime
    }
}
