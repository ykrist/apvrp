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
    if t1.end != t2.start {
        EdgeConstrKind::AvTravelTime
    } else {
        if &t2.ty == &TaskType::Request {
            EdgeConstrKind::Loading
        } else if &t1.ty == &TaskType::Request {
            EdgeConstrKind::Unloading
        } else {
            unreachable!("bugalug")
        }
    }
}



#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[repr(u8)]
pub enum Phase {
    NoAvTTCost = 0,
    Final,
}

impl Phase {
    #[inline]
    pub fn next(&self) -> Self {
        use Phase::*;
        match self {
            NoAvTTCost => Final,
            _ => panic!("cannot call next() in Final phase"),
        }
    }

    pub fn idx(&self) -> u8 {
        *self as u8
    }

    #[inline]
    pub fn set_next(&mut self) {
        *self = self.next();
    }

    pub fn is_final(&self) -> bool {
        matches!(self, Phase::Final)
    }

    pub fn name(&self) -> String {
        let s = match self {
            Phase::NoAvTTCost => "no_avtt_cost_mip",
            Phase::Final => "final_mip",
        };
        s.to_string()
    }
}
