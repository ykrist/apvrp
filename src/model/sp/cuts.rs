use super::*;

#[derive(Copy, Clone, Debug)]
pub struct IisCut;

impl GenIisCut for IisCut {
    fn cut(cb: &mut cb::Cb, iis: &Iis) -> Result<()> {
        todo!()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct MrsCut;

impl GenOptimalityCut for MrsCut {
    fn cut(cb: &mut cb::Cb, subproblem: &mut GraphModel, constr: &Set<SpConstr>) -> Result<()> {
        todo!()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct MrsTreeCut;

impl GenOptimalityCut for MrsTreeCut {
    fn cut(cb: &mut cb::Cb, subproblem: &mut GraphModel, constr: &Set<SpConstr>) -> Result<()> {
        todo!()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct MrsPathCut;

impl GenOptimalityCut for MrsPathCut {
    fn cut(cb: &mut cb::Cb, subproblem: &mut GraphModel, constr: &Set<SpConstr>) -> Result<()> {
        todo!()
    }
}

#[derive(Copy, Clone, Debug)]
pub struct CriticalPathCut;

impl GenOptimalityCut for CriticalPathCut {
    fn cut(cb: &mut cb::Cb, subproblem: &mut GraphModel, constr: &Set<SpConstr>) -> Result<()> {
        todo!()
    }
}