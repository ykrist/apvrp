use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use fnv::FnvHashSet;
use tracing::{info, trace, error, debug};
use crate::solution::Solution;

#[derive(Clone)]
pub struct MpVars {
  pub obj: Var,
  pub x: Map<PvTask, Var>,
  pub y: Map<(Av, Task, Task), Var>,
  pub u: Map<Req, Var>,
  pub theta: Map<(Av, Task), Var>,
}

#[derive(Debug, Copy, Clone)]
pub struct ObjWeights {
  pub tt: f64,
  pub av_finish_time: f64,
  pub cover: f64,
}

impl std::default::Default for ObjWeights {
  fn default() -> Self { ObjWeights { tt: 10.0, av_finish_time: 1.0, cover: 10_000.0 } }
}

impl MpVars {
  pub fn build(l: &Lookups, model: &mut Model) -> Result<Self> {
    let mut x = map_with_capacity(l.tasks.pvtask_to_task.len());
    for &t in l.tasks.pvtask_to_task.keys() {
      x.insert(t, add_binvar!(model, name: &format!("X[{:?}]", &t))?);
    }

    let n_yvars: usize = l.tasks.succ.values().map(|s| s.len()).sum();
    let mut y = map_with_capacity(n_yvars);

    for (&av, av_tasks) in &l.tasks.compat_with_av {
      for &t1 in av_tasks {
        for &t2 in &l.tasks.succ[&t1] {
          if av_tasks.contains(&t2) {
            y.insert((av, t1, t2), add_binvar!(model, name: &format!("Y[{:?}-{:?}|{}]", &t1, &t2, av))?);
          }
        }
      }
    }

    let mut u = map_with_capacity(l.data.n_req as usize);
    for r in l.sets.reqs() {
      u.insert(r, add_binvar!(model, name: &format!("U[{}]", r))?);
    }

    let mut theta = map_with_capacity(l.data.n_active as usize * l.tasks.all.len());
    for a in l.sets.avs() {
      for &t in &l.tasks.compat_with_av[&a] {
        theta.insert((a, t), add_ctsvar!(model, name: &format!("Theta[{:?}|{}]", &t, a))?);
      }
    }

    let obj = add_ctsvar!(model, name: "Obj", obj: 1)?;

    Ok(MpVars { obj, x, y, theta, u })
  }


  /// For task pair `(t1, t2)`, return a sum of variables which is 1 if the `(t1, t2)` edge constraint appears in the
  /// subproblem and 0 otherwise.
  pub fn max_weight_edge_sum<'a>(&'a self, lu: &'a Lookups, t1: Task, t2: Task) -> impl Iterator<Item=Var> + 'a {
    use itertools::Either::*;

    fn iter_x_all_pv<'a>(lu: &'a Lookups, x: &'a Map<PvTask, Var>, t: Task) -> impl Iterator<Item=Var> + 'a {
      lu.tasks.task_to_pvtasks[&t].iter().map(move |t| x[t])
    }

    if &t2.ty == &TaskType::Request {
      Left(iter_x_all_pv(lu, &self.x, t1))
    } else if &t1.ty == &TaskType::Request {
      Left(iter_x_all_pv(lu, &self.x, t2))
    } else {
      Right(lu.sets.avs().filter_map(move |a| self.y.get(&(a, t1, t2)).copied()))
    }
  }

  // FIXME: this should go into sp_lp, it is just the no-good cut
  // / Construct a expression of variables which is at most `k` (the other return value),
  // /// If the expression is equal to `k` then the supplied constraints are guaranteed to appear in the subproblem.
  // pub fn sum_responsible(&self, sets: &Sets, sp_constraints: &[super::SpConstr]) -> (usize, Expr) {
  //   use super::SpConstr::*;
  //
  //   let mut x_tasks = set_with_capacity(sp_constraints.len());
  //   let mut y_tasks = set_with_capacity(sp_constraints.len());
  //
  //   let mut expr = Expr::default();
  //   let mut k = 0;
  //
  //   for &c in sp_constraints {
  //     match c {
  //       Unloading(t) | Lb(t) | Ub(t) | Loading(t) => {
  //         x_tasks.insert(t);
  //       }
  //       AvTravelTime(t1, t2) => {
  //         k += 1;
  //         for a in sets.avs() {
  //           if let Some(&y) = self.y.get(&(a, t1, t2)) {
  //             expr += y;
  //           }
  //         }
  //         y_tasks.insert(t1);
  //         y_tasks.insert(t2);
  //       }
  //     }
  //   }
  //
  //   for t in x_tasks.difference(&y_tasks) {
  //     k += 1;
  //     expr += self.x[t]
  //   }
  //
  //   (k, expr)
  // }
}

pub struct MpConstraints {
  pub obj: Constr,
  pub req_cover: Map<Req, Constr>,
  pub pv_cover: Map<Pv, Constr>,
  pub pv_flow: Map<(Pv, Loc), Constr>,
  pub num_av: Map<Av, Constr>,
  pub av_flow: Map<(Av, Task), Constr>,
  pub xy_link: Map<Task, Constr>,
  pub av_pv_compat: Map<PvTask, Constr>,
  // TODO add trivial lb on theta here
}

impl MpConstraints {
  pub fn build(lu: &Lookups, model: &mut Model, vars: &MpVars, obj_param: &ObjWeights) -> Result<Self> {
    let req_cover = {
      let mut cmap = map_with_capacity(lu.data.n_req as usize);
      for r in lu.sets.reqs() {
        trace!(r);
        let xsum = lu.tasks.by_cover[&r].iter()
          .flat_map(|t| lu.tasks.task_to_pvtasks[t].iter())
          .map(|t| vars.x[t])
          .grb_sum();
        let c = model.add_constr(&format!("req_cover[{}]", r), c!(xsum + vars.u[&r] == 1))?;
        cmap.insert(r, c);
      }
      cmap
    };

    let pv_cover = {
      let mut cmap = map_with_capacity(lu.data.n_passive as usize);
      for po in lu.sets.pv_origins() {
        let xsum = lu.tasks.by_start[&po].iter()
          .flat_map(|t| lu.tasks.task_to_pvtasks[t].iter()) // should be one-to-one for these tasks
          .map(|t| vars.x[t])
          .grb_sum();
        let c = model.add_constr(&format!("pv_cover[{}]", po.pv()), c!(xsum == 1))?;
        cmap.insert(po.pv(), c);
      }
      cmap
    };

    let pv_flow = {
      let mut cmap = map_with_capacity((lu.data.n_loc as usize - 2) * lu.data.n_passive as usize);
      for (&r, pvs) in &lu.data.compat_req_passive {
        let rp = Loc::ReqP(r);
        let rd = Loc::ReqD(r);
        for &p in pvs {
          for &i in &[rp, rd] {
            let lhs = lu.tasks.by_start[&i].iter()
              .filter_map(|&t| lu.tasks.pvtask.get(&(p, t)))
              .map(|t| vars.x[t])
              .grb_sum();

            let rhs = lu.tasks.by_end[&i].iter()
              .filter_map(|&t| lu.tasks.pvtask.get(&(p, t)))
              .map(|t| vars.x[t])
              .grb_sum();

            let c = model.add_constr(&format!("pv_flow[{},{}]", p, i), c!(lhs == rhs))?;
            cmap.insert((p, i), c);
          }
        }
      }
      cmap
    };

    let num_av = {
      let mut cmap = map_with_capacity(lu.data.n_active as usize);
      for (&av, av_tasks) in &lu.tasks.compat_with_av {
        let to = lu.tasks.odepot;
        let ysum = lu.tasks.succ[&to]
          .iter()
          .filter_map(|t| if av_tasks.contains(t) { Some(vars.y[&(av, to, *t)]) } else { None })
          .grb_sum();

        let c = model.add_constr(&format!("num_av[{}]", av), c!(ysum <= lu.data.av_groups[&av].len()))?;
        cmap.insert(av, c);
      }
      cmap
    };


    let av_flow = {
      let mut cmap = Map::default(); // TODO capacity
      for (&av, av_tasks) in &lu.tasks.compat_with_av {
        tracing::trace!(av, ?av_tasks);
        for &t1 in av_tasks {
          if t1.is_depot() { continue; }

          let lhs = lu.tasks.succ[&t1].iter()
            .filter_map(|&t2| vars.y.get(&(av, t1, t2)))
            .grb_sum();

          let rhs = lu.tasks.pred[&t1].iter()
            .filter_map(|&t2| vars.y.get(&(av, t2, t1)))
            .grb_sum();

          debug_assert!(!rhs.clone().into_linexpr().unwrap().is_empty());
          debug_assert!(!lhs.clone().into_linexpr().unwrap().is_empty());

          let c = model.add_constr(&format!("av_flow[{:?}|{}]", &t1, av), c!(lhs == rhs))?;
          cmap.insert((av, t1), c);
        }
      }
      cmap
    };

    let xy_link = {
      let mut cmap = map_with_capacity(lu.tasks.all.len());
      for &t2 in &lu.tasks.all {
        if t2.is_depot() { continue; }
        let ysum = lu.tasks.pred[&t2].iter()
          .flat_map(|&t1| lu.sets.avs().into_iter().map(move |av| (av, t1, t2)))
          .filter_map(|k| vars.y.get(&k))
          .grb_sum();
        let xsum = lu.tasks.task_to_pvtasks[&t2].iter()
          .filter_map(|t| vars.x.get(t))
          .copied()
          .grb_sum();
        let c = model.add_constr(&format!("xy_link[{:?}]", &t2), c!(ysum == xsum))?;
        cmap.insert(t2, c);
      }
      cmap
    };

    let av_pv_compat = {
      let mut cmap = map_with_capacity(lu.tasks.all.len() * lu.sets.pvs().len());
      for (&pt, &x) in &vars.x {
        let t = lu.tasks.pvtask_to_task[&pt];

        let ysum =lu.data.compat_passive_active[&pt.p].iter()
          .cartesian_product(&lu.tasks.pred[&t])
          .filter_map(|(&a, &td)| vars.y.get(&(a, td, t)))
          .grb_sum();

        let c = model.add_constr(&format!("av_pv_compat[{:?}]", pt), c!(x <= ysum))?;
        cmap.insert(pt, c);
      }
      cmap
    };

    let obj = {
      let mut obj_expr = Expr::default();

      for (t, &x) in &vars.x {
        let obj = if !t.is_depot() {
          obj_param.tt * (lu.data.travel_cost[&(t.start, t.end)] as f64)
        } else { 0.0 };
        obj_expr += obj * x;
      }

      for ((_, t1, t2), &y) in &vars.y {
        let obj = obj_param.tt * (lu.data.travel_cost[&(t1.end, t2.start)] as f64);
        obj_expr += obj * y;
      }

      for (_, &u) in &vars.u {
        obj_expr += obj_param.cover * u;
      }

      for (&(a, t), &theta) in &vars.theta {
        obj_expr += obj_param.av_finish_time * theta;
        let last_lil_bit = (t.tt + lu.data.travel_cost[&(t.end, Loc::Ad)]) as f64;
        obj_expr += obj_param.av_finish_time * last_lil_bit * vars.y[&(a, t, lu.tasks.ddepot)];
      }

      model.add_constr("Obj", c!(vars.obj == obj_expr))?
    };
    Ok(MpConstraints { obj, req_cover, pv_cover, pv_flow, num_av, av_flow, xy_link, av_pv_compat })
  }
}

pub struct TaskModelMaster {
  pub vars: MpVars,
  pub cons: MpConstraints,
  pub model: Model,
  pub obj_param: ObjWeights,
}


impl TaskModelMaster {
  pub fn build(lu: &Lookups, obj_param: ObjWeights) -> Result<Self> {
    let mut model = Model::new("Task Model MP")?;
    let vars = MpVars::build(lu, &mut model)?;
    let cons = MpConstraints::build(lu, &mut model, &vars, &obj_param)?;

    // initial Benders Cuts
    for (&(av, t, td), &y) in vars.y.iter() {
      if td.ty == TaskType::DDepot {
        let tt_d = lu.data.travel_time[&(t.end, td.start)];
        model.add_constr(&format!("initial_bc[{:?}|{}]", &t, av), c!(vars.theta[&(av, t)] >= (t.t_release + t.tt + tt_d)*y ))?;
      }
    }

    Ok(TaskModelMaster { vars, cons, model, obj_param })
  }

  #[tracing::instrument(level = "error", skip(self, sol))]
  pub fn fix_solution(&mut self, sol: &Solution) -> Result<()> {
    for (p, r) in &sol.pv_routes {
      trace!(p, ?r, "fix PV route");
      self.model.set_obj_attr_batch(attr::LB, r.iter().map(|t| (self.vars.x[t], 1.0)))?;
    }

    for (a, r) in &sol.av_routes {
      trace!(a, ?r, "fix AV route");
      for (t1, t2) in r.iter().tuple_windows() {
        if let Some(y) = self.vars.y.get(&(*a, *t1, *t2)) {
          self.model.set_obj_attr(attr::LB, y, 1.0)?;
        } else {
          error!(?a, ?t1, ?t2, "missing task-task connection");
          panic!("bugalug")
        }
      }
    }
    if let Some(obj) = sol.objective {
      self.model.set_obj_attr(attr::UB, &self.vars.obj, obj as f64)?;
      self.model.set_obj_attr(attr::LB, &self.vars.obj, obj as f64)?;
    }
    Ok(())
  }

  pub fn unfix_solution(&mut self) -> Result<()> {
    debug!("unfix solution");
    self.model.set_obj_attr_batch(attr::LB,
                                  self.vars.x.values().copied()
                                    .chain(self.vars.y.values().copied())
                                    .map(|var| (var, 0.0)),
    )?;
    self.model.set_obj_attr(attr::UB, &self.vars.obj, grb::INFINITY)?;
    self.model.set_obj_attr(attr::LB, &self.vars.obj, 0.0)?;
    Ok(())
  }
}

