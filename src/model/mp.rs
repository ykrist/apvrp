use crate::*;
use crate::tasks::TaskId;
use crate::logging::*;
use grb::prelude::*;
use fnv::FnvHashSet;
use crate::solution::Solution;
use crate::model::{EdgeConstrKind, edge_constr_kind};
use crate::utils::CollectExt;
use crate::experiment::{ApvrpExp, GurobiParamVal};

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
  pub tt: Cost,
  pub av_finish_time: Cost,
  pub cover: Cost,
}

impl std::default::Default for ObjWeights {
  fn default() -> Self { ObjWeights { tt: 10, av_finish_time: 1, cover: 10_000 } }
}

impl ObjWeights {
  pub fn without_av_finish_time(&self) -> Self {
    Self {
      av_finish_time: 0,
      ..*self
    }
  }
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
    for (&a, av_tasks) in &l.tasks.compat_with_av {
      for &t in av_tasks {
        if !t.is_depot() {
          theta.insert((a, t), add_ctsvar!(model, name: &format!("Theta[{:?}|{}]", &t, a))?);
        }
      }
    }

    let obj = add_ctsvar!(model, name: "Obj", obj: 1)?;
    Ok(MpVars { obj, x, y, theta, u })
  }

  pub fn binary_vars<'a>(&'a self) -> impl Iterator<Item=&'a Var> + 'a {
    self.x.values()
      .chain(self.y.values())
      .chain(self.u.values())
  }

  /// For task pair `(t1, t2)`, return a sum of variables which is 1 if the `(t1, t2)` edge constraint appears in the
  /// subproblem and 0 otherwise.
  #[instrument(level = "trace", skip(self, lu))]
  pub fn max_weight_edge_sum<'a>(&'a self, lu: &'a Lookups, t1: Task, t2: Task) -> impl Iterator<Item=Var> + 'a {
    use itertools::Either::*;

    fn iter_x_all_pv<'a>(lu: &'a Lookups, x: &'a Map<PvTask, Var>, t: Task) -> impl Iterator<Item=Var> + 'a {
      lu.tasks.task_to_pvtasks[&t].iter().map(move |t| x[t])
    }

    match edge_constr_kind(&t1, &t2) {
      EdgeConstrKind::Loading => {
        trace!(t=?t1, edge_ty=?EdgeConstrKind::Loading, "sum all PVs for task");
        Left(iter_x_all_pv(lu, &self.x, t1))
      }

      EdgeConstrKind::Unloading => {
        trace!(t=?t2, edge_ty=?EdgeConstrKind::Unloading, "sum all PVs for task");
        Left(iter_x_all_pv(lu, &self.x, t2))
      }

      EdgeConstrKind::AvTravelTime => {
        trace!(?t1, ?t2, edge_ty=?EdgeConstrKind::AvTravelTime, "all AVs for edge");
        Right(self.y_sum_av(lu, t1, t2))
      }
    }
  }

  pub fn x_sum_similar_tasks<'a>(&'a self, lu: &'a Lookups, t: PvTask) -> impl Iterator<Item=Var> + 'a {
    lu.tasks.pvtask_to_similar_pvtask[&t].iter()
      .map(move |t| self.x[t])
  }
  pub fn x_sum_all_task<'a>(&'a self, lu: &'a Lookups, t: Task) -> impl Iterator<Item=Var> + 'a {
    lu.tasks.task_to_pvtasks[&t].iter().map(move |t| self.x[t])
  }

  #[inline(always)]
  pub fn y_sum_av<'a>(&'a self, lu: &'a Lookups, t1: Task, t2: Task) -> impl Iterator<Item=Var> + 'a {
    trace!(?t1, ?t2, "y_sum_av");
    debug_assert_ne!(lu.sets.avs().filter_map(move |a| self.y.get(&(a, t1, t2))).count(), 0);
    self.y_sum_av_possibly_empty(lu, t1, t2)
  }

  #[inline(always)]
  pub fn y_sum_av_possibly_empty<'a>(&'a self, lu: &'a Lookups, t1: Task, t2: Task) -> impl Iterator<Item=Var> + 'a {
    lu.sets.avs()
      .filter_map(move |a| self.y.get(&(a, t1, t2)).copied())
  }
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
  pub initial_cuts: Map<(Av, Task), Constr>,
  pub initial_lifted_cuts: Map<(Av, Task), Constr>,
  pub whole_route_cuts: Map<Av, Constr>,
}

impl MpConstraints {
  pub fn build(lu: &Lookups, model: &mut Model, vars: &MpVars) -> Result<Self> {
    let req_cover = {
      let mut cmap = map_with_capacity(lu.data.n_req as usize);
      for r in lu.sets.reqs() {
        trace!(r);
        let xsum = lu.tasks.by_req_cover[&r].iter()
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

        let ysum = lu.data.compat_passive_active[&pt.p].iter()
          .cartesian_product(&lu.tasks.pred[&t])
          .filter_map(|(&a, &td)| vars.y.get(&(a, td, t)))
          .grb_sum();

        let c = model.add_constr(&format!("av_pv_compat[{:?}]", pt), c!(x <= ysum))?;
        cmap.insert(pt, c);
      }
      cmap
    };

    let obj = model.add_constr("Obj", c!(-vars.obj == 0))?;


    // initial Benders Cuts
    // let initial_cuts : Map<_, _> = vars.theta.iter()
    //   .map(|(&(av, t), &theta)| {
    //     let y = vars.y[&(av, t, lu.tasks.ddepot)];
    //     let c = model.add_constr(&format!("initial_bc[{:?}|{}]", &t, av), c!(theta >= t.t_release * y))?;
    //     Ok(((av, t), c))
    //   })
    //   .collect_ok()?;
    let initial_cuts = Default::default();
    // // initial Benders Cuts
    // let initial_lifted_cuts : Map<_, _> = vars.theta.iter()
    //   .map(|(&(av, t2), &theta)| {
    //     let y_t2_depot = vars.y[&(av, t2, lu.tasks.ddepot)];
    //     let finish_t2_dd = t2.t_release;
    //
    //     let mut rhs = y_t2_depot * finish_t2_dd;
    //
    //     for (t1, y_t1_t2) in lu.tasks.pred[&t2].iter().filter_map(|&t1| vars.y.get(&(av, t1, t2)).map(|&y| (t1, y))) {
    //       if t1.is_depot() {
    //         continue
    //       }
    //       let route = [t1, t2];
    //       trace!(?route);
    //       let finish_t1_t2_dd = schedule::av_route_finish_time(lu, &route);
    //       let coeff = finish_t1_t2_dd - finish_t2_dd;
    //       // (y_t1_t2 + y_t2_depot - 1) * coeff
    //       rhs += coeff*y_t2_depot;
    //       rhs += coeff*y_t1_t2;
    //       rhs += -coeff;
    //     }
    //
    //     let c = model.add_constr(&format!("initial_lifted[{:?}|{}]", &t2, av), c!(theta >= rhs))?;
    //     Ok(((av, t2), c))
    //   })
    //   .collect_ok()?;
    let initial_lifted_cuts = Default::default();
    // { // At least one av route must finish on a Direct or End task
    //   let lhs = vars.y.iter()
    //     .filter(|((av, t1, t2), _) | t2.is_depot() && matches!(&t1.ty, TaskType::Direct | TaskType::End))
    //     .map(|(_, &y)| y)
    //     .grb_sum();
    //   model.add_constr("", c!(lhs >= 1))?;
    // }

    let whole_route_cuts: Map<_, _> = lu.sets.avs().map(|av| {
      let _s = trace_span!("whole_route_cuts", av).entered();
      let mut cut = Expr::default();

      for &t1 in &lu.tasks.compat_with_av[&av] {
        for &t2 in &lu.tasks.succ[&t1] {
          if let Some(&y) = vars.y.get(&(av, t1, t2)) {
            if t1.is_depot() {
              cut += t2.t_release * y;
            } else if t2.is_depot() {
              // handled already
            } else {
              let travel_time = t1.tt + lu.data.travel_time[&(t1.end, t2.start)];
              let service_time = if t1.end == t2.start {
                trace!(?t1, ?t2, s=?lu.data.srv_time.get(&t1.end));
                debug_assert_eq!(t1.tt, travel_time);
                lu.data.srv_time.get(&t1.end).copied().unwrap_or(0)
                // 0
              } else {
                0
              };
              let latest_arrival_at_t2 = t1.t_deadline + travel_time + service_time;
              let minimum_waiting_time = std::cmp::max(t2.t_release - latest_arrival_at_t2, 0);
              cut += (travel_time + service_time + minimum_waiting_time) * y;
            }
          }
        }
      }
      let theta_sum = vars.theta.iter()
        .filter(|((a, _), _)| a == &av)
        .map(|(_, &theta)| theta)
        .grb_sum();

      let c = model.add_constr(&format!("WholeRoute[{}]", av), c!(theta_sum >= cut))?;
      Ok((av, c))
    })
      .collect_ok()?;

    Ok(MpConstraints {
      obj,
      req_cover,
      pv_cover,
      pv_flow,
      num_av,
      av_flow,
      xy_link,
      av_pv_compat,
      initial_cuts,
      initial_lifted_cuts,
      whole_route_cuts,
    })
  }
}

pub struct TaskModelMaster {
  pub vars: MpVars,
  pub cons: MpConstraints,
  pub model: Model,
}


impl TaskModelMaster {
  pub fn relax_integrality(&mut self) -> Result<()> {
    for var in self.vars.binary_vars() {
      self.model.set_obj_attr(attr::VType, var, VarType::Continuous)?
    }
    Ok(())
  }

  pub fn enforce_integrality(&mut self) -> Result<()> {
    for var in self.vars.binary_vars() {
      self.model.set_obj_attr(attr::VType, var, VarType::Binary)?
    }
    Ok(())
  }

  pub fn obj_val(&self) -> Result<Cost> {
    Ok(self.model.get_attr(attr::ObjVal)?.round() as Cost)
  }

  pub fn obj_bound(&self) -> Result<Cost> {
    Ok(self.model.get_attr(attr::ObjBound)?.round() as Cost)
  }

  pub fn build(lu: &Lookups) -> Result<Self> {
    let mut model = Model::new("Task Model MP")?;
    let vars = MpVars::build(lu, &mut model)?;
    let cons = MpConstraints::build(lu, &mut model, &vars)?;

    Ok(TaskModelMaster { vars, cons, model })
  }

  pub fn set_objective(&mut self, lu: &Lookups, obj_param: ObjWeights) -> Result<()> {
    let vars = &self.vars;
    let model = &mut self.model;
    let constr = self.cons.obj;

    let new_coeffs = vars.x.iter().map(|(t, &var)|
      (var, obj_param.tt * lu.data.travel_cost[&(t.start, t.end)])
    ).chain(
      vars.y.iter().map(|((_, t1, t2), &var)| {
        let mut obj = obj_param.tt * lu.data.travel_cost[&(t1.end, t2.start)];
        if t2.is_depot() {
          obj += obj_param.av_finish_time * lu.data.travel_time_to_ddepot(t1)
        }
        (var, obj)
      })
    ).chain(
      vars.u.values().map(|&var| (var, obj_param.cover))
    ).chain(
      vars.theta.values().map(|&var| (var, obj_param.av_finish_time))
    );

    model.set_coeffs(new_coeffs.map(|(var, coeff)| (var, constr, coeff as f64)))?;
    Ok(())
  }

  #[tracing::instrument(level = "error", skip(self, sol))]
  pub fn fix_solution(&mut self, sol: &Solution) -> Result<()> {
    for r in &sol.pv_routes {
      trace!(?r, "fix PV route");
      self.model.set_obj_attr_batch(attr::LB, r.iter().map(|t| (self.vars.x[t], 1.0)))?;
    }

    for r in &sol.av_routes {
      trace!(a=r.av(), r=?(&*r), "fix AV route");
      for key in r.iter_y_vars() {
        if let Some(y) = self.vars.y.get(&key) {
          self.model.set_obj_attr(attr::LB, y, 1.0)?;
        } else {
          let (a, t1, t2) = key;
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

  pub fn print_constraint_at_current_sol(&self, c: &grb::Constr) -> Result<()> {
    use std::fmt::Write;
    let mut s = String::new();

    for var in self.model.get_vars()? {
      let coeff = self.model.get_coeff(var, c)?;
      if coeff.abs() > 1e-4 {
        let x = self.model.get_obj_attr(attr::X, var)?;
        if x.abs() > 1e-4 {
          let name = self.model.get_obj_attr(attr::VarName, var)?;
          let sgn = if coeff.is_sign_negative() { '-' } else { '+' };
          write!(s, " {} {} * ({} = {})", sgn, coeff.abs(), name, x)?;
        }
      }
    }

    match self.model.get_obj_attr(attr::Sense, c)? {
      ConstrSense::Equal => s.push_str(" == "),
      ConstrSense::Less => s.push_str(" <= "),
      ConstrSense::Greater => s.push_str(" >= "),
    }
    write!(s, " {}", self.model.get_obj_attr(attr::RHS, c)?)?;
    println!("{}", s);
    Ok(())
  }


  pub fn apply_gurobi_parameters(&mut self, exp: &ApvrpExp) -> Result<()> {
    use grb::parameter::Undocumented;

    for (pname, val) in &exp.parameters.gurobi {
      let param = Undocumented::new(pname.clone())?;
      match val {
        GurobiParamVal::Int(v) =>
          self.model.set_param(&param, *v)?,
        GurobiParamVal::Dbl(v) =>
          self.model.set_param(&param, *v)?,
      }
    }
    Ok(())
  }
}

