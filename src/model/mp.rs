use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use fnv::FnvHashSet;
use tracing::{info};

#[derive(Clone)]
pub struct MpVars {
  pub x: Map<Task, Var>,
  pub y: Map<(Av, Task, Task), Var>, /// These will be (Av, PvirTaskId, PvirTaskId)
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
  fn default() -> Self { ObjWeights { tt: 10.0, av_finish_time: 1.0, cover: 10_000.0 }}
}

impl MpVars {
  pub fn build(data: &Data, sets: &Sets, tasks: &Tasks, model: &mut Model, obj_param: ObjWeights) -> Result<Self> {
    let mut x = map_with_capacity(tasks.all.len());
    for &t in &tasks.all {
      let obj_coeff =
        if !t.is_depot() { obj_param.tt *( data.travel_cost[&(t.start, t.end)] as f64) }
        else { 0.0 };
      x.insert(t, add_binvar!(model, name: &format!("X[{:?}]", &t), obj: obj_coeff)?);
    }

    let n_yvars: usize = tasks.succ.values().map(|s| s.len()).sum();
    let mut y = map_with_capacity(n_yvars);

    for (&av, av_tasks) in &tasks.compat_with_av {
      for &t1 in av_tasks {
        for &t2 in &tasks.succ[&t1] {
          if av_tasks.contains(&t2) {
            let obj_coeff = obj_param.tt * (data.travel_cost[&(t1.end, t2.start)] as f64);
            y.insert((av, t1, t2), add_binvar!(model, name: &format!("Y[{:?}-{:?}|{}]", &t1, &t2, av))?);
          }
        }
      }
    }

    let mut u = map_with_capacity(data.n_req);
    for p in sets.req_pickups() {
      u.insert(p, add_binvar!(model, name: &format!("U[{}]", p), obj: obj_param.av_finish_time)?);
    }

    let mut theta = map_with_capacity(data.n_active * tasks.all.len());
    for a in sets.avs() {
      for &t in &tasks.compat_with_av[&a] {
        theta.insert((a, t), add_ctsvar!(model, name: &format!("Theta[{:?}|{}]", &t, a), obj: obj_param.cover)?);
      }
    }

    Ok(MpVars { x, y, theta, u })
  }
}

pub struct MpConstraints {
  pub req_cover: Map<Req, Constr>,
  pub pv_cover: Map<Pv, Constr>,
  pub pv_flow: Map<(Pv, Loc), Constr>,
  pub num_av: Map<Av, Constr>,
  pub av_flow: Map<(Av, Task), Constr>,
  pub xy_link: Map<Task, Constr>,
}

impl MpConstraints {
  pub fn build(data: &Data, sets: &Sets, tasks: &Tasks, model: &mut Model, vars: &MpVars) -> Result<Self> {
    let req_cover = {
      let mut cmap = map_with_capacity(data.n_req);
      for r in sets.req_pickups() {
        let xsum = tasks.by_cover[&r].iter().map(|t| vars.x[t]).grb_sum();
        let c = model.add_constr(&format!("req_cover[{}]", r), c!(xsum + vars.u[&r] == 1))?;
        cmap.insert(r, c);
      }
      cmap
    };

    let pv_cover = {
      let mut cmap = map_with_capacity(data.n_passive);
      for pv in sets.pv_origins() {
        let xsum = tasks.by_start[&pv].iter().map(|t| vars.x[t]).grb_sum();
        let c = model.add_constr(&format!("pv_cover[{}]", pv), c!(xsum == 1))?;
        cmap.insert(pv, c);
      }
      cmap
    };

    let pv_flow = {
      let mut cmap = map_with_capacity((data.n_loc - 2) * data.n_passive);
      for (&pv, pv_tasks) in &tasks.by_pv {
        for &rp in &data.compat_passive_req[&pv] {
          let rd = rp + data.n_req;
          for &i in &[rp, rd] {
            let lhs = tasks.by_start[&i].iter()
              .filter_map(|t| if pv_tasks.contains(t) { Some(vars.x[t]) } else { None })
              .grb_sum();

            let rhs = tasks.by_end[&i].iter()
              .filter_map(|t| if pv_tasks.contains(t) { Some(vars.x[t]) } else { None })
              .grb_sum();

            let c = model.add_constr(&format!("pv_flow[{}|{}]", i, pv), c!(lhs == rhs))?;
            cmap.insert((pv, i), c);
          }
        }
      }
      cmap
    };

    let num_av = { // ASSUMPTION this is only correct if there is only one vehicle in every vehicle class
      let mut cmap = map_with_capacity(data.n_active);
      for (&av, av_tasks) in &tasks.compat_with_av {
        let to = tasks.odepot;
        let ysum = tasks.succ[&to]
          .iter()
          .filter_map(|t| if av_tasks.contains(t) { Some(vars.y[&(av, to, *t)]) } else { None })
          .grb_sum();

        let c = model.add_constr(&format!("num_av[{}]", av), c!(ysum <= data.av_groups[&av].len()))?;
        cmap.insert(av, c);
      }
      cmap
    };


    let av_flow = {
      let mut cmap = Map::default(); // TODO capacity
      for (&av, av_tasks) in &tasks.compat_with_av {
        tracing::trace!(av, ?av_tasks);
        for &t1 in av_tasks {
          if t1.is_depot() { continue }

          let lhs = tasks.succ[&t1].iter()
            .filter_map(|&t2| vars.y.get(&(av, t1, t2)))
            .grb_sum();

          let rhs = tasks.pred[&t1].iter()
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
      let mut cmap = map_with_capacity(tasks.all.capacity());
      for &t2 in &tasks.all {
        if t2.is_depot() { continue }
        let ysum = tasks.pred[&t2].iter()
          .flat_map(|&t1| sets.avs().into_iter().map(move |av| (av, t1, t2)))
          .filter_map(|k| vars.y.get(&k))
          .grb_sum();
        let c = model.add_constr(&format!("xy_link[{:?}]", &t2), c!(ysum == vars.x[&t2]))?;
        cmap.insert(t2, c);
      }
      cmap
    };

    Ok(MpConstraints { req_cover, pv_cover, pv_flow, num_av, av_flow, xy_link })
  }
}

pub struct TaskModelMaster {
  pub vars: MpVars,
  pub cons: MpConstraints,
  pub model: Model,
  pub sp_env: Env,
}


impl TaskModelMaster {
  pub fn build(data: &Data, sets: &Sets, tasks: &Tasks, obj_param: ObjWeights) -> Result<Self> {
    let mut model = Model::new("Task Model MP")?;
    let vars = MpVars::build(data, sets, tasks, &mut model, obj_param)?;
    let cons = MpConstraints::build(data, sets, tasks, &mut model, &vars)?;

    // initial Benders Cuts
    for (&(av, t, td), &y) in vars.y.iter() {
      if td.ty == TaskType::DDepot {
        model.add_constr(&format!("initial_bc[{:?}|{}]", &t, av),c!(vars.theta[&(av, t)] >= (t.t_release + t.tt)*y ))?;
      }
    }

    let sp_env = {
      let mut e = Env::empty()?;
      e.set(param::OutputFlag, 0)?;
      e.start()?
    };

    Ok(TaskModelMaster { vars, cons, model, sp_env })
  }
}

