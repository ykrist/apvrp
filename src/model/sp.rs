use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use super::mp::MpVars;

pub struct SpConstraints {
  av_sync: Map<(TaskId, TaskId), Constr>,
  start_req: Map<TaskId, Constr>,
  end_req: Map<TaskId, Constr>,
  lb: Map<TaskId, Constr>,
  ub: Map<TaskId, Constr>,
}

impl SpConstraints {
  pub fn build(data: &ApvrpInstance, tasks: &Tasks, av_routes: &[Vec<TaskId>], pv_routes: &[Vec<TaskId>], model: &mut Model, vars: &Map<TaskId, Var>) -> Result<Self> {
    let mut av_sync = map_with_capacity(vars.len()); // slightly too big but close enough
    for av_tasks in av_routes {
      for (t1, t2) in av_tasks.iter().map(|t| tasks.by_id[t]).tuple_windows() {
        let c = c!(vars[&t1.id()] + t1.tt + data.travel_time[&(t1.end, t2.start)] <= vars[&t2.id()]);
        av_sync.insert((t1.id(), t2.id()), model.add_constr("", c)?);
      }
    }

    let mut start_req = Map::default();
    let mut end_req = Map::default();
    let mut lb = map_with_capacity(vars.len());
    let mut ub = map_with_capacity(vars.len());

    let mut add_bounds = |t: Task, model: &mut Model| -> Result<()> {
      lb.insert(t.id(), model.add_constr("", c!(vars[&t.id()] >= t.t_release))?);
      ub.insert(t.id(), model.add_constr("", c!(vars[&t.id()] <= t.t_deadline - t.tt))?);
      Ok(())
    };

    for pv_route in pv_routes {
      let mut pv_route = pv_route.iter().map(|t| tasks.by_id[t]);
      let mut t1 = pv_route.next().unwrap();
      add_bounds(t1, model)?;

      for t2 in pv_route {
        debug_assert_ne!((t1.ty, t2.ty), (TaskType::Req, TaskType::Req));

        if t2.ty == TaskType::Req {
          let c = c!(vars[&t1.id()] + t1.tt + data.srv_time[&t2.start] <= vars[&t2.id()]);
          start_req.insert(t1.id(), model.add_constr("", c)?);
        } else if t1.ty == TaskType::Req {
          let c = c!(vars[&t1.id()] + t1.tt + data.srv_time[&t1.end] <= vars[&t2.id()]);
          end_req.insert(t2.id(), model.add_constr("", c)?);
        }
        add_bounds(t2, model)?;
        t1 = t2;
      }
    }
    Ok(SpConstraints { av_sync, start_req, end_req, ub, lb })
  }
}

pub struct TimingSubproblem {
  pub vars: Map<TaskId, Var>,
  pub cons: SpConstraints,
  pub model: Model,
}

#[derive(Clone)]
pub enum BendersCut {
  Optimality(OptCut),
  Feasibility(FeasCut),
}

impl BendersCut {
  pub fn into_ineq(self, sets: &Sets, tasks: &Tasks, vars: &MpVars) -> IneqExpr {
    match self {
      BendersCut::Optimality(cut) => cut.into_ineq(sets, tasks, vars),
      BendersCut::Feasibility(cut) => cut.into_ineq(sets, vars),
    }
  }
}

#[derive(Clone)]
pub struct OptCut {
  pub sp_obj: f64,
  pub task_pairs: Vec<(TaskId, TaskId)>,
}

impl OptCut {
  pub fn build(model: &Model, tasks: &Tasks, cons: &SpConstraints) -> Result<Self> {
    let sp_obj = model.get_attr(attr::ObjVal)?;
    let mut task_pairs = Vec::new();

    for (&(t1, t2), c) in &cons.av_sync {
      if tasks.by_id[&t2].ty == TaskType::End
        || model.get_obj_attr(attr::Pi, c)?.abs() > 1e-8
      {
        task_pairs.push((t1, t2));
      }
    }

    Ok(OptCut { sp_obj, task_pairs })
  }

  pub fn into_ineq(self, sets: &Sets, tasks: &Tasks, vars: &MpVars) -> IneqExpr {
    let mut lhs = Expr::default();
    let mut ysum = Expr::default();
    let obj = self.sp_obj;
    let n_pairs = self.task_pairs.len() as isize;

    for (t1, t2) in self.task_pairs {
      let t1 = tasks.by_id[&t1];
      let t2 = tasks.by_id[&t2];

      if t2.ty == TaskType::End {
        for av in sets.avs() {
          if let Some(&theta) = vars.theta.get(&(av, t1.id())) {
            lhs = lhs + theta;
          }
        }
      }

      for av in sets.avs() {
        if let Some(&y) = vars.y.get(&(av, t1.id(), t2.id())) {
          ysum = ysum + y;
        }
      }
    }
    c!(lhs >= self.sp_obj*(1 - n_pairs + ysum))
  }
}

#[derive(Clone)]
pub struct FeasCut {
  pub task_pairs: Vec<(TaskId, TaskId)>,
  pub tasks: FnvHashSet<TaskId>,
}

impl FeasCut {
  pub fn build(model: &Model, cons: &SpConstraints) -> Result<Self> {
    let mut tasks = FnvHashSet::default();
    let mut task_pairs = Vec::new();

    let x_cons = cons.end_req.iter()
      .chain(&cons.start_req)
      .chain(&cons.lb)
      .chain(&cons.ub);

    for (&t, c) in x_cons {
      if model.get_obj_attr(attr::IISConstr, c)? > 0 {
        tasks.insert(t);
      }
    }

    for (&pair, c) in &cons.av_sync {
      if model.get_obj_attr(attr::IISConstr, c)? > 0 {
        task_pairs.push(pair);
      }
    }
    Ok(FeasCut { tasks, task_pairs })
  }

  pub fn into_ineq(self, sets: &Sets, vars: &MpVars) -> IneqExpr {
    let n_pairs = self.task_pairs.len();
    let n_x = self.tasks.len();
    let xsum = self.tasks.iter()
      .map(|t| vars.x[t])
      .grb_sum();
    let ysum = self.task_pairs.into_iter()
      .flat_map(|(t1, t2)|
        sets.avs().filter_map(move |av| vars.y.get(&(av, t1, t2))))
      .grb_sum();

    c!(xsum + ysum <= n_pairs + n_x - 1)
  }
}


impl TimingSubproblem {
  pub fn build(data: &ApvrpInstance, tasks: &Tasks, av_routes: &[Vec<TaskId>], pv_routes: &[Vec<TaskId>]) -> Result<TimingSubproblem> {
    let mut model = Model::new("subproblem")?;
    let vars: Map<_, _> = {
      let mut v = map_with_capacity(av_routes.iter().map(|av_tasks| av_tasks.len()).sum());
      for av_tasks in av_routes {
        for &t in av_tasks {
          v.insert(t, add_ctsvar!(model)?);
        }
      }
      v
    };

    let cons = SpConstraints::build(data, tasks, av_routes, pv_routes, &mut model, &vars)?;

    let mut obj_constant = 0.0;
    let obj = av_routes.iter()
      .map(|av_route| {
        let second_last_task = av_route[av_route.len() - 2];
        let second_last_task = tasks.by_id[&second_last_task];
        obj_constant += (second_last_task.tt + data.travel_time[&(second_last_task.end, data.ddepot)]) as f64;
        vars[&second_last_task.id()]
      })
      .grb_sum() + obj_constant;
    model.set_objective(obj, Minimize)?;

    Ok(TimingSubproblem { vars, cons, model })
  }


  pub fn solve(mut self, tasks: &Tasks) -> Result<Vec<BendersCut>> {
    let mut cuts = Vec::with_capacity(1);

    loop {
      self.model.optimize()?;
      match self.model.status()? {
        Status::Optimal => {
          cuts.push(BendersCut::Optimality(OptCut::build(&self.model, tasks, &self.cons)?));
        }
        Status::Infeasible => {
          self.model.compute_iis()?;
          cuts.push(BendersCut::Feasibility(FeasCut::build(&self.model, &self.cons)?));
        }
        other => {
          panic!("unexpected subproblem status: {:?}", other)
        }
      }
      break; // TODO remove constraints and loop
    }

    Ok(cuts)
  }
}

