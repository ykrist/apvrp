use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use fnv::FnvHashSet;
use grb::constr::IneqExpr;
use super::mp::MpVars;
use super::cb::{AvPath, PvPath, CbError};
use tracing::{error_span, debug, trace, error};
use crate::TaskType::ODepot;

pub struct SpConstraints {
  av_sync: Map<(Task, Task), Constr>,
  start_req: Map<Task, Constr>,
  end_req: Map<Task, Constr>,
  lb: Map<Task, Constr>,
  ub: Map<Task, Constr>,
}

impl SpConstraints {
  pub fn build(data: &Data, tasks: &Tasks, av_routes: &Map<Avg, Vec<AvPath>>, pv_routes: &Map<Pv, PvPath>, model: &mut Model, vars: &Map<Task, Var>) -> Result<Self> {
    let _span = error_span!("sp_cons").entered();

    // Constraints (3b)
    let mut av_sync = map_with_capacity(vars.len()); // slightly too big but close enough
    for routes in av_routes.values() {
      for task_pairs in routes {
        for &(t1, t2) in task_pairs {
          if t1 != tasks.odepot && t2 != tasks.ddepot {
            debug!(?t1, ?t2, "av_sync");
            let c = c!(vars[&t1] + t1.tt + data.travel_time[&(t1.end, t2.start)] <= vars[&t2]);
            av_sync.insert((t1, t2), model.add_constr("", c)?);
          }
        }
      }
    }


    let mut start_req = Map::default();
    let mut end_req = Map::default();
    let mut lb = map_with_capacity(vars.len());
    let mut ub = map_with_capacity(vars.len());

    let mut add_bounds = |t: Task, model: &mut Model| -> Result<()> {
      // Constraints (3e)
      lb.insert(t, model.add_constr("", c!(vars[&t] >= t.t_release))?);
      ub.insert(t, model.add_constr("", c!(vars[&t] <= t.t_deadline - t.tt))?);
      Ok(())
    };

    for pv_route in pv_routes.values() {
      let mut pv_route = pv_route.iter();
      let mut t1 = *pv_route.next().unwrap();
      add_bounds(t1, model)?;

      for &t2 in pv_route {
        debug_assert_ne!((t1.ty, t2.ty), (TaskType::Request, TaskType::Request));

        if t2.ty == TaskType::Request {
          // Constraints (3c)
          let c = c!(vars[&t1] + t1.tt + data.srv_time[&t2.start] <= vars[&t2]);
          start_req.insert(t1, model.add_constr("", c)?);
        } else if t1.ty == TaskType::Request {
          // Constraints (3d)
          let c = c!(vars[&t1] + t1.tt + data.srv_time[&t1.end] <= vars[&t2]);
          end_req.insert(t2, model.add_constr("", c)?);
        }
        add_bounds(t2, model)?;
        t1 = t2;
      }
    }
    Ok(SpConstraints { av_sync, start_req, end_req, ub, lb })
  }
}

pub struct TimingSubproblem {
  pub vars: Map<Task, Var>,
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

#[derive(Clone, Debug)]
pub struct OptCut {
  /// Subproblem objective when this cut is active.
  pub sp_obj: f64,
  /// Task-to-task pairs which must ALL be active for the cut to be active
  pub task_pairs: Vec<(Task, Task)>,
  /// Tasks which appear in the objective.
  pub obj_tasks: Vec<Task>
}

impl OptCut {
  pub fn build(sp: &TimingSubproblem, tasks: &Tasks) -> Result<Self> {
    let _span = error_span!("opt_cut").entered();

    let sp_obj = sp.model.get_attr(attr::ObjVal)?;
    debug!(?sp_obj);
    let mut task_pairs = Vec::new();
    for (&(t1, t2), c) in &sp.cons.av_sync {
      trace!(?t1, ?t2, dual=?sp.model.get_obj_attr(attr::Pi, c));
      // if t2.ty == TaskType::DDepot
      //   || model.get_obj_attr(attr::Pi, c)?.abs() > 1e-8
      // {
        task_pairs.push((t1, t2));
      // }
    }

    let mut obj_tasks = Vec::with_capacity(sp.vars.len());
    for (t, var) in &sp.vars {
      if sp.model.get_obj_attr(attr::Obj, var)? > 0.001 {
        obj_tasks.push(*t);
      }
    }
    trace!(task_pairs=?&task_pairs);

    Ok(OptCut { sp_obj, task_pairs, obj_tasks })
  }

  pub fn into_ineq(self, sets: &Sets, tasks: &Tasks, vars: &MpVars) -> IneqExpr {
    let _span = error_span!("opt_cut").entered();

    let mut lhs = sets.avs()
      .cartesian_product(self.obj_tasks)
      .filter_map(|(av, t)| vars.theta.get(&(av, t)))
      .grb_sum();

    let mut ysum = Expr::default();
    let obj = self.sp_obj;
    let n_pairs = self.task_pairs.len() as isize;

    for (t1, t2) in self.task_pairs {
      trace!(?t1, ?t2);
      for av in sets.avs() {
        if let Some(&y) = vars.y.get(&(av, t1, t2)) {
          ysum = ysum + y;
        }
      }
    }

    c!(lhs >= self.sp_obj*(1 - n_pairs + ysum))
  }
}

#[derive(Clone)]
pub struct FeasCut {
  pub task_pairs: Vec<(Task, Task)>,
  pub tasks: Vec<Task>,
}

impl FeasCut {
  pub fn build(model: &Model, cons: &SpConstraints) -> Result<Self> {
    let mut tasks = Vec::new();
    let mut task_pairs = Vec::new();

    let x_cons = cons.end_req.iter()
      .chain(&cons.start_req)
      .chain(&cons.lb)
      .chain(&cons.ub);

    for (&t, c) in x_cons {
      if model.get_obj_attr(attr::IISConstr, c)? > 0 {
        tasks.push(t);
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
  pub fn build(env: &Env, data: &Data, tasks: &Tasks, av_routes: &Map<Avg, Vec<AvPath>>, pv_routes: &Map<Pv, PvPath>) -> Result<TimingSubproblem> {
    let _span = error_span!("sp_build").entered();

    let mut model = Model::with_env("subproblem", env)?;
    let vars: Map<_, _> = {
      let mut v = map_with_capacity(pv_routes.values().map(|tasks| tasks.len()).sum());
      for tasks in pv_routes.values() {
        for &t in tasks {
          v.insert(t, add_ctsvar!(model)?);
        }
      }
      v
    };

    let cons = SpConstraints::build(data, tasks, av_routes, pv_routes, &mut model, &vars)?;

    let mut obj_constant = 0.0;
    let obj = av_routes.values()
      .flat_map(|routes| routes.iter())
      .map(|task_pairs| {
        let second_last_task = &task_pairs.last().expect("bug: empty PV route?").0;
        obj_constant += (second_last_task.tt + data.travel_time[&(second_last_task.end, data.ddepot)]) as f64;
        trace!(?second_last_task);
        vars[second_last_task]
      })
      .grb_sum() + obj_constant;
    model.set_objective(obj, Minimize)?;

    Ok(TimingSubproblem { vars, cons, model })
  }


  pub fn solve(mut self, tasks: &Tasks) -> Result<Vec<BendersCut>> {
    let mut cuts = Vec::with_capacity(1);
    // if tracing::level_filters::STATIC_MAX_LEVEL < tracing::Level::INFO {
    self.model.set_param(param::OutputFlag, 0)?;
    // }

    self.model.optimize()?;
    let status = self.model.status();
    trace!(?status);
    match status? {
      Status::Optimal => {
        // TODO check
        cuts.push(BendersCut::Optimality(OptCut::build(&self, tasks)?));
      }
      Status::Infeasible => {
        self.model.compute_iis()?;
        cuts.push(BendersCut::Feasibility(FeasCut::build(&self.model, &self.cons)?));
        // TODO remove constraints and loop
      }

      other => {
        let err = CbError::Status(other);
        error!(status=?other, "{}", err);
        return Err(err.into());
      }
    }

    Ok(cuts)
  }
}

