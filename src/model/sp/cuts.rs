use super::*;

// #[derive(Copy, Clone, Debug)]
// pub struct MrsCut;

// impl GenOptimalityCut for MrsCut {
//     fn cut(cb: &mut cb::Cb, subproblem: &mut GraphModel, active_mp_vars: &Set<MpVar>, theta_est: &ThetaVals) -> Result<()> {
//         todo!()
//     }
// }

// #[derive(Copy, Clone, Debug)]
// pub struct MrsTreeCut;

// impl GenOptimalityCut for MrsTreeCut {
//     fn cut(cb: &mut cb::Cb, subproblem: &mut GraphModel, active_mp_vars: &Set<MpVar>, theta_est: &ThetaVals) -> Result<()> {
//         todo!()
//     }
// }

#[derive(Copy, Clone, Debug)]
pub struct MrsPathCut;

impl GenOptimalityCut for MrsPathCut {
  fn cut(
    cb: &mut cb::Cb,
    sp: &mut GraphModel,
    active_mp_vars: &Set<MpVar>,
    theta_est: &ThetaVals,
  ) {

    for path in sp.model.critical_paths().unwrap() {
      
      let k = path.len();
      let last_task = cb.lu.tasks.by_index[&sp.var_to_task[&path[k - 1]]];
      let path_objective = sp.model.get_solution(&path[k - 1]).unwrap() as Time
        + cb.lu.data.travel_time_to_ddepot(&last_task);
      let a = sp.obj_tasks[&last_task.index()];

      if path_objective <= theta_est[&(a, last_task)] {
        trace!(a = a.0, t=?last_task, true_obj=path_objective, "skip slack optimality cut");
        continue;
      }

      let first_task = sp.var_to_task[&path[0]];
      
      let mut constraint_set: Set<_> = sp.cp_edge_constraints(&last_task, &path).collect();
      debug_assert_eq!(k, constraint_set.len());
      constraint_set.insert(SpConstr::Lb(first_task, sp.lbs[&first_task]));
      
      let cover = cb.inference_model.cover(active_mp_vars, &constraint_set);
      // TODO: lift cover here?

      let mut lhs = Expr::from(-path_objective * (k as Time - 1));

      for (var, _) in cover {
        let grb_var = cb.mp_vars.get_grb_var(cb.lu, &var);
        lhs += path_objective * grb_var;
      }

      cb.enqueue_cut(c!(lhs <= cb.mp_vars.theta[&(a, last_task)]), CutType::LpOpt);
    }
  }
}

impl<'a> GraphModel<'a> {
  fn cp_edge_constraints<'b>(&'b self, last_task: &Task, critical_path: &'b [daggylp::Var]) -> impl Iterator<Item=SpConstr> + 'b {
      let edge_to_ddepot = SpConstr::Delta(
        last_task.index(),
        IdxTask::DDepot,
        self.lu.data.travel_time_to_ddepot(last_task),
      );
      iter_pairs(critical_path)
        .map(move |(v1, v2)| self.edge_constraints[&(*v1, *v2)])
        .chain(std::iter::once(edge_to_ddepot))
  }
}

#[derive(Copy, Clone, Debug)]
pub struct CriticalPathCut;

impl GenOptimalityCut for CriticalPathCut {
  fn cut(cb: &mut cb::Cb, sp: &mut GraphModel, active_mp_vars: &Set<MpVar>, theta_est: &ThetaVals) {
    #[inline]
    fn subpath_objective(lbs: &[Time], edge_constrs: &[SpConstr]) -> Time {
      debug_assert_eq!(lbs.len(), edge_constrs.len());
      let mut time = lbs[0] + edge_constrs[0].d().unwrap();
      for (&lb, &c) in lbs[1..].iter().zip(&edge_constrs[1..]) {
        time = std::cmp::max(time, lb) + c.d().unwrap();
      }
      time
    }

    for path in sp.model.critical_paths().unwrap() {
      let k = path.len();
      let last_task = cb.lu.tasks.by_index[&sp.var_to_task[&path[k - 1]]];
      let path_objective = sp.model.get_solution(&path[k - 1]).unwrap() as Time
        + cb.lu.data.travel_time_to_ddepot(&last_task);
      let a = sp.obj_tasks[&last_task.index()];

      if path_objective <= theta_est[&(a, last_task)] {
        trace!(a = a.0, t=?last_task, true_obj=path_objective, "skip slack optimality cut");
        continue;
      }

      let first_task = sp.var_to_task[&path[0]];

      let implied_lbs: Vec<_> = path
        .iter()
        .map(|t| cb.lu.tasks.by_index[&sp.var_to_task[t]].t_release)
        .collect();

      let edge_constraints: Vec<_> = sp.cp_edge_constraints(&last_task, &path).collect();
      debug_assert_eq!(k, edge_constraints.len());
      trace!(path_len=k, ?last_task, a=a.0, ?edge_constraints, "optimality cut needed");

      let mut constraint_set: Set<_> = edge_constraints.iter().copied().collect();
      constraint_set.insert(SpConstr::Lb(first_task, sp.lbs[&first_task]));

      let cover = cb.inference_model.cover(active_mp_vars, &constraint_set);

      let mut lhs = Expr::default();
      for (var, constrs) in cover {
        let grb_var = cb.mp_vars.get_grb_var(cb.lu, &var);
        if constrs.contains(edge_constraints.last().unwrap()) {
          lhs += grb_var * path_objective;
          continue;
        }
        let start_idx = match constrs
          .iter()
          .filter_map(|c| edge_constraints.iter().rposition(|x| c == x))
          .max()
        {
          None => {
            debug_assert_eq!(constrs.len(), 1);
            0
          }
          Some(k) => k + 1,
        };
        trace!(start_idx, ?var, ?constrs);
        let new_obj = subpath_objective(&implied_lbs[start_idx..], &edge_constraints[start_idx..]);
        let coeff = new_obj - path_objective;
        trace!(?var, ?constrs, new_obj, coeff);
        debug_assert!(coeff <= 0);
        lhs += (1 - grb_var) * coeff;
      }

      cb.enqueue_cut(c!(lhs <= cb.mp_vars.theta[&(a, last_task)]), CutType::LpOpt);
    }
  }
}
