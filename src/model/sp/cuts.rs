use super::*;

#[derive(Copy, Clone, Debug)]
pub struct MrsCut;

impl GenOptimalityCut for MrsCut {
  fn cut(cb: &mut cb::Cb, sp: &mut GraphModel, active_mp_vars: &Set<MpVar>, theta_est: &ThetaVals) {
    let obj = sp.model.compute_obj().unwrap() as Time
      + sp
        .obj_tasks
        .keys()
        .map(|t| sp.lu.data.travel_time_to_ddepot(&sp.lu.tasks.by_index[t]))
        .sum::<Time>();

    debug_assert!(obj > theta_est.values().sum());

    let mut constraints = Set::default();
    let mut leaf_tasks = Vec::new();

    for tree in sp.model.compute_mrs().unwrap() {
      leaf_tasks.extend(tree.obj_vars().map(|v| {
        let t = &sp.var_to_task[&v];
        (sp.obj_tasks[t], sp.lu.tasks.by_index[t])
      }));

      constraints.extend(
        tree
          .edge_constraints()
          .map(|pair| sp.edge_constraints[&pair]),
      );

      constraints.insert(SpConstr::Lb(
        sp.var_to_task[&tree.root()],
        tree.root_lb() as Time,
      ));
    }

    for (_, t) in &leaf_tasks {
      let d = sp.lu.data.travel_time_to_ddepot(&t);
      constraints.insert(SpConstr::Delta(t.index(), IdxTask::DDepot, d));
    }

    let cover = cb.inference_model.cover(&active_mp_vars, &constraints);
    standard_mrs_cut(cb, obj, cover.iter().map(|(v, _)| *v), leaf_tasks);
  }
}

fn standard_mrs_cut(
  cb: &mut cb::Cb,
  true_obj: Time,
  xy_vars: impl IntoIterator<Item = MpVar>,
  theta_vars: impl IntoIterator<Item = (Avg, Task)>,
) {
  let mut k = 0;

  let mut lhs = Expr::default();

  for var in xy_vars {
    k += 1;
    let grb_var = cb.mp_vars.get_grb_var(cb.lu, &var);
    lhs += true_obj * grb_var;
  }

  lhs += true_obj * (1 - k);

  let rhs = theta_vars
    .into_iter()
    .map(|key| cb.mp_vars.theta[&key])
    .grb_sum();

  cb.enqueue_cut(c!(lhs <= rhs), CutType::LpOpt);
}

#[derive(Copy, Clone, Debug)]
pub struct MrsTreeCut;

impl GenOptimalityCut for MrsTreeCut {
  fn cut(cb: &mut cb::Cb, sp: &mut GraphModel, active_mp_vars: &Set<MpVar>, theta_est: &ThetaVals) {
    let mut total = 0;
    for (tree_idx, tree) in sp.model.compute_mrs().unwrap().into_iter().enumerate() {
      let leaf_tasks: Vec<_> = tree
        .obj_vars()
        .map(|v| {
          let t = &sp.var_to_task[&v];
          (sp.obj_tasks[t], sp.lu.tasks.by_index[t])
        })
        .collect();

      let last_tasks_to_ddepot: Time = leaf_tasks
        .iter()
        .map(|(_, t)| sp.lu.data.travel_time_to_ddepot(t))
        .sum();
      let obj_tree_est = leaf_tasks
        .iter()
        .map(|key| theta_est[key])
        .inspect(|val| assert!(val > &0))
        .sum::<Time>();
      let obj_tree_actual = tree.obj() as Time + last_tasks_to_ddepot;
      let foo = leaf_tasks
        .iter()
        .map(|(_, t)| sp.model.get_solution(&sp.task_to_var[&t.index()]).unwrap() as Time)
        .sum::<Time>();
      assert_eq!(foo + last_tasks_to_ddepot, obj_tree_actual);

      total += obj_tree_actual;

      let _s = trace_span!(
        "build_cut",
        tree_idx,
        obj_tree_actual,
        obj_tree_est,
        last_tasks_to_ddepot,
        ?leaf_tasks
      )
      .entered();
      if obj_tree_actual <= obj_tree_est as Time {
        // FIXME: somehow this is wrong
        trace!("skip slack optimality cut");
        continue;
      }

      trace!("opt cut required");

      let mut constraints: Set<_> = tree
        .edge_constraints()
        .map(|pair| sp.edge_constraints[&pair])
        .collect();

      for (_, t) in &leaf_tasks {
        let d = sp.lu.data.travel_time_to_ddepot(&t);
        constraints.insert(SpConstr::Delta(t.index(), IdxTask::DDepot, d));
      }

      constraints.insert(SpConstr::Lb(
        sp.var_to_task[&tree.root()],
        tree.root_lb() as Time,
      ));

      let cover = cb.inference_model.cover(&active_mp_vars, &constraints);
      standard_mrs_cut(
        cb,
        obj_tree_actual,
        cover.iter().map(|(v, _)| *v),
        leaf_tasks,
      );
    }

    trace!(sp_obj = total);
  }
}

#[derive(Copy, Clone, Debug)]
pub struct MrsPathCut;

impl GenOptimalityCut for MrsPathCut {
  fn cut(cb: &mut cb::Cb, sp: &mut GraphModel, active_mp_vars: &Set<MpVar>, theta_est: &ThetaVals) {
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

      standard_mrs_cut(
        cb,
        path_objective,
        cover.iter().map(|(v, _)| *v),
        [(a, last_task)],
      );
    }
  }
}

impl<'a> GraphModel<'a> {
  fn cp_edge_constraints<'b>(
    &'b self,
    last_task: &Task,
    critical_path: &'b [daggylp::Var],
  ) -> impl Iterator<Item = SpConstr> + 'b {
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
      trace!(
        path_len = k,
        ?last_task,
        a = a.0,
        ?edge_constraints,
        "optimality cut needed"
      );

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

#[cfg(all(test, feature = "expensive-tests"))]
mod tests {
  use labrat::Experiment;

  use super::*;
  use crate::experiment::*;
  use crate::solution::load_test_solution;
  use crate::test::*;

  fn cut_correctness(ty: OptimalityCutKind) -> Result<()> {
    let mut params = test_params("opt_cuts")?;
    params.opt_cut = ty;

    let test = |index: usize| -> Result<()> {
      let exp = ApvrpExp::test(index, params.clone());
      let (lookups, sol) = test_run(exp)?;
      let correct_sol = load_test_solution(index, &lookups)?;
      assert_eq!(correct_sol.objective, sol.objective);
      Ok(())
    };

    let inputs: Vec<_> = dataset_group("test_easy")?;
    batch_test(inputs, test)
  }

  #[cfg(feature = "expensive-tests")]
  #[test]
  fn critical_path() -> Result<()> {
    cut_correctness(OptimalityCutKind::CriticalPath)
  }

  #[cfg(feature = "expensive-tests")]
  #[test]
  fn mrs_path() -> Result<()> {
    cut_correctness(OptimalityCutKind::MrsPath)
  }

  #[cfg(feature = "expensive-tests")]
  #[test]
  fn mrs_tree() -> Result<()> {
    cut_correctness(OptimalityCutKind::MrsTree)
  }

  #[cfg(feature = "expensive-tests")]
  #[test]
  fn mrs_classic() -> Result<()> {
    cut_correctness(OptimalityCutKind::Mrs)
  }
}
