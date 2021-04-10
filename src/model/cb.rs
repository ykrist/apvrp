use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use grb::callback::{Callback, Where, CbResult};
use fnv::FnvHashSet;
use super::sp::{BendersCut, TimingSubproblem};
use crate::graph::DecomposableDigraph;
use tracing::{info, info_span};
use std::fmt;

#[derive(Default, Clone)]
pub struct CbStats {
  pub n_opt_bc: usize,
  pub n_feas_bc: usize,
}

pub struct Cb<'a> {
  data: &'a Data,
  sets: &'a Sets,
  tasks: &'a Tasks,
  mp_vars: super::mp::MpVars,
  stats: CbStats,
}

#[derive(Debug, Clone)]
pub enum CbError {
  Other(String)
}

impl fmt::Display for CbError {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    f.write_str("Callback error: ")?;
    match self {
      CbError::Other(s) => f.write_str(s)?,
    }
    Ok(())
  }
}

impl std::error::Error for CbError {}

impl<'a> Cb<'a> {
  pub fn new(data: &'a Data, sets: &'a Sets, tasks: &'a Tasks, mp_vars: super::mp::MpVars) -> Self {
    let stats = CbStats::default();
    Cb {
      data,
      sets,
      tasks,
      mp_vars,
      stats
    }
  }
}

pub enum Component {
  Path(Vec<Task>),
  Cycle(Vec<Task>),
}

#[inline]
fn discard_edge_weights<T,W>(pairs: Vec<(T, W)>) -> Vec<T> {
  pairs.into_iter().map(|(x, _)| x).collect()
}

pub(super) type AvPath = Vec<(Task, Task)>;
pub(super) type PvPath = Vec<Task>;

pub fn construct_av_routes(tasks: &Tasks, task_pairs: &[(Task, Task)]) -> (Vec<AvPath>, Vec<AvPath>) {
  use crate::graph::DecomposableDigraph;
  println!("{:?}", task_pairs);
  struct AvGraph {
    pub odepot: Task,
    pub succ: Map<Task, Task>,
  }

  impl DecomposableDigraph<Task, (Task, Task), u8> for AvGraph {
    fn is_sink(&self, node: &Task) -> bool { node.ty == TaskType::End }

    fn next_start(&self) -> Option<Task> {
      if self.succ.contains_key(&self.odepot) { Some(self.odepot) }
      else { self.succ.keys().next().copied() }
    }

    fn next_outgoing_arc(&self, task: &Task) -> ((Task, Task), Task, u8) {
      dbg!(&self.succ, task);
      let next_task = self.succ[task];
      ((*task, next_task), next_task, 1)
    }

    fn subtract_arc(&mut self, arc: &(Task, Task), _: u8) {
      let removed = self.succ.remove(&arc.0);
      debug_assert!(removed.is_some())
    }
  }

  let graph = AvGraph{
    odepot: tasks.odepot,
    succ: task_pairs.iter().cloned().collect()
  };

  let (paths, cycles) = graph.decompose_paths_cycles();
  (discard_edge_weights(paths), discard_edge_weights(cycles))
}

pub fn construct_pv_routes(data: &Data,  tasks_used: &[Task]) -> (PvPath, Vec<PvPath>) {
  debug_assert!(!tasks_used.is_empty());
  let first_task = *tasks_used.first().unwrap();

  if tasks_used.len() == 1 {
    debug_assert_eq!(first_task.ty, TaskType::Direct);
    return (vec![first_task], vec![])
  }

  struct PvGraph {
    pub pv_origin: Loc,
    pub pv_dest: Loc,
    pub task_by_start: Map<Loc, Task>,
  }

  impl DecomposableDigraph<Loc, Task, u8> for PvGraph {
    fn is_sink(&self, node: &Loc) -> bool { node == &self.pv_dest }

    fn next_start(&self) -> Option<Loc> {
      if self.task_by_start.contains_key(&self.pv_origin) { Some(self.pv_origin) }
      else { self.task_by_start.keys().next().copied() }
    }

    fn next_outgoing_arc(&self, node: &Loc) -> (Task, Loc, u8) {
      let task = self.task_by_start[node];
      (task, task.end, 1)
    }

    fn subtract_arc(&mut self, arc: &Task, _: u8) {
      let removed = self.task_by_start.remove(&arc.start);
      debug_assert!(removed.is_some())
    }
  }

  let pv_origin = first_task.p.unwrap();
  let graph = PvGraph {
    pv_origin,
    pv_dest: pv_origin + data.n_passive,
    task_by_start: tasks_used.iter().map(|&t| (t.start, t)).collect()
  };


  let (pv_paths, pv_cycles) = graph.decompose_paths_cycles();

  debug_assert_eq!(pv_paths.len(), 1);
  let pv_path = discard_edge_weights(pv_paths).pop().unwrap();
  (pv_path, discard_edge_weights(pv_cycles))
}

impl<'a> Callback for Cb<'a> {
  fn callback(&mut self, w: Where) -> CbResult {
    let _span = info_span!("callback").entered();


    match w {
      Where::MIPSol(ctx) => {
        let mut heuristic_cut_add = false;

        let mut task_pairs = map_with_capacity(self.data.av_groups.len());

        for ((av, t1, t2), val) in self.mp_vars.y.keys()
          .zip(ctx.get_solution(self.mp_vars.y.values())?)
        {
          if val > 0.9 {
            info!(av, t1 = ?self.tasks.by_id[t1], t2 = ?self.tasks.by_id[t2], ?val);
            task_pairs.entry(*av).or_insert_with(Vec::new).push((self.tasks.by_id[t1], self.tasks.by_id[t2]));
          }
        }


        let mut av_routes = map_with_capacity(task_pairs.len());
        for (&av, av_task_pairs) in &task_pairs {
          let (av_paths, av_cycles) = construct_av_routes(&self.tasks, av_task_pairs);
          if av_cycles.len() > 0 {
            info!(num_cycles=av_cycles.len(), "AV cycles found")
          }

          av_routes.insert(av, av_paths);

          // TODO heuristic AV-path feasibility cuts
        }

        let mut x_tasks = map_with_capacity(self.data.n_passive);

        for (t, val) in self.mp_vars.x.keys()
          .zip(ctx.get_solution(self.mp_vars.x.values())?) {
          if val > 0.9 {
            let t = self.tasks.by_id[t];
            x_tasks.entry(t.p.unwrap()).or_insert_with(Vec::new).push(t);
          }
        }

        let mut pv_routes = map_with_capacity(x_tasks.len());
        for (&pv, pv_tasks) in &x_tasks {
          let (pv_path, pv_cycles) = construct_pv_routes(&self.data, pv_tasks);
          pv_routes.insert(pv, pv_path);

          // TODO heuristic PV feasibility cuts
          // TODO heuristic cycle cuts
        }
        // let (av_paths, av_cycles) = construct_av_routes(&self.tasks, );
        // let pv_routes = construct_pv_routes();

        if !heuristic_cut_add {
          info!("no heuristic cuts found, building MIP subproblem");
          let sp = TimingSubproblem::build(self.data, self.tasks, &av_routes, &pv_routes)?;
          info!("solving MIP subproblem");
          let cuts = sp.solve(self.tasks)?;

          debug_assert_eq!(cuts.len(), 1); // TODO remove this when multiple cuts are added
          for cut in cuts {
            match &cut {
              BendersCut::Optimality(_) => { self.stats.n_opt_bc += 1 }
              BendersCut::Feasibility(_) => { self.stats.n_feas_bc += 1 }
            }

            let cut = cut.into_ineq(self.sets, self.tasks, &self.mp_vars);
            ctx.add_lazy(cut)?;
          }

        }
      }
      _ => {}
    }
    Ok(())
  }
}
