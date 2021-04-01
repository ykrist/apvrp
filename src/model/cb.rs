use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;
use grb::callback::{Callback, Where, CbResult};
use fnv::FnvHashSet;
use super::sp::{BendersCut, TimingSubproblem};
use crate::graph::DecomposableDigraph;

pub struct CbStats {
  pub n_opt_bc: usize,
  pub n_feas_bc: usize,
}

pub struct Cb<'a> {
  data: &'a ApvrpInstance,
  sets: &'a Sets,
  tasks: &'a Tasks,
  mp: &'a TaskModelMaster,
  stats: CbStats,
}

pub enum Component {
  Path(Vec<Task>),
  Cycle(Vec<Task>),
}

#[inline]
fn discard_weights<T,W>(pairs: Vec<(T,W)>) -> Vec<T> {
  pairs.into_iter().map(|(x, _)| x).collect()
}

type AvPath = Vec<(Task, Task)>;
type PvPath = Vec<Task>;

pub fn av_routes(tasks: &Tasks, task_pairs: &[(Task, Task)]) -> (Vec<AvPath>, Vec<AvPath>) {
  use crate::graph::DecomposableDigraph;

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
  (discard_weights(paths), discard_weights(cycles))
}

pub fn construct_av_routes() -> Vec<Component> {


  // let mut succ = Map::default();
  // for &(av, t1, t2) in task_pairs {
  //   succ.insert(t1, t2)
  // }
  // let mut components = Vec::new();
  //
  // loop {
  //   let mut cycle = false;
  //   let mut component = Vec::with_capacity(3);
  //
  //   let mut current_task = if let Some(t) = succ.get(&tasks.odepot) {
  //     component.push(t);
  //     t
  //   } else {
  //     if Some((&first_task, &current_task)) = succ.iter().first() {
  //       cycle = true;
  //       component.push(first_task);
  //       component.push(current_task);
  //       current_task
  //     } else {
  //       break; // successors map is empty - we are done
  //     }
  //   };
  //
  //
  //   loop {
  //     if let Some(cycle_index) = component[..component.len() - 1].iter().rposition(|t| t == current_task) { // TODO: could optimise this
  //       component = component[cycle_index..].to_vec();
  //       cycle = true;
  //     } else if current_task.ty == TaskType::End {
  //       assert!(!cycle);
  //     } else {
  //       current_task = succ[&current_task];
  //       component.push(current_task);
  //       continue;
  //     }
  //
  //     for t in &component {
  //       succ.remove(t);
  //     }
  //
  //     if cycle {
  //       components.push(Component::Cycle(component))
  //     } else {
  //       components.push(Component::Path(component))
  //     }
  //     break;
  //   }
  // }
  //
  // components
  todo!()
}

pub fn construct_pv_routes() -> Vec<Component> {
  todo!()
}


impl<'a> Callback for Cb<'a> {
  fn callback(&mut self, w: Where) -> CbResult {
    match w {
      Where::MIPSol(ctx) => {
        // let av_routes = construct_av_routes();
        // let pv_routes = construct_pv_routes();
        // let sp = TimingSubproblem::build(self.data, self.tasks, &av_routes, &pv_routes)?;
        // let cuts = sp.solve(self.tasks)?;
        //
        // debug_assert_eq!(cuts.len(), 1); // TODO remove this when multiple cuts are added
        // for cut in cuts {
        //   match &cut {
        //     BendersCut::Optimality(_) => { self.stats.n_opt_bc += 1 }
        //     BendersCut::Feasibility(_) => { self.stats.n_feas_bc += 1 }
        //   }
        //   let cut = cut.into_ineq(self.sets, self.tasks, &self.mp.vars);
        //   ctx.add_lazy(cut)?;
        // }
      }
      _ => {}
    }
    Ok(())
  }
}
