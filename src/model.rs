use crate::*;
use crate::tasks::TaskId;
use grb::prelude::*;

pub struct Constraints<K> {
  map: Map<K, Constr>
}

pub struct TaskModelMaster {
  pub x_vars: Map<(Pv, Loc, Loc), Var>,
  pub y_vars: Map<(Task, Task), Var>,
  pub u_vars: Map<Req, Var>,
  pub theta_vars: Map<Task, Var>,
  pub model: Model,
}

impl TaskModelMaster {
  pub fn new(data: &ApvrpInstance, sets: &Sets, tasks: &Tasks) -> Result<()> {
    let mut model = Model::new("Task Model MP")?;

    let x_vars: grb::Result<Map<_, _>> = tasks.all.iter()
      .map(|t|
        add_binvar!(model)
          .map(|var| ((t.p, t.start, t.end), var))
      )
      .collect();
    let x_vars = x_vars?;
    //
    // let y_vars: grb::Result<Map<_, _>> = tasks.all.iter()
    //   .map(|t| tasks.by_start[t.start].)

    Ok(())
  }
}



