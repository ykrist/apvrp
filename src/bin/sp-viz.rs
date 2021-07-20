// use structopt::*;
// use apvrp::{solution::SerialisableSolution, ShorthandTask};
// use anyhow::{Result};
// use dot::*;
//
// #[derive(Debug, Clone)]
// struct Graph(SerialisableSolution);
//
// impl<'a> GraphWalk<'a, ShorthandTask, (ShorthandTask, ShorthandTask)> for Graph {
//   fn nodes(&'a self) -> Nodes<'a, ShorthandTask> {
//     let mut nodes = self.0.pv_routes
//       .iter()
//       .map(|()|)
//   }
//   fn edges(&self) -> Edges<'a, (ShorthandTask, ShorthandTask)> {
//     todo!()
//   }
//   fn source(&self, edge: &(ShorthandTask, ShorthandTask)) -> ShorthandTask { edge.0 }
//   fn target(&self, edge: &(ShorthandTask, ShorthandTask)) -> ShorthandTask { edge.1 }
// }
// impl Labeller<'a, ShorthandTask, (ShorthandTask, ShorthandTask)> for Graph {
//   fn
// }
//
//
//
// fn main() -> Result<()> {
//   let sol_json = r#"{"objective":null,"av_routes":[[0,["ODepot",{"Start":[3,0]},{"Request":0},{"Request":1},{"Transfer":[2,3]},{"Transfer":[0,4]},{"Request":4},{"Transfer":[4,7]},{"Request":7},{"Transfer":[1,8]},{"Request":8},{"End":[1,8]},{"Direct":0},"DDepot"]],[0,["ODepot",{"Start":[1,1]},{"Start":[2,2]},{"Request":2},{"Request":3},{"Transfer":[3,5]},{"Request":5},{"Transfer":[5,6]},{"Request":6},{"Transfer":[6,9]},{"Request":9},{"End":[2,9]},{"End":[3,7]},"DDepot"]]],"pv_routes":[[1,[{"Start":[1,1]},{"Request":[1,1]},{"Transfer":[1,1,8]},{"Request":[1,8]},{"End":[1,8]}]],[0,[{"Direct":0}]],[3,[{"Start":[3,0]},{"Request":[3,0]},{"Transfer":[3,0,4]},{"Request":[3,4]},{"Transfer":[3,4,7]},{"Request":[3,7]},{"End":[3,7]}]],[2,[{"Start":[2,2]},{"Request":[2,2]},{"Transfer":[2,2,3]},{"Request":[2,3]},{"Transfer":[2,3,5]},{"Request":[2,5]},{"Transfer":[2,5,6]},{"Request":[2,6]},{"Transfer":[2,6,9]},{"Request":[2,9]},{"End":[2,9]}]]]}"#;
//
//   let sol : SerialisableSolution = serde_json::from_str(sol_json)?;
//   let sol = sol.to_solution()
//
// }

fn main() {
  
}