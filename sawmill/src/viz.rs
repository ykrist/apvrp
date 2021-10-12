use crate::*;
use std::fmt::{self, Write as FmtWrite};
use gvdot::{Graph, GraphComponent};

pub struct InferenceModelViz<'a, A, C: Constraint> {
  model: &'a InferenceModel<A, C>,
  nodes: Map<Node<A, C>, usize>,
  flat: bool,
  active_nodes: Map<Node<A, C>, NodeVizState>,
  fmt_constraint: fn(&mut String, &C),
  fmt_clause: fn(&mut String, &A),
}

fn fmt_with_debug<T: fmt::Debug>(s: &mut String, c: &T) {
  write!(s, "{:?}", c).unwrap();
}

#[derive(Debug, Copy, Clone)]
enum NodeVizState {
  Cover,
  LiftedCover,
  Active,
}

impl Default for NodeVizState {
  fn default() -> Self { NodeVizState::Active }
}

impl NodeVizState {
  fn set_cover(&mut self) { *self = NodeVizState::Cover }

  fn set_lifted(&mut self) {
    if !matches!(self, NodeVizState::Cover) {
      *self = NodeVizState::LiftedCover
    }
  }
}

impl<'a, A: Clause, C: Constraint> InferenceModelViz<'a, A, C> {
  pub fn new(model: &'a InferenceModel<A, C>) -> Self {
    let mut nodes = Map::default();
    model.predecessors.keys().chain(
      model.predecessors.values().flat_map(|preds| preds.iter())
    ).for_each(|n|
      if !nodes.contains_key(n) {
        nodes.insert(n.clone(), nodes.len());
      }
    );

    InferenceModelViz {
      model,
      nodes,
      active_nodes: Map::default(),
      fmt_clause: fmt_with_debug,
      fmt_constraint: fmt_with_debug,
      flat: false,
    }
  }

  pub fn active_clauses(mut self, clauses: &'a Set<A>) -> Self {
    for a in clauses {
      self.active_nodes.entry(Node::Clause(a.clone())).or_default();
      for c in self.model.implications.get(a).into_iter().flatten() {
        self.active_nodes.entry(Node::Constr(c.clone())).or_default();
      }
    }
    self
  }

  pub fn active_constraints(mut self, constraints: &'a Set<C>) -> Self {
    for c in constraints {
      self.active_nodes.entry(Node::Constr(c.clone())).or_default();
    }
    self
  }

  pub fn cover(mut self, cover: &cover::Cover<A, C>) -> Self {
    for (a, cons) in cover {
      self.active_nodes.insert(Node::Clause(a.clone()), NodeVizState::Cover);
      for c in cons {
        self.active_nodes.insert(Node::Constr(c.clone()), NodeVizState::Cover);
      }
    }
    self
  }

  pub fn lifted_cover(mut self, l: &lift::LiftedCover<A, C>) -> Self {
    for (clauses, cons) in l {
      for a in clauses {
        self.active_nodes.entry(Node::Clause(a.clone())).or_default().set_lifted();
      }
      for c in self.model.implied_constraints(clauses) {
        let s = self.active_nodes.entry(Node::Constr(c)).or_default();
      }
      for c in cons {
        self.active_nodes.entry(Node::Constr(c.clone())).or_default().set_lifted();
      }
    }
    self
  }

  pub fn flatten(mut self, on: bool) -> Self {
    self.flat = on;
    self
  }

  pub fn fmt_clause(mut self, f: fn(&mut String, &A)) -> Self {
    self.fmt_clause;
    self
  }

  pub fn fmt_constraint(mut self, f: fn(&mut String, &C)) -> Self {
    self.fmt_constraint;
    self
  }

  fn draw_edge(&self,g: &mut gvdot::Graph<Vec<u8>>, e: &(Node<A, C>, Node<A, C>)) {
    if self.node_is_visible(&e.0) && self.node_is_visible(&e.1) {
      g.add_edge(self.nodes[&e.0], self.nodes[&e.1]).unwrap();
    }
  }

  pub fn render_svg(&self, filepath: &str) -> std::io::Result<()> {
    use gvdot::{GraphComponent, attr, val, SetAttribute};

    let mut g = gvdot::Graph::new().directed().strict(true).in_memory();

    for (n, i) in self.nodes.iter() {
      if self.node_is_visible(n) {

        let color = match n {
          Node::Constr(_) => "/pastel28/1",
          Node::Clause(_) => "/pastel28/2",
        };

        let (border_color, border_weight) = match self.active_nodes.get(n) {
          Some(&NodeVizState::Cover) => ("red", "3"),
          Some(&NodeVizState::LiftedCover) => ("blue", "3"),
          _ =>  ("black", "0"),
        };

        let mut html = format!(
          concat!(
          r#"<FONT FACE="fantasque sans mono">"#,
          r#"<TABLE CELLSPACING="0" CELLBORDER="1" BGCOLOR="{}" COLOR="{}" BORDER="{}">"#,
          r#"<TR><TD>"#,
          ),
          color, border_color, border_weight
        );

        match n {
          Node::Clause(a) => (self.fmt_clause)(&mut html, a),
          Node::Constr(c) => (self.fmt_constraint)(&mut html, c),
        }

        write!(&mut html, concat!(
        r#"</TD></TR>"#,
        r#"</TABLE>"#,
        r#"</FONT>"#
        )).unwrap();


        g.add_node(*i).unwrap()
          .attr(attr::Shape, val::Shape::None).unwrap()
          .attr(attr::Label, attr::html(&html)).unwrap();
      }
    }


    let edges = if self.flat {
      for e in self.model.implications.iter().flat_map(|(a, cons)|
          cons.iter().map(move |c| (Node::Clause(a.clone()), Node::Constr(c.clone())))
      ) {
        self.draw_edge(&mut g, &e);
      }
    } else {
      for e in self.model.predecessors.iter().flat_map(|(n, preds)|
          preds.iter().map(move |p| (p.clone(), n.clone()))
        ) {
        self.draw_edge(&mut g, &e);
      }
    };

    let dot = g.into_string();
    let status = gvdot::render_svg(&dot, gvdot::Layout::Dot, filepath)?;
    assert!(status.success());
    Ok(())
  }

  fn node_is_visible(&self, n: &Node<A, C>) -> bool {
    self.active_nodes.is_empty() || self.active_nodes.contains_key(n)
  }
}
