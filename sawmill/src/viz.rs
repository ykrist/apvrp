use crate::*;
use std::fmt::{self, Write as FmtWrite};

pub struct InferenceModelViz<'a, A, C: Constraint> {
  model: &'a InferenceModel<A, C>,
  nodes: Map<Node<A, C>, usize>,
  flat: bool,
  active_clauses: Option<&'a Set<A>>,
  active_constraints: Option<&'a Set<C>>,
  cover: Option<cover::Cover<A, C>>,
  lifted_cover: Option<lift::LiftedCover<A, C>>,
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
  None,
}

impl NodeVizState {
  fn active(&self) -> bool {
    !matches!(self, NodeVizState::None)
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
      fmt_clause: fmt_with_debug,
      fmt_constraint: fmt_with_debug,
      flat: false,
      active_clauses: None,
      active_constraints: None,
      cover: None,
      lifted_cover: None,
    }
  }

  pub fn active_clauses(mut self, clauses: &'a Set<A>) -> Self {
    self.active_clauses = Some(clauses);
    self
  }

  pub fn active_constraints(mut self, constraints: &'a Set<C>) -> Self {
    self.active_constraints = Some(constraints);
    self
  }

  pub fn cover(mut self, cover: cover::Cover<A, C>) -> Self {
    self.cover = Some(cover);
    self
  }

  pub fn lifted_cover(mut self, l: lift::LiftedCover<A, C>) -> Self {
    self.lifted_cover = Some(l);
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

  pub fn render_svg(&self, filepath: &str) -> std::io::Result<()> {
    render_svg(self, filepath)
  }

  fn node_state(&self, n: &Node<A, C>) -> NodeVizState {
    if let Some(cover) = self.cover.as_ref() {
      match n {
        Node::Constr(c) => {
          if cover.values().any(|s| s.contains(c)) {
            return NodeVizState::Cover
          }
        }
        Node::Clause(c) => {
          if cover.contains_key(c) {
            return NodeVizState::Cover
          }
        }
      }
    }

    if let Some(cover) = self.lifted_cover.as_ref() {
      match n {
        Node::Constr(c) => {
          if cover.iter().any(|(_, s)| s.contains(c)) {
            return NodeVizState::Cover
          }
        }
        Node::Clause(c) => {
          if cover.iter().any(|(s, _)| s.contains(c)) {
            return NodeVizState::LiftedCover
          }
        }
      }
    }

    match (n, self.active_constraints, self.active_clauses) {
      (Node::Clause(c), _, Some(active)) if active.contains(c) => NodeVizState::Active,
      (Node::Constr(c), Some(active), _) if active.contains(c) => NodeVizState::Active,
      _ => NodeVizState::None
    }

  }
}

fn render_svg<'a, N: Clone + 'a, E: Clone + 'a, G: dot::Labeller<'a, N, E> + dot::GraphWalk<'a, N, E>>(
  g: &'a G,
  filepath: &str,
) -> std::io::Result<()> {
  let mut buf: Vec<u8> = Vec::with_capacity(8096);
  dot::render(g, &mut buf).unwrap();
  let mut proc = std::process::Command::new("dot")
    .arg("-Tsvg")
    .arg("-Grankdir=LR")
    .arg("-Granksep=2")
    .arg(format!("-o{}", filepath))
    .stdin(std::process::Stdio::piped())
    .stdout(std::process::Stdio::piped())
    .spawn()
    .unwrap();

  proc.stdin.take().unwrap().write(&buf)?;
  let output = proc.wait_with_output()?;
  assert!(output.status.success());
  Ok(())
}

impl<'a, A: Clause, C: Constraint> dot::GraphWalk<'a, Node<A, C>, (Node<A, C>, Node<A, C>)> for InferenceModelViz<'a, A, C> {
  fn nodes(&self) -> dot::Nodes<Node<A, C>> {
    let nodes = self.nodes.keys().filter(|n| self.node_state(n).active()).cloned().collect();
    dot::Nodes::Owned(nodes)
  }

  fn edges(&self) -> dot::Edges<(Node<A, C>, Node<A, C>)> {
    let edges: Vec<_> = if self.flat {
      self.model.implications.iter()
        .flat_map(|(a, cons)|
          cons.iter().map(move |c| (Node::Clause(a.clone()), Node::Constr(c.clone())))
        )
        .filter(|e| self.node_state(&e.0).active() && self.node_state(&e.1).active())
        .collect()
    } else {
      self.model.predecessors.iter()
        .flat_map(|(n, preds)|
          preds.iter().map(move |p| (p.clone(), n.clone()))
        )
        .filter(|e| self.node_state(&e.0).active() && self.node_state(&e.1).active())
        .collect()
    };
    edges.into()
  }

  fn source(&self, edge: &(Node<A, C>, Node<A, C>)) -> Node<A, C> { edge.0.clone() }

  fn target(&self, edge: &(Node<A, C>, Node<A, C>)) -> Node<A, C> { edge.1.clone() }
}


impl<'a, A: Clause, C: Constraint> dot::Labeller<'a, Node<A, C>, (Node<A, C>, Node<A, C>)> for InferenceModelViz<'a, A, C> {
  fn graph_id(&self) -> dot::Id {
    dot::Id::new("debug").unwrap()
  }

  fn node_shape(&self, _n: &Node<A, C>) -> Option<dot::LabelText> {
    Some(dot::LabelText::label("none"))
  }

  fn node_id(&self, n: &Node<A, C>) -> dot::Id {
    dot::Id::new(format!("n{}", self.nodes[n])).unwrap()
  }

  fn node_label(&self, n: &Node<A, C>) -> dot::LabelText {
    let color = match n {
      Node::Constr(_) => "/pastel28/1",
      Node::Clause(_) => "/pastel28/2",
    };

    let (border_color, border_weight) = match self.node_state(n) {
      NodeVizState::Cover => ("red", "3"),
      NodeVizState::LiftedCover => ("blue", "3"),
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

    dot::LabelText::html(html)
  }
}
