// use sawmill::*;
//
// #[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
// struct Var(usize);
//
//
// #[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
// enum TimeConstraint {
//   /// Var >= ub
//   Ub(Var, i64),
//   /// Var <= lb
//   Lb(Var, i64),
//   /// Var + delta <= Var
//   Delta(Var, i64, Var)
// }
//
// #[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
// enum TimeConstraintClass {
//   Ub(Var),
//   Lb(Var),
//   Delta(Var, Var),
// }
//
// impl Constraint for TimeConstraint {
//   type Class = TimeConstraintClass;
//   type Rest;
//
//   fn class(&self) -> Self::Class {
//     match self {
//       TimeConstraint::Lb(v,..) => TimeConstraintClass::Lb(*v),
//       TimeConstraint::Ub(v, ..) => TimeConstraintClass::Ub(*v),
//       TimeConstraint::Delta(v1, _, v2) => TimeConstraintClass::Delta(*v1, *v2),
//     }
//   }
// }
//
// struct SimpleDominance;
//
// impl Dominance<TimeConstraint> for SimpleDominance {
//   fn dominates(&self, a: &TimeConstraint, b: &TimeConstraint) -> bool {
//     match (a, b) {
//       // constraint a: var(a) <= UB(a)
//       // constraint b: var(b) <= UB(b)
//       (TimeConstraint::Ub(_, val1), TimeConstraint::Ub(_, val2)) => {
//         val1 <= val2
//       }
//       // constraint a: var(a) >= LB(a)
//       // constraint b: var(b) >= LB(b)
//       (TimeConstraint::Lb(_, val_a), TimeConstraint::Lb(_, val_b)) => {
//         val_a >= val_b
//       }
//       // constraint a: var1(a) + d(a) <= var2(a)
//       // constraint b: var1(b) + d(b) <= var2(b)
//       (TimeConstraint::Delta(_, da, _), TimeConstraint::Delta(_, db, _)) => {
//         da >= db
//       }
//       _ => false
//     }
//   }
//
//   fn dominance_hint(&self, class1: &TimeConstraintClass, class2: &TimeConstraintClass) -> bool {
//     class1 == class2
//   }
// }
//
//
// fn main() {
//   let mut model = InferenceModel::build()
//     .with_dominance(SimpleDominance);
//
//   model.set_active_clauses(todo!());
//   let s = model.get_lifted_expr::<sawmill::cover::Greedy>(todo!());
//
// }

fn main() {

}