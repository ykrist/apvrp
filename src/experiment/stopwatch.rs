use std::collections::HashMap;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
enum State {
  Stopped,
  Running(String, Instant),
}

pub struct Stopwatch {
  // lap times in milliseconds
  laps: Vec<(String, u128)>,
  state: State,
}

impl Stopwatch {
  pub fn new() -> Self {
    Stopwatch {
      laps: Default::default(),
      state: State::Stopped,
    }
  }

  pub fn start(&mut self, name: String) {
    match &mut self.state {
      State::Running(..) => panic!("Stopwatch is already running"),
      state @ State::Stopped => *state = State::Running(name, Instant::now()),
    }
  }

  pub fn stop(&mut self) {
    let old_state = std::mem::replace(&mut self.state, State::Stopped);
    match old_state {
      State::Running(name, start) => self.laps.push((name, start.elapsed().as_millis())),
      State::Stopped => panic!("Stopwatch is already stopped"),
    }
  }

  pub fn lap(&mut self, new_lap_name: String) {
    let t = Instant::now();
    let new_state = State::Running(new_lap_name, t);
    let old_state = std::mem::replace(&mut self.state, new_state);

    match old_state {
      State::Running(name, start_t) => self
        .laps
        .push((name, t.duration_since(start_t).as_millis())),
      State::Stopped => panic!("Stopwatch is stopped"),
    }
  }

  /// Returns all laps, in milliseconds
  pub fn into_laps(self) -> Vec<(String, u128)> {
    if !matches!(&self.state, State::Stopped) {
      panic!("Stopwatch is still running")
    }
    self.laps
  }

  pub fn print_laps(&self) {
    let mut total = 0.0;
    println!("{:>20}  {:>12}", "Component", "Time (s)");
    for (name, time) in &self.laps {
      let t = (*time as f64) / 1_000.0;
      total += t;
      println!("{:>20}  {:12.2}", name, t);
    }
    println!("{:>20}  {:>12.2}", "[total]", total);
  }
}

pub struct Deadline {
  start_time: Instant,
  total_time: Duration,
}

impl Deadline {
  pub fn start(time_limit: Duration) -> Self {
    Deadline {
      start_time: Instant::now(),
      total_time: time_limit,
    }
  }
  pub fn sec_remaining(&self) -> f64 {
    (self.total_time - self.start_time.elapsed()).as_secs_f64()
  }

  pub fn reached(&self) -> bool {
    self.start_time.elapsed() >= self.total_time
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn simple() {
    let mut s = Stopwatch::new();
    s.start("Foo".to_string());
    s.stop();
    let laps = s.into_laps();
    assert_eq!(laps.len(), 1);
    assert_eq!(&laps[0].0, "Foo");
  }

  #[test]
  #[should_panic]
  fn stop_twice() {
    let mut s = Stopwatch::new();
    s.start("Foo".to_string());
    s.stop();
    s.stop();
  }

  #[test]
  #[should_panic]
  fn stop_without_start() {
    let mut s = Stopwatch::new();
    s.stop();
  }

  #[test]
  #[should_panic]
  fn start_twice() {
    let mut s = Stopwatch::new();
    s.start("lap1".into());
    s.start("lap2".into());
  }

  #[test]
  #[should_panic]
  fn lap_without_start() {
    let mut s = Stopwatch::new();
    s.lap("lap1".into());
  }

  #[test]
  fn lap() {
    let mut s = Stopwatch::new();
    s.start("lap0".into());
    s.lap("lap1".into());
    s.stop();
    let laps = s.into_laps();
    assert_eq!(laps.len(), 2);
    assert_eq!(&laps[0].0, "lap0");
    assert_eq!(&laps[1].0, "lap1");
  }

  #[test]
  #[should_panic]
  fn finish_while_running() {
    let mut s = Stopwatch::new();
    s.start("test".into());
    s.into_laps();
  }
}
