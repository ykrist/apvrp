use tracing_subscriber::{EnvFilter, fmt, registry, prelude::*};
use tracing_appender::{non_blocking, non_blocking::WorkerGuard};
use std::fs::OpenOptions;
use std::path::Path;

fn build_and_set_global_subscriber<P>(logfile: Option<P>, is_test : bool) -> Option<WorkerGuard> where
  P : AsRef<Path>
{
  let stderr_log = fmt::layer();
  let env_filter = EnvFilter::from_default_env();
  let r = registry().with(stderr_log).with(env_filter);

  let flush_guard = match logfile {
    Some(p) => {
      let logfile = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(p).unwrap();
      let (writer, _guard) = non_blocking::NonBlockingBuilder::default()
        .lossy(false)
        .finish(logfile);
      let json = fmt::layer()
        .json()
        .with_span_list(true)
        .with_current_span(false)
        .with_writer(writer);

      let r = r.with(json);
      if is_test { r.try_init().ok(); }
      else { r.init(); }
      Some(_guard)
    },
    None => {
      if is_test { r.try_init().ok(); }
      else { r.init(); }
      None
    }
  };
  return flush_guard
}

pub fn init_logging(logfile: Option<impl AsRef<Path>>) -> Option<WorkerGuard> {
  return build_and_set_global_subscriber(logfile, false);
}

#[allow(dead_code)]
pub(crate) fn init_test_logging(logfile: Option<impl AsRef<Path>>) -> Option<WorkerGuard> {
  return build_and_set_global_subscriber(logfile, true);
}


