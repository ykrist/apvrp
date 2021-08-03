use tracing_subscriber::{EnvFilter, fmt, registry, prelude::*};
use tracing_appender::{non_blocking, non_blocking::WorkerGuard};
use std::fs::{OpenOptions, File};
use std::path::Path;
use std::env;
use std::fmt::Debug;
use crate::Result;

const LOG_BUFFER_LINES: usize = 4_194_304; // 2 ** 22; should be < 2GiB with <= 512 bytes per log entry

pub fn init_logging(logfile: Option<impl AsRef<Path>>, stderr: bool) -> Result<Option<WorkerGuard>> {
  let env_filter = EnvFilter::from_default_env();

  let stderr_log = if stderr {
    let layer = fmt::layer()
      .with_target(false)
      .without_time();
    Some(layer)
  } else {
    None
  };

  let (json_log, guard) = if let Some(p) = logfile {
    let (writer, guard) = non_blocking::NonBlockingBuilder::default()
      .lossy(false)
      .buffered_lines_limit(LOG_BUFFER_LINES)
      .finish(File::create(p)?);

    let layer = fmt::layer()
      .json()
      .with_span_list(true)
      .with_current_span(false)
      .with_writer(writer);
    (Some(layer), Some(guard))
  } else {
    (None, None)
  };

  registry()
    .with(env_filter)
    .with(stderr_log)
    .with(json_log)
    .init();

  Ok(guard)
}

#[allow(dead_code)]
pub(crate) fn init_test_logging() {
  let env_filter = EnvFilter::from_default_env();

  let stderr_log = fmt::layer()
      .with_target(false)
      .without_time()
      .with_test_writer();

  registry()
    .with(env_filter)
    .with(stderr_log)
    .try_init()
    .ok();

}

#[derive(Copy, Clone)]
pub struct ConciseDebug<'a, T: ?Sized>(&'a T);

pub trait TracingDebugExt {
  fn concise_debug<'a>(&'a self) -> ConciseDebug<'a, Self> {
    ConciseDebug(self)
  }
}

pub use tracing::{
  instrument,
  trace,
  info,
  debug,
  warn,
  error,
  trace_span,
  info_span,
  debug_span,
  warn_span,
  error_span,
  span::Span,
};
use itertools::Itertools;