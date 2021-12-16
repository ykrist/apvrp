use crate::Result;
use std::env;
use std::fmt::Debug;
use std::fs::{File, OpenOptions};
use std::path::Path;
use std::{
  io::{BufWriter, Write},
  sync::{Arc, Mutex},
};
use tracing_subscriber::{fmt, prelude::*, registry, EnvFilter};
use tracing_subscriber_serde::{
  format::{Json, MessagePack},
  writer::{FlushGuard, NonBlocking},
  SerdeLayer, WriteEvent,
};

const LOG_BUFFER_LINES: usize = 1 << 22; // 2 ** 22 should be < 2GiB with <= 512 bytes per log entry

pub fn init_logging(logfile: Option<impl AsRef<Path>>, stderr: bool) -> Result<Option<FlushGuard>> {
  let env_filter = EnvFilter::from_default_env();

  let stderr_log = if stderr {
    let layer = fmt::layer().with_target(false).without_time();
    Some(layer)
  } else {
    None
  };

  let (json_log, guard) = if let Some(p) = logfile {
    let (writer, guard) = NonBlocking::new()
      .buf_size(LOG_BUFFER_LINES)
      .finish(BufWriter::new(File::create(p)?));

    let layer = SerdeLayer::new()
      .with_source_location(true)
      .with_format(Json)
      .with_span_ids(true)
      .with_writer(writer.warn_on_error())
      .finish();

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

  registry().with(env_filter).with(stderr_log).try_init().ok();
}

#[derive(Copy, Clone)]
pub struct ConciseDebug<'a, T: ?Sized>(&'a T);

pub trait TracingDebugExt {
  fn concise_debug<'a>(&'a self) -> ConciseDebug<'a, Self> {
    ConciseDebug(self)
  }
}

use itertools::Itertools;
pub use tracing::{
  debug, debug_span, error, error_span, info, info_span, instrument, span::Span, trace, trace_span,
  warn, warn_span,
};
