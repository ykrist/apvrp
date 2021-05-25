use tracing_subscriber::{EnvFilter, fmt, registry, prelude::*};
use tracing_appender::{non_blocking, non_blocking::WorkerGuard};
use std::fs::OpenOptions;
use std::path::Path;
use tracing::{info, warn};
use std::env;

fn build_and_set_global_subscriber(logfile: Option<impl AsRef<Path>>, logfilter_file: Option<impl AsRef<Path>>, is_test: bool) -> Option<WorkerGuard>
{
  let filter_from_file = logfilter_file
    .as_ref()
    .map(|filename| -> Option<String> {
      if let Ok(filter) = std::fs::read_to_string(filename) {
        let lines: Vec<_> = filter.lines()
          .map(|s| s.trim())
          .filter(|s| !s.starts_with('#'))
          .collect();
        Some(lines.join(","))
      } else { None }
    })
    .flatten();

  let message = {
    if filter_from_file.is_some() {
      let s= logfilter_file.as_ref().unwrap();
      Some((tracing::Level::INFO, s.as_ref().to_str().unwrap(), "using log-filter file".to_string()))
    } else if let Some(filename) = &logfilter_file {
      Some((tracing::Level::WARN, filename.as_ref().to_str().unwrap(), format!("unable to open log-filter file, falling back to RUST_LOG env var.")))
    } else {
      None
    }
  };

  let using_filter_from_file = filter_from_file.is_some();
  let logfilter_file_supplied = logfilter_file.is_some();

  let stderr_log = fmt::layer().with_target(false).without_time();
  let env_filter = filter_from_file.map(EnvFilter::new).unwrap_or_else(EnvFilter::from_default_env);
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
      if is_test { r.try_init().ok(); } else { r.init(); }
      Some(_guard)
    }
    None => {
      if is_test { r.try_init().ok(); } else { r.init(); }
      None
    }
  };

  if let Some((level, filename, message)) = message {
    // for reason, I can't provide a non-const value to the `event!` macro
    if level == tracing::Level::WARN { warn!(filename, "{}", message);  }
    else {  info!(filename, "{}", message); }
  }
  // if using_filter_from_file {
  //   tracing::info!(filename=?logfilter_file.unwrap(), "using log-filter file");
  // } else {
  //   if let Some(filename) = logfilter_file {
  //     warn!(filename=?filename, "Unable to read from log-filter file. Falling back to RUST_LOG env var.");
  //   }
  // }


  return flush_guard;
}

pub fn init_logging(logfile: Option<impl AsRef<Path>>, logfilter_file: Option<impl AsRef<Path>>) -> Option<WorkerGuard> {
  return build_and_set_global_subscriber(logfile, logfilter_file, false);
}

#[allow(dead_code)]
pub(crate) fn init_test_logging(logfile: Option<impl AsRef<Path>>) -> Option<WorkerGuard> {
  return build_and_set_global_subscriber(logfile, None::<&str>, true);
}


