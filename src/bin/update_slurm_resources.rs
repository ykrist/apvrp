use posix_cli_utils::*;
use slurm_tools::{read_json, resource_limit, IndexMap, Job, JobId};
use std::{io::Read, path::PathBuf};

type Limits = IndexMap<usize, u64>;

const TIME_LEVELS: &[f64] = &[300.0, 600.0, 900.0, 1200.0, 3600.0, 4800.0];

#[allow(non_upper_case_globals)]
const GiB: f64 = 1073741824.;

const MEMORY_LEVELS: &[f64] = &[
  4. * GiB,
  8. * GiB,
  16. * GiB,
  32. * GiB,
  64. * GiB,
  128. * GiB,
  256. * GiB,
];

#[derive(ArgEnum, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceKind {
  Memory,
  Time,
}

impl ResourceKind {
  fn levels(&self) -> resource_limit::Levels {
    let margin = resource_limit::Margin::new(0.15, 0., f64::INFINITY).unwrap();
    match self {
      ResourceKind::Memory => resource_limit::Levels::new(
        resource_limit::ResourceKind::Memory,
        MEMORY_LEVELS.to_vec(),
        margin,
      ),
      ResourceKind::Time => resource_limit::Levels::new(
        resource_limit::ResourceKind::Time,
        TIME_LEVELS.to_vec(),
        margin,
      ),
    }
  }

  fn profile_path(&self, name: &str) -> Result<PathBuf> {
    let mut path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    path.push("data/slurm");
    path.push(match self {
      ResourceKind::Memory => "memory",
      ResourceKind::Time => "time",
    });
    path.push(name);
    path.set_extension("json");
    if path.exists() {
      Ok(path)
    } else {
      bail!("Profile {} does not exist ({})", name, path.display())
    }
  }

  fn load_profile(&self, name: &str) -> Result<Limits> {
    read_json(self.profile_path(name)?)
  }

  fn save_profile(&self, name: &str, limits: &Limits) -> Result<()> {
    let p = self.profile_path(name)?;
    let writer = std::fs::File::create(&p).context_write(p)?;
    serde_json::to_writer_pretty(writer, limits)?;
    Ok(())
  }
}

#[derive(Parser)]
struct ClArgs {
  /// Which resource to increase (time, memory)
  #[clap(arg_enum)]
  resource: ResourceKind,
  /// Which profile to modify
  profile: String,
  /// Input Sacct log file (cleaned), defaults to STDIN
  input: Option<PathBuf>,
  /// Dry-run (print changes only, do not save)
  #[clap(short = 'n')]
  dry_run: bool,
}

fn run(i: impl Read, opts: &ClArgs) -> Result<()> {
  let mut limits = opts.resource.load_profile(&opts.profile)?;
  let old_limits = limits.clone();

  let levels = opts.resource.levels();

  for job in serde_json::Deserializer::from_reader(i).into_iter::<Job>() {
    let job = job?;
    let (_, index) = match job.id {
      JobId::Array {
        job_id,
        array_index,
      } => (job_id, array_index as usize),
      other => bail!("expected array jobs only: {:?}", other),
    };

    let new_limit = resource_limit::get_new_limit(&job, &levels).map(|l| l.round() as u64);
    match opts.resource {
      ResourceKind::Memory => match new_limit {
        Ok(new) => {
          let l = limits.get_mut(&index).unwrap();
          *l = new.max(*l);
        }
        Err(e) => {
          return Err(e).context(format!("error assigning limit to job with ID {:?}", job.id))
        }
      },
      ResourceKind::Time => match new_limit {
        Ok(new) => {
          limits.entry(index).and_modify(|l| *l = new.max(*l));
        }
        Err(resource_limit::AssignResourceError::ExceedsLimits { .. }) => {
          limits.remove(&index);
        }
        Err(e) => {
          return Err(e).context(format!("error assigning limit to job with ID {:?}", job.id))
        }
      },
    }
  }

  if opts.dry_run {
    for (i, old) in &old_limits {
      match limits.get(i) {
        None => println!("{}: removed", i),
        Some(new) => {
          if old != new {
            if opts.resource == ResourceKind::Memory {
              println!(
                "{}: change {} GiB to {} GiB",
                i,
                *old as f64 / GiB,
                *new as f64 / GiB
              );
            } else {
              println!("{}: change {} to {}", i, old, new);
            }
          }
        }
      }
    }
  } else {
    opts.resource.save_profile(&opts.profile, &limits)?;
  }

  Ok(())
}

fn main() -> Result<()> {
  let args = ClArgs::parse();
  match Input::default_stdin(args.input.as_ref())? {
    Input::File(f) => run(f, &args),
    Input::Stdin(i) => run(i, &args),
  }
}
