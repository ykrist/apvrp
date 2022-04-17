#![feature(exit_status_error)]

use anyhow::Result;
use std::path::Path;
use std::process::Command;

const DATA_ROOT: &'static str = concat!(env!("CARGO_MANIFEST_DIR"), "/data/uncompressed");

fn extract_data() -> Result<()> {
  let root = Path::new(DATA_ROOT);
  let mut subdir = root.join("placeholder");
  let mut tarfile = root.parent().unwrap().join("placeholder");

  for name in ["apvrp_meisel", "apvrp_tilk_b_part", "apvrp_tilk"] {
    subdir.set_file_name(name);
    if !subdir.exists() {
      tarfile.set_file_name(name);
      tarfile.set_extension("tar.xz");

      Command::new("tar")
        .arg("-C")
        .arg(root)
        .arg("-Jxf")
        .arg(&tarfile)
        .status()?
        .exit_ok()?;
    }
  }
  Ok(())
}

fn current_commit() -> Result<String> {
  let output = std::process::Command::new("git")
    .args(["rev-parse", "HEAD"])
    .output()?;
  output.status.exit_ok()?;
  let commit = std::str::from_utf8(&output.stdout)?.trim().to_string();
  Ok(commit)
}

fn main() -> Result<()> {
  extract_data()?;
  println!("cargo:rustc-env=COMMIT_HASH={}", current_commit()?);
  Ok(())
}
