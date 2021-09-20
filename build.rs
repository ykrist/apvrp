use anyhow::Result;

fn main() -> Result<()> {
  let output = std::process::Command::new("git").args(["rev-parse", "HEAD"]).output()?;
  assert!(output.status.success());
  let commit = std::str::from_utf8(&output.stdout)?.trim();
  println!("cargo:rustc-env=COMMIT_HASH={}", commit);
  Ok(())
}
