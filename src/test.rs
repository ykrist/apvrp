use std::path::Path;

pub fn test_data_dir() -> &'static Path {
  Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/"))
}

pub fn test_output_dir() -> &'static Path {
  Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/outputs/"))
}
