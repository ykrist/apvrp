use structopt::StructOpt;
use apvrp::experiment::Inputs;
use apvrp::*;
use serde::{Serialize};
use std::fmt::Display;
use prettytable::*;

#[derive(StructOpt)]
struct Args {
  /// JSON output
  #[structopt(long)]
  json: bool,
  #[structopt(flatten)]
  input: Inputs
}

#[derive(Serialize, Clone, Debug)]
struct Output {
  index: usize,
  n_req: RawReq,
  n_pv: RawPv,
  n_av: RawAv,
}

impl Output {
  pub fn to_table(&self) -> Table {
    fn display_val(val: impl Display) -> String {
      format!("{}", val)
    }

    let mut table = table!(
      ["Index", display_val(self.index)],
      ["Requests", display_val(self.n_req)],
      ["Passive vehicles", display_val(self.n_pv)],
      ["Active vehicles", display_val(self.n_av)]
    );

    let fmt = format::FormatBuilder::new()
      // .separators(&[format::LinePosition::Bottom, format::LinePosition::Top], format::LineSeparator::new('-', '-', '-', '-'))
      .padding(1, 1)
      .build();

    table.set_format(fmt);
    for cell in table.column_iter_mut(0) {
      cell.align(format::Alignment::RIGHT);
    }
    for cell in table.column_iter_mut(1) {
      cell.align(format::Alignment::RIGHT);
    }
    table
  }
}

fn main() -> anyhow::Result<()> {
  let args: Args = Args::from_args();
  let data = instances::dataset::apvrp::DSET.load_instance(args.input.index)?;
  let output = Output {
    index: args.input.index,
    n_req: data.n_req,
    n_pv: data.n_passive,
    n_av: data.n_active,
  };

  if args.json {
    println!("{}", serde_json::to_string_pretty(&output)?)
  } else {
    output.to_table().printstd();
  }
  Ok(())
}