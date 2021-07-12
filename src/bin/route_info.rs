use apvrp::*;
use apvrp::ShorthandTask;
use serde::{Serialize, Deserialize};
use anyhow::{Result, Context};
use itertools::Itertools;
use instances::dataset::Dataset;

struct Args {
  input: String,
}

impl Args {
  pub fn parse() -> Self {
    let args = clap::App::new("route_info")
      .arg(clap::Arg::with_name("input")
        .default_value("routes.json")
      )
      .get_matches();

    let input = args.value_of("input").unwrap().to_owned();

    Args {
      input,
    }
  }
}

#[derive(Clone, Serialize, Deserialize)]
struct RouteInput {
  data_index: usize,
  #[serde(rename="routes")]
  route_specs: Vec<(Av, String)>,
}



fn parse_route_spec(spec: &str) -> Result<Vec<ShorthandTask>> {
  let mut tasks = Vec::new();
  for s in spec.split(", ").map(|s| s.trim()) {
    tasks.push(s.parse()?)
  }
  return Ok(tasks)
}

fn convert_route(tasks: &PvTasks, route: &[ShorthandTask]) -> Result<Vec<PvTask>> {
  route.iter()
    .map(|t|
      tasks.by_shorthand.get(t).copied()
        .ok_or_else(|| anyhow::anyhow!("unable to find task for {:?}", t)))
    .collect()
}

fn print_schedule(data: &Data, route: &[PvTask]) {
  use prettytable::*;
  let times = schedule::av_route(data,route);
  let mut table = Table::new();
  let mut task_row = vec![cell!("Task")];
  let mut release_time = vec![cell!("Rel")];
  let mut st_row = vec![cell!("ST")];
  let mut tt_time = vec![cell!("TT")];

  for ((task1, t1), (task2, _)) in route.iter().zip(&times).tuple_windows() {
    task_row.push(cell!(format!("{:?}", task1)));
    task_row.push(cell!(format!("...")));
    st_row.push(cell!(format!("{:?}", t1)));
    st_row.push(cell!(format!("")));
    tt_time.push(cell!(format!("{:?}", task1.tt)));
    tt_time.push(cell!(data.travel_time[&(task1.end, task2.start)]));
    release_time.push(cell!(format!("{}", task1.t_release)));
    release_time.push(cell!(""));
  }

  if let Some(task) = route.last() {
    let t = times.last().unwrap();
    task_row.push(cell!(format!("{:?}", task)));
    st_row.push(cell!(format!("{:?}", t)));
    tt_time.push(cell!(format!("{:?}", task.tt)));
    release_time.push(cell!(format!("{}", task.t_release)));
  }

  table.add_row(Row::new(task_row));
  table.add_row(Row::new(release_time));
  table.add_row(Row::new(st_row));
  table.add_row(Row::new(tt_time));
  table.printstd();

}

fn print_partial_finish_times(data: &Data, route: &[PvTask]) {
  use prettytable::*;
  let mut table = Table::new();
  let mut task_row = vec![cell!("Task")];
  let mut release_time = vec![cell!("Rel")];
  let mut st_row = vec![cell!("Fin")];


  for (k, t) in route.iter().enumerate() {
    let time = schedule::av_route_finish_time(data, &route[k..]);
    task_row.push(cell!(format!("{:?}", t)));
    st_row.push(cell!(format!("{:?}", time)));
    release_time.push(cell!(format!("{}", t.t_release)));
  }

  table.add_row(Row::new(task_row));
  table.add_row(Row::new(release_time));
  table.add_row(Row::new(st_row));
  table.printstd();

}

fn main() -> Result<()> {
  let args : Args = Args::parse();

  let input = std::fs::read_to_string(&args.input).context("read input")?;
  let input: RouteInput = serde_json::from_str(&input).context("parse input file")?;

  let av_routes : Result<Vec<_>> = input.route_specs.iter()
    .map(|(av, spec)| Ok((*av, parse_route_spec(spec)?)))
    .collect();
  let av_routes = av_routes.context("parse route specs")?;

  let data = dataset(0.5)
    .load_instance(input.data_index)
    .context("load data")
    .map(preprocess::full_pipeline)?;

  let sets = Sets::new(&data);
  let tasks = PvTasks::generate(&data, &sets, &schedule::earliest_departures(&data));

  for (av, route) in av_routes {
    let route = convert_route(&tasks, &route)?;
    println!("Active Vehicle Group {}", av);
    println!("Schedule:");
    print_schedule(&data, &route);
    println!("Partial finish times");
    print_partial_finish_times(&data, &route);
  }


  Ok(())
}