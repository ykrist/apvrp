# The Active Passive Vehicle Routing Problem

This repository contains the code used for the paper _"Benders Decomposition with Delayed Disaggregation for the Active
Passive Vehicle Routing Problem"_, which is currently under review.


## Instances and results

Instances and results can be found in the `data/` and `results/` directories, respectively, and are documented there.

## Installation

This repo should be cloned using:

```bash
git clone --recurse-submodules https://github.com/ykrist/apvrp
cd apvrp/
git lfs checkout
```

The repo requires the Rust build tool `cargo` to be installed.  You can do this using [__rustup__](https://rustup.rs/).

You will also need a version of Gurobi installed and set up for use with Rust, see [here](https://github.com/ykrist/rust-grb) for instructions.

If all has gone well, you can run the code with

```bash
cargo run -- --help
```

To run various utility scripts, use the `--bin` flag to `cargo run`, for example:

```bash
cargo run --bin dinfo 0
```
