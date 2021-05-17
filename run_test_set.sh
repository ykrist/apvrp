#!/bin/bash
set -e
cargo build
export RUST_LOG="warn"
export RUST_BACKTRACE="full"
parallel -j 6 --halt now,fail=1 target/debug/apvrp --cpus 1 :::: <( python ../../src/apvrp/data_indices.py test )
