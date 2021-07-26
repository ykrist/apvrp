#!/bin/bash
set -e
cargo build
export RUST_LOG="warn"
export RUST_BACKTRACE="full"
parallel -j 6 --halt now,fail=1 target/debug/apvrp --param-name test-dag --cpus 1 --sp dag --two-phase :::: <( python ../../src/apvrp/data_indices.py test )
