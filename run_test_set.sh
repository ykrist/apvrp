#!/bin/bash
set -e
cargo build
export RUST_LOG="warn"
# export RUST_LOG="[add_cuts]=trace,[enqueue_cut]=trace,info"
export RUST_BACKTRACE="full"
parallel -j 6 --halt now,fail=1 target/debug/apvrp --param-name test-lp --cpus 1 --sp lp :::: <( python ../../src/apvrp/data_indices.py test_easy )
# parallel -j 6 --halt now,fail=1 target/debug/apvrp --param-name test-dag --cpus 1 --sp dag :::: <( python ../../src/apvrp/data_indices.py test_easy )
