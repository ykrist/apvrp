#!/bin/bash
set -e
cargo build
export SHELL=$(type -p bash)
# export RUST_LOG=debug

function trace_on_fail {
    target/debug/apvrp --param-name test --cpus 1 "$@"

    if [[ $? -ne 0 ]] ; then
        echo
        echo "------------------------- job failed (capturing trace...) ---------------------------------"
        echo
        RUST_LOG=trace RUST_BACKTRACE=full target/debug/apvrp --param-name test -q --cpus 1 "$@"
    fi
}

export -f trace_on_fail
rm -f logs/test/parameters.json
parallel -j 6 --halt now,fail=1 trace_on_fail --load-params params/run/debug-dag_38732.json :::: <( python ../../src/apvrp/data_indices.py test )
