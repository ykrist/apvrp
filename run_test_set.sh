#!/bin/bash
set -e
cargo build
export SHELL=$(type -p bash)

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
parallel -j 6 --halt now,fail=1 trace_on_fail --sp dag --two-phase :::: <( python ../../src/apvrp/data_indices.py test )
# parallel -j 6 --halt now,fail=1 target/debug/apvrp --param-name test-dag --cpus 1 --sp dag --two-phase :::: <( python ../../src/apvrp/data_indices.py test )
