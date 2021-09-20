#!/bin/bash
#SBATCH --cpus-per-task 1
#SBATCH --nodes 1
#SBATCH --time 0-5:00:00
#SBATCH --mem 8000MB
#SBATCH --array 1-20
#SBATCH --out /data/uqyrist/phd/pkgs/apvrp/logs/debug/%a.out

set -e

source ~/.profile
conda activate or

FAIL_LIST=fail-list.txt
PARAMS=`sed -n "${SLURM_ARRAY_TASK_ID}p" $FAIL_LIST | cut -d / -f 1`
DATA_INDEX=`sed -n "${SLURM_ARRAY_TASK_ID}p" $FAIL_LIST | cut -d / -f 2 | cut -d . -f 1`

export RUST_LOG=trace
echo target/debug/apvrp -l params/run-dbg/${PARAMS}.json -q $DATA_INDEX
