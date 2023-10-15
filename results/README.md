# Computational Results

The following table maps the variant names in the files to the names used in the paper.

| Name in Filename   | Name in Paper    |
|--------------------|------------------|
| `bl`               | BL               |
| `bl_2p`            | P                |
| `bl_2p_minvi`      | PCF              |
| `2p_minvi_pvcg`    | PCFR             |
| `2p_minvi_pvcg_1t` | PCFR (1 Thread)  |
| `2p_minvi_pvcg_2t` | PCFR (2 Threads) |
| `2p_minvi_pvcg_3t` | PCFR (3 Threads) |
| `2p_minvi_pvcg_5t` | PCFR (5 Threads) |
| `2p_minvi_pvcg_6t` | PCFR (6 Threads) |
| `dropout_pvcg`     | PCFT             |
| `dropout_fork`     | PCRT             |
| `dropout_cycle`    | PFRT             |
| `2p_vi_pvcg`       | PCFRT            |

## Individual Raw Results & Solutions

In the TODO directory are the raw files produced by running a single instance over single set of parameters.  Each directory is of the form `NAME_[0-4]` where `NAME` is the name of the algorithmic variant.  Each variant is run 5 times over different solver seeds to account for variability in the branch-and-bound trajectory, hence the `[0-4]` suffix.  Every directory contains a `parameters.json` which is a machine-readable set of parameter values (including the Gurobi seed).  You can pass these parameters to the main binary using the `-l` flag:

```bash
cargo run --bin apvrp -- -l parameters.json INDEX
```

Inside each directory is a pile of output files for each instance, which 4 files created for each instance run:

| File           | Description                                                                          |
|----------------|--------------------------------------------------------------------------------------|
| `N.out`        | Standard output of the command, including the Gurobi log.                            |
| `N-soln.json`  | The best solution found by the algorithm.  See below for format.                     |
| `N-info.json`  | Various metrics and information used to measure performance, used for paper results. |
| `N-index.json` | A small metadata file used by scripts.                                               |

The `N` in the filenames represents the [instance index](../data/README.md).

### Solution format

The top-level JSON object has the following form (there are some extra fields which are not relevant and not listed):

```json
{
  "objective": 1001,
  "av_routes": [...],
  "pv_routes": [...],
  ...
}
```

The `objective` is objective value of the solution.  Each AV route in the list represented by a pair `[a, tasks]` where
`a` is the ID of the __active vehicle group__ (_not_ the active vehicle) and tasks is a list of tasks done in order.  The JSON description of tasks
is given in the table below.

| Task Type | JSON representation      | Description                                                 |
|-----------|--------------------------|-------------------------------------------------------------|
| AV Depot  | `"ODepot"`, `"DDepot"`   | These are "dummy" tasks used to construct the AV network.   |
| Start     | `{"Start": [p, r]}`      | `p` is the passive vehicle index, `r` is the request index. |
| End       | `{"End": [p, r]}`        | `p` is the passive vehicle index, `r` is the request index. |
| Transfer  | `{"Transfer": [r1, r2]}` | `r1`, `r2` are the request indices .                        |
| Request   | `{"Request": r}`         | `r` is the request index.                                   |
| Direct    | `{"Direct": p}`          | `p` is the passive vehicle index.                           |

`pv_routes` has a similar structure `[p, tasks]` where `p` is the passive vehicle index.  Note that `"ODepot"` and `"DDepot"` won't appear in the task list for `pv_routes`.

In the code the JSON form of a task is directly deserialised into `IdxTask`.

## Aggregated Results (`aggregated.tar.xz`)

These results are machine-readable version of the results presented in the paper.  They aggregate each of 5 solver seed runs into one table, so the directories `bl_[0-4]/` yield one aggregated file, `bl.csv`.  The data for these results mostly comes from the `N-info.json` files.

There is also an additional file, `network_sizes.csv` which contains the size of the task network and number of PV routes for each instance.  These stats can be gathered using

```bash
cargo run --bin network_size INDEX
```