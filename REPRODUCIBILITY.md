# Reproducibility guide

## Reproduction levels

### Level 1: packaged offline analysis — verified

Inputs:

- `data/formal/FINAL_RESULTS.csv`
- `data/formal/INVOCATION_RESULTS.csv`
- `data/diagnostic/STOP_REASONS.csv`
- `data/diagnostic/ITERATION_CONTRIBUTIONS.csv`

Command:

```bash
MPLCONFIGDIR=/tmp/matplotlib-paper2 \
python3 scripts/reproduce_analysis.py
```

Verified with Python 3, NumPy 1.24.4, and Matplotlib 3.7.5. The script produced
the checked-in contents of `generated/`. It validates:

- 60 formal runs;
- four declared scenes and five declared budgets;
- 9,480 invocation rows;
- zero recorded tracking gaps;
- separation of formal and diagnostic batches.

The regenerated PNG files use a pure white background, no decorative grid, and
600 dpi output metadata.

### Level 2: statistical audit — source data included

`STATISTICAL_ROBUSTNESS.csv` contains run-level and scene-level estimates used
by the manuscript's hierarchical audit. Repeated-scene intervals use runs as
the repeat unit; single-run scenes do not receive invented confidence intervals.
Invocation-level correlations are descriptive because adjacent invocations are
not independent experimental repeats.

The public script rebuilds core summaries and descriptive correlations. It does
not rerun the 10,000-resample manuscript bootstrap; the audited bootstrap
output is included as frozen data in `STATISTICAL_ROBUSTNESS.csv`.

### Level 3: instrumented runtime reconstruction — source published, inputs external

The package contains the audited instrumentation overlay and exact tracked
patch. It does not contain the complete base tree, dataset images, ground
truth, ORB vocabulary, model weights, binaries, raw run directories, or raw
backend snapshots. Rebuilding and rerunning therefore requires the compatible
licensed base and source datasets obtained from their maintainers.

### Level 4: g2o/Ceres snapshot replay — implementation and results published

`validation/backend_replay/source/` contains the common snapshot reader,
four-round chained replay and consistency tools. `data/backend_validation/`
contains 50 frozen run-level records and nested call/round results. Raw snapshot
JSON is not redistributed. The formal unit is five diagnostic runs; calls and
rounds are nested observations.

```bash
python3 scripts/analysis/verify_release.py
```

## Generated outputs

- `scene_budget_recomputed.csv`
- `b5_b10_response.csv`
- `runtime_correlations.csv`
- `work_response.png`
- `budget_vs_poseopt_runtime.png`
- `realized_work_vs_runtime.png`
- `accuracy_vs_budget.png`
- `diagnostic_stop_and_decay.png`
- `reproduction_summary.json`

These are derived outputs. Delete or replace `generated/` only in a working copy,
then rerun the script to compare hashes and values.

## Interpretation constraints

1. Do not treat 9,480 invocations as 9,480 independent scene repetitions.
2. Do not pool the diagnostic B10 batch with the 60 formal runs.
3. Do not infer a universal optimal budget from four short clips.
4. Do not interpret recorded offline processing intervals as deadline compliance
   or live real-time capability.
5. Do not interpret non-monotonic ATE/RPE values as a new optimization method.
6. Do not pool the historical 60, extension 60, or backend replay records.
7. Do not describe `CHAINED_OFFLINE_REPLAY_V1` as an online SLAM replication.

## Integrity

Run:

```bash
sha256sum -c SHA256SUMS
```

The manifest covers documentation, code, inputs, and verified generated outputs.
