# Data dictionary

## Statistical hierarchy

- **iteration:** one realized outer LM update inside a round;
- **round:** one of four robust PoseOptimization rounds;
- **invocation:** one PoseOptimization call containing up to four rounds;
- **run:** one 80-frame execution for one scene and budget;
- **scene:** one evaluated TUM RGB-D clip.

Only runs are experimental repeats. Invocations and rounds within the same run
are correlated observations.

## `data/formal/FINAL_RESULTS.csv` — 60 rows

One row per formal run.

| Field | Definition |
|---|---|
| `sequence` | Short scene identifier used by this study. |
| `role` | Repeat-scene or additional cross-scene role. |
| `budget` | Requested maximum outer LM iterations per round. |
| `run` | Run number within scene and budget. |
| `requested_total` | Sum of authorized outer iterations over all invocation rounds. |
| `realized_total` | Sum of actual outer iterations reported by the solver. |
| `realized_requested_ratio` | `realized_total / requested_total`; budget realization ratio. |
| `round1_realized`--`round4_realized` | Realized totals for each of the four rounds. |
| `cap_hit_rate` | Fraction of round records whose actual count equals the requested cap. |
| `stop_requested_max` | Count of round records classified as reaching the requested cap. |
| `stop_small_improvement` | Count classified as natural small-improvement termination. |
| `stop_max_trials` | Count classified as reaching the internal trial limit. |
| `stop_zero_gain` | Count classified as zero-gain termination. |
| `poseopt_mean_ms` | Mean measured PoseOptimization routine duration per processed frame. |
| `poseopt_p95_ms` | Within-run 95th percentile of PoseOptimization duration. |
| `tracking_mean_ms` | Mean measured Tracking duration. |
| `e2e_mean_ms` | Mean recorded offline processing interval; not a live queueing deadline. |
| `ate_rmse_m` | Absolute trajectory error RMSE in metres. |
| `rpe_translation_rmse_m` | Translational relative pose error RMSE in metres. |
| `rpe_rotation_rmse_deg` | Rotational relative pose error RMSE in degrees. |
| `gt_matches` | Trajectory estimates matched to ground truth. |
| `tracking_gaps` | Recorded missing trajectory-frame count. |
| `wall_seconds` | Wall-clock duration of the run command. |
| `load1_before`, `load1_after` | One-minute system load observations before and after the run. |
| `trajectory_sha256` | SHA256 of the run's estimated trajectory. |
| `run_directory` | Historical relative artifact identifier; not a public filesystem path. |

## `data/formal/INVOCATION_RESULTS.csv` — 9,480 rows

One row per PoseOptimization invocation.

| Field | Definition |
|---|---|
| `sequence`, `budget`, `run` | Parent formal run identifiers. |
| `requested_per_round` | Requested outer-iteration maximum for each round. |
| `realized_iterations` | Sum of realized outer iterations across the invocation's four rounds. |
| `optimization_core_ms` | Instrumented optimization-core duration for that invocation. |

These rows support descriptive runtime association. They are not independent
experimental repeats.

## Formal aggregate files

### `RUNTIME_REPRODUCIBILITY.csv` — 10 rows

Five-budget summaries for each of the two five-repeat scenes. `n` is run count;
`*_mean` and `*_sd` are run-level mean and sample standard deviation for
PoseOptimization time, realized total work, and ATE.

### `SCENE_BUDGET_SUMMARY.csv` — 20 rows

One row per scene--budget cell. Fields cover run count, realized-work mean/SD,
cap-hit rate, PoseOptimization mean/SD/P95, Tracking and offline processing
means, ATE mean/SD, RPE means, GT matches, and tracking gaps. SD is not inferred
for one-run cells.

### `SCENE_GENERALIZATION.csv` — 4 rows

Descriptive per-scene transition summary: B3/B4/B5/B10 cap-hit rates, B5 and
B10 work, B5--B10 change, transition flag, and invocation-level descriptive
runtime correlations.

### `STATISTICAL_ROBUSTNESS.csv` — 210 rows

Long-format statistical audit. `analysis` and `metric` identify the quantity;
`sequence` and `budget_or_transition` identify the group; `n_runs`, `estimate`,
`sd`, and `ci95_*` store run-level evidence where available; `unit` and
`inference` state interpretation. `nan` means not estimable, not zero.

### `CORE_RESULT_TABLE.csv` — 20 rows

Compact manuscript-facing scene--budget table containing run count, budget
realization, cap-hit, realized total, optimizer-core time, and available
run-level uncertainty intervals.

### `EXPERIMENT_ENVIRONMENT.json`

Records CPU affinity, CPU model, platform, tested budgets, and binary/library
hashes. It contains no executable or dataset path.

## Separate diagnostic files

These files describe independently instrumented B10 diagnostic runs. They must
not be joined to formal accuracy means as if they were additional formal runs.

### `STOP_REASONS.csv` — 1,264 rows

One row per diagnostic optimization round: scene, invocation identifier, round,
requested and actual counts, and exact logged stop label.

### `ITERATION_DIAGNOSTICS.csv` — 5,139 rows

One row per realized diagnostic iteration: frame and invocation identifiers,
round, requested/actual counts, stop label, iteration number, robust objective
(`robust_chi2`), pose-update norm, and scene. Pose-update norm combines rotation
and translation parameter components and is a convergence diagnostic, not a
physical distance.

### `ITERATION_CONTRIBUTIONS.csv` — 20 rows

Per-scene, per-iteration medians and P95 values for pose-update norm and robust
objective reduction. `nan` at iteration 1 reflects the absence of a preceding
iteration from which to compute a reduction.

### `PER_ROUND_ACTUAL_ITERATIONS.csv` — 16 rows

Diagnostic budget--scene aggregates for actual totals, means, and cap-hit rates
in each of four rounds, total requested/actual work, realization ratio, and mean
optimizer-core time. This diagnostic sweep includes budgets beyond the five
formal settings and must retain its separate scope.

## Generated files

Files under `generated/` are outputs of `scripts/reproduce_analysis.py` and may
be regenerated from the packaged input CSVs. They are not additional experiments.
