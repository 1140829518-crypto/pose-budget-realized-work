# Runtime instrumentation

## Source identity

- audited Paper2 base: `7a9200ba9a1ad3006a9fbad02163d31437135371`;
- tracked changes: `runtime/SOURCE_DIFF.patch`;
- complete public overlay: `runtime/orbslam2_overlay/`;
- g2o licence notice: `LICENSES/G2O-BSD.txt`;
- Paper2/ORB-SLAM2 derivative notice:
  `LICENSES/ORB_SLAM2-PAPER2-GPL-3.0-or-later-NOTICE.txt`.

The overlay contains the files actually audited for the experiments. It is not
a standalone SLAM distribution.

## Realised-work counter

The actual outer-iteration counter is implemented in the instrumented g2o
optimisation path (`optimization_algorithm_levenberg.*` and
`sparse_optimizer.*`). `src/Optimizer.cc` requests per-round budgets, reads the
native realised counts, records termination information, and aggregates four
rounds into each PoseOptimization invocation. `include/Optimizer.h` exposes the
record structures. `BSRWSnapshot.*` serialises replay inputs without changing
the online optimisation budget or robust-round logic.

## Runtime controls

The public runner uses explicit environment variables rather than fixed local
paths. The key controls are the budget mode, budget/invocation log locations,
optional snapshot directory, source-run identifier, and frame limit. See
`configs/pose_budget_sweep.example.json` and
`scripts/run/run_pose_budget_sweep.py`.

## Boundary

This source supports reconstructing the instrumentation when combined with the
matching licensed base, dependencies, source datasets and configurations. The
repository does not contain enough third-party material to build or execute a
complete SLAM system by itself.
