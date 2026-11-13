# Current experimental-code public-release audit

Date: 2026-11-13

## Decision

**PASS FOR SCOPED PUBLIC BRANCH.** The release contains audited experimental
instrumentation, replay source, derived evidence and analysis without source
datasets or credentials. It is not a self-contained redistribution of
ORB-SLAM2 or the benchmark datasets.

## Scientific checks

- historical batch: 60 runs, 9,480 invocations;
- independent extension: 60 successful runs, 9,480 invocations;
- cross-batch index: 120 rows with explicit batch identity;
- backend validation: 50 records, five independent runs, 7,590 calls, 30,360
  rounds, 3,795 paired calls, zero formal failures;
- frozen backend mean BRR values match the reported rounded values;
- calls and rounds remain nested, not independent repeats.

## Test checks

- historical offline reconstruction: PASS;
- release hierarchy/path validator: PASS;
- all published Python sources compile: PASS;
- generic runner dry run: PASS (15 planned records for 5 budgets × 3 repeats);
- g2o counter synthetic test: PASS;
- PoseOptimization non-interference test: PASS;
- build-capability banner test: PASS;
- public replay CMake configure/build against audited dependencies: PASS;
- SE(3)/Huber model-equivalence executable: PASS.

No new SLAM experiment was run for this release.

## Exclusions

Raw RGB-D data, associations, ground truth, raw snapshots, ORB vocabulary,
model weights, binaries, full run directories, internal backups, unrelated
Paper1/V4/V5 material and local machine paths are excluded. Extension failure
logs remain in the internal archive and are not presented as successful runs.
