# Public release scope

Included: audited instrumentation overlay, exact tracked diff, replay source,
portable execution helpers, frozen derived CSVs, reports, documentation and
hashes.

Excluded: TUM RGB-D and ICL-NUIM source frames/ground truth, ORB vocabulary,
YOLO weights, compiled libraries, executables, raw snapshots, trajectories,
private paths, temporary worktrees, Windows backups, failure logs with local
environment details, and unrelated Paper1/V4/V5 development material.

The extension batch contains only 60 successful effective runs in analysis
tables. Earlier failed attempts remain archived internally and are neither
deleted nor counted as successes. Backend failure records are retained in
`FORMAL_FAILURES.csv`; the formal batch contains no failed run-level record.
