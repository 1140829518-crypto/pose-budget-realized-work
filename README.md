# Budget-Saturation-Aware Realised-Work Evaluation for RGB-D SLAM

This repository contains the audited code, derived records, and offline analysis
used to study the difference between a requested Levenberg--Marquardt (LM)
iteration budget and the work actually realised by RGB-D SLAM pose
optimisation.

The BSRW framework reports three complementary quantities:

- **Budget Realisation Ratio (BRR):** realised outer iterations divided by
  requested capacity;
- **Cap-hit rate:** the fraction of optimisation rounds that reach the
  requested cap under the recorded termination semantics;
- **Marginal realised-work elasticity (`E_W`):** the adjacent-budget response
  of realised work.

This is an evaluation and instrumentation contribution, not a new optimiser,
online budget controller, or proof of a universal saturation threshold.

## Evidence batches

The batches remain separate throughout the repository:

1. `data/formal/`: historical frozen batch, 60 complete TUM RGB-D runs and
   9,480 PoseOptimization invocations;
2. `data/extension60/`: independent extension batch, 60 successful runs and
   9,480 invocations. Failed acquisition attempts are not included in BRR,
   `E_W`, Pareto, ATE, or RPE calculations;
3. `data/backend_validation/`: offline chained replay with g2o and Ceres 2.2.0,
   50/50 completed run-level records over five independent diagnostic runs,
   7,590 call-result rows and 30,360 round-result rows.

The backend validation uses `CHAINED_OFFLINE_REPLAY_V1`. It is not a bitwise
replica of the historical online implementation and is not a complete online
SLAM validation. Its independent statistical unit is the diagnostic run
(`n=5`), not the call or round.

## Repository layout

```text
runtime/orbslam2_overlay/       Audited instrumentation source overlay
runtime/SOURCE_DIFF.patch       Exact tracked-file patch against the stated base
validation/backend_replay/      Snapshot and g2o/Ceres replay implementation
scripts/run/                    Configuration-driven execution helper
scripts/analysis/               Release and batch consistency checks
configs/                        Example public configuration (no private paths)
data/formal/                    Historical frozen 60-run batch
data/diagnostic/                Separate historical B10 diagnostics
data/extension60/               Independent 60-run extension batch
data/cross_batch/               Batch-separated 120-run summaries
data/backend_validation/        Ceres/g2o formal replay results
generated/                      Historical offline-analysis outputs
docs/                           Runtime, backend, release and security notes
```

Original TUM RGB-D/ICL-NUIM frames, ground truth, ORB vocabulary, model
weights, binaries, raw snapshots, and private machine paths are not
redistributed.

## Quick verification

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
MPLCONFIGDIR=/tmp/matplotlib-paper2 python3 scripts/reproduce_analysis.py
python3 scripts/analysis/verify_release.py
sha256sum -c SHA256SUMS
```

See [INSTALL.md](INSTALL.md), [REPRODUCIBILITY.md](REPRODUCIBILITY.md), and
[DATA_DICTIONARY.md](DATA_DICTIONARY.md). Runtime reconstruction and replay
builds require a separately obtained compatible ORB-SLAM2-derived checkout and
dependencies; see `docs/RUNTIME_INSTRUMENTATION.md` and
`validation/backend_replay/README.md`.

## Version identity

The released overlay and patch were audited against local Paper2 base commit
`7a9200ba9a1ad3006a9fbad02163d31437135371`. This identifier is not claimed to
be an upstream ORB-SLAM2 commit. Apply the patch only to a matching licensed
base or review the overlay manually.

## Licensing

Licensing is file-scoped. Instrumented ORB-SLAM2/g2o-derived runtime and replay
source are distributed under GPL-3.0-or-later while preserving upstream
notices; standalone offline release scripts use MIT; author-generated data and
documentation use CC BY 4.0. No third-party dataset is relicensed. See
[LICENSE_SCOPE.md](LICENSE_SCOPE.md) and `LICENSES/`.

## Citation and scientific boundaries

Publication metadata is intentionally not invented; see `CITATION.cff`.
Do not pool the historical, extension, diagnostic, and backend batches, treat
invocations as independent repetitions, or infer numerical equivalence between
g2o and Ceres from this release.
