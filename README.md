# RGBD_SLAM_Realized_Work_Analysis

Public reproducibility package for the data and offline analysis accompanying:

> **Realized-Work-Aware Empirical Analysis of Pose Optimization in RGB-D SLAM**

## Scope

This package studies a measurement question in a fixed RGB-D SLAM estimator:
does a requested Levenberg--Marquardt (LM) iteration cap represent the work
actually executed by pose optimization? It separates requested capacity,
realized outer iterations, cap-hit behavior, runtime, and trajectory outcomes.

The package does **not** introduce a new LM solver, adaptive budget method,
early-stopping rule, anytime algorithm, universal optimal iteration count, or
real-time safety guarantee. Lower-budget trajectory errors must not be read as
evidence that less optimization improves accuracy.

## Included evidence

- 60 frozen formal runs;
- four 80-frame TUM RGB-D clips;
- five requested budgets: B1, B3, B4, B5, B10;
- 9,480 PoseOptimization invocation records;
- run-, scene-, and budget-level summaries;
- a separate B10 diagnostic batch for stop labels and iteration decay;
- an offline script that validates the frozen counts, rebuilds summary tables,
  and regenerates data-driven figures.

The diagnostic batch is separate from the 60 formal runs and must not be pooled
with their accuracy statistics.

## Repository structure

```text
data/formal/       Frozen formal-run and invocation CSV files
data/diagnostic/   Separate B10 diagnostic CSV files
scripts/           Offline reconstruction script
generated/         Verified outputs produced by the script
README.md          Scope and quick start
DATA_DICTIONARY.md Column definitions and statistical units
REPRODUCIBILITY.md Detailed reproduction boundary and procedure
LICENSE_REVIEW.md  Licensing observations and file-scope decision
```

No RGB/depth images, TUM ground truth, SLAM binaries, ORB vocabulary, model
weights, ORB-SLAM2 source, g2o source, DBoW2 source, or Paper1 experiment files
are included.

## Quick start: no SLAM execution

Python 3.9 or newer is recommended.

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 scripts/reproduce_analysis.py
sha256sum -c SHA256SUMS
```

The script reads only `data/`, checks the expected 60 formal runs and 9,480
invocations, and writes to `generated/`. It does not access a dataset, invoke a
SLAM executable, or modify the frozen CSV files.

## What can be independently recomputed

Without running SLAM, this package supports:

- scene--budget means and run-level sample standard deviations;
- budget realization ratio and cap-hit summaries;
- B5--B10 marginal realized-work response;
- descriptive requested/realized-work runtime correlations;
- ATE/RPE result plots from frozen evaluated runs;
- diagnostic stop-reason counts and pose-update decay plots;
- integrity checks for all packaged files.

## What is not reproduced here

End-to-end recreation of trajectories, associations, optimizer logs, and ATE/RPE
from RGB-D frames requires the exact instrumented runtime, TUM source data,
associations, ORB vocabulary, and original run configuration. Those components
are not included in this data-analysis release. Consequently, this package is a
verified **analysis reproduction**, not a full SLAM execution reproduction.

See `REPRODUCIBILITY.md`, `DATA_AVAILABILITY.md`, and `CODE_AVAILABILITY.md`.

## Dataset source

The experiments use clips derived from the public TUM RGB-D Dataset and
Benchmark. Obtain any original RGB-D and ground-truth files from the dataset
maintainer:

https://cvg.cit.tum.de/data/datasets/rgbd-dataset

The original dataset is not redistributed here.

## Separation from Paper1

This repository is independent of the Paper1 code release:

https://github.com/1140829518-crypto/BeliefAware_RGBD_SLAM

Paper1 studies persistent MapPoint-level temporal reliability and measurement
adaptation. This package studies requested versus realized optimization work in
a fixed estimator. No Paper1 source, result, figure, or data archive is copied
into this package.

## Citation

Bibliographic publication details are not yet available. See `CITATION.cff`.
The citation record must be updated after publication; do not invent a DOI.

## License scope

Licensing is file-scoped rather than repository-wide:

- the original offline analysis code explicitly listed in `LICENSE_SCOPE.md`
  is available under the MIT License;
- the author-generated derived data, figures, and documentation explicitly
  listed there are available under CC BY 4.0;
- third-party software and the original TUM RGB-D data are not included and
  are not relicensed by this repository.

See `LICENSE_SCOPE.md` and `LICENSES/` for the controlling scope and notices.
