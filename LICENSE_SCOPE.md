# Licence scope

Reviewed: 2026-11-13

This repository uses file-scoped licensing. Nothing here relicenses datasets,
dependencies, or source obtained elsewhere.

## GPL-3.0-or-later and preserved upstream notices

The Paper2/ORB-SLAM2-derived instrumentation overlay, exact source patch,
runtime tests, and g2o/Ceres replay implementation are distributed under
GPL-3.0-or-later subject to preserved third-party notices:

- `runtime/SOURCE_DIFF.patch`;
- `runtime/orbslam2_overlay/**`;
- `validation/backend_replay/source/**`;
- `validation/backend_replay/README.md`.

Modified g2o-origin files also retain the BSD notice in
`LICENSES/G2O-BSD.txt`. See
`LICENSES/ORB_SLAM2-PAPER2-GPL-3.0-or-later-NOTICE.txt`. The repository does
not claim ownership of upstream code.

## MIT — standalone offline release tools

The following original, standalone scripts are licensed under
`LICENSES/MIT-CODE.txt`:

- `scripts/reproduce_analysis.py`;
- `scripts/analysis/verify_release.py`;
- `scripts/run/run_pose_budget_sweep.py`.

Imported packages retain their own terms.

## CC BY 4.0 — author-generated data and documentation

Author-generated CSV/JSON/PNG evidence under `data/` and `generated/`, public
configuration examples under `configs/`, and documentation/metadata files at
the repository root and under `docs/` are licensed under
`LICENSES/CC-BY-4.0-DATA.md`, except where another notice is embedded.

## Explicit exclusions

No licence grant is made here for original TUM RGB-D or ICL-NUIM data, ORB
vocabulary files, model weights, Ceres, complete ORB-SLAM2/g2o/DBoW2 source
trees, binaries, or any unlisted third-party material. Those items are not
redistributed.

This scope record is an engineering and provenance statement, not legal advice.
