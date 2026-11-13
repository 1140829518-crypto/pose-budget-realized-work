# Code availability

The repository includes:

- standalone historical offline analysis;
- a portable release-integrity validator and execution helper;
- the audited Paper2/ORB-SLAM2 instrumentation overlay;
- the exact tracked-file patch against the stated local base identity;
- common snapshot, g2o and Ceres 2.2.0 replay source.

The complete third-party ORB-SLAM2/g2o/DBoW2 base, Ceres installation,
binaries, ORB vocabulary, model weights and raw snapshots are not included.
Consequently the packaged analysis is directly reproducible, while end-to-end
runtime and backend replay require separately obtained licensed dependencies
and input data. See `INSTALL.md`, `LICENSE_SCOPE.md`, and `docs/RELEASE_SCOPE.md`.
