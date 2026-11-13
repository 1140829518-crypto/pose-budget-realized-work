# Installation

## Offline analysis

Python 3.9 or newer is recommended.

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 scripts/analysis/verify_release.py
```

## Instrumented RGB-D SLAM runtime

The repository publishes an overlay and patch, not a complete ORB-SLAM2
distribution. Obtain the compatible base and its dependencies under their own
licences. The audited base identity is recorded in
`docs/RUNTIME_INSTRUMENTATION.md`.

```bash
git -C /path/to/compatible-paper2-base apply --check \
  /path/to/this-repository/runtime/SOURCE_DIFF.patch
git -C /path/to/compatible-paper2-base apply \
  /path/to/this-repository/runtime/SOURCE_DIFF.patch
```

The two new snapshot files and three test files are untracked additions in the
historical worktree and are therefore supplied in `runtime/orbslam2_overlay/`;
copy them to the corresponding paths after reviewing their licence notices.
Never apply the patch blindly to another ORB-SLAM2 revision.

## g2o/Ceres replay

Requirements include a C++17 compiler, CMake 3.16+, Eigen3, OpenSSL, jsoncpp,
an instrumented g2o build, and Ceres 2.2.0. A typical configuration is:

```bash
cmake -S validation/backend_replay/source \
  -B build/backend-replay \
  -DBSRW_ORBSLAM2_ROOT=/path/to/instrumented-paper2-tree \
  -DBSRW_G2O_LIBRARY=/path/to/libg2o.so \
  -DCeres_DIR=/path/to/ceres-2.2.0/lib/cmake/Ceres
cmake --build build/backend-replay -j2
```

The repository does not install Ceres or modify system packages.
