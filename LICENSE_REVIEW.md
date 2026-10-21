# License review

Reviewed: 2026-10-21. This is an engineering inventory, not legal advice.

## Release decision

The public package uses separate, explicit licenses:

- `scripts/reproduce_analysis.py`: MIT;
- the exact author-generated documentation, derived CSV/JSON records, and
  figures listed in `LICENSE_SCOPE.md`: CC BY 4.0.

There is no blanket repository license. Files not explicitly listed in the
scope receive no additional grant merely because they are present.

## Original-code review

`scripts/reproduce_analysis.py` is a standalone offline reconstruction utility
written for this release. It uses normal public APIs from the Python standard
library, NumPy, and Matplotlib. No ORB-SLAM2, g2o, DBoW2, Paper1, or other
third-party source was found in it. NumPy and Matplotlib are dependencies, not
vendored code, and retain their own licenses.

## Derived-data review

The listed CSV, JSON, and PNG files were generated from the authors' frozen
Paper2 experiment outputs. Provenance hashes for the 12 copied inputs are in
`PROVENANCE.csv`. The package contains no TUM RGB-D image, depth, ground-truth,
or association file and no third-party runtime source or binary.

## Paper1 public repository

The public Paper1 repository is:

https://github.com/1140829518-crypto/BeliefAware_RGBD_SLAM

This Paper2 release does not inherit Paper1's license and contains no Paper1
source, results, figures, tables, or artifacts.

## Upstream and dataset boundaries

- ORB-SLAM2's official repository identifies the runtime as GPL-3.0. No
  ORB-SLAM2 source or binary is included here.
- g2o and DBoW2 source are not included.
- NumPy and Matplotlib are installed externally and retain their own licenses.
- Original TUM RGB-D files are not redistributed. Users are directed to the
  dataset maintainer and its applicable terms.

If runtime code or raw third-party material is added later, this review and
`LICENSE_SCOPE.md` must be redone before release.
