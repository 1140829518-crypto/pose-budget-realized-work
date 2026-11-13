# Licence review

Reviewed: 2026-11-13. This is an engineering inventory, not legal advice.

The release uses file-scoped licences recorded in `LICENSE_SCOPE.md`:

- GPL-3.0-or-later for the Paper2/ORB-SLAM2-derived overlay, patch, tests and
  backend replay implementation;
- preserved BSD notice for modified bundled-g2o files;
- MIT for standalone offline/public release scripts;
- CC BY 4.0 for author-generated derived records and documentation.

No blanket relicensing of ORB-SLAM2, g2o, Ceres, DBoW2, NumPy, Matplotlib,
TUM RGB-D, ICL-NUIM or another dependency is asserted. Complete third-party
source trees, datasets and binaries are excluded. The overlay must be combined
only with a lawfully obtained compatible base.
