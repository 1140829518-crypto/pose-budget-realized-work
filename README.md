# Pose Budget Realized Work

Reproducibility materials for the empirical study of requested versus realized Levenberg–Marquardt (LM) optimization work in a fixed RGB-D SLAM pipeline.

## Scope

The planned release contains author-generated analysis scripts, experiment logs, derived summary tables, plots, data dictionaries, and reproduction instructions for the Paper2 study. It does **not** distribute the TUM RGB-D image/depth sequences, ORB-SLAM2 runtime source, third-party dependencies, or compiled binaries.

**Release status:** licensing and repository scaffolding are present. The complete audited release package has not yet been uploaded to this repository. Do not cite this repository as containing the complete reproducibility package until the data, scripts, provenance manifest, and checksums have been uploaded and verified.

## License boundaries

Licenses are intentionally separated by material type:

- Original analysis scripts authored for this study: MIT, only for files explicitly identified as original code in `LICENSE_SCOPE.md`.
- Author-generated derived experiment tables, logs, and plots: CC BY 4.0, only for files explicitly identified as original derived data in `LICENSE_SCOPE.md`.
- Third-party code, dependencies, and any separately licensed material remain under their respective licenses. No license in this repository relicenses them.
- The original TUM RGB-D dataset is not included. The TUM RGB-D benchmark page states that, unless otherwise noted, its data is licensed under CC BY 4.0; please cite Sturm et al. and consult the official page: https://cvg.cit.tum.de/data/datasets/rgbd-dataset
- ORB-SLAM2 states that its code is released under GPLv3: https://github.com/raulmur/ORB_SLAM2

## Scientific integrity

The release is intended to preserve the frozen scientific results. Reproduction scripts may regenerate tables and figures from the released source data, but should not silently change experiment values, definitions, exclusions, or manuscript claims.

## Citation

Please cite the associated Paper2 manuscript when bibliographic details become final. Until then, use the title and authors provided in the manuscript accompanying the release.

## Reproducibility status

The complete package will be considered released only after the authors upload the audited files, check all SHA-256 hashes, verify the repository tree, and confirm the licensing manifest.