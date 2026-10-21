# License scope and release gate

This file defines the intended license boundaries for the Paper2 release. It must be completed with exact paths before the full data package is uploaded.

## MIT — original analysis code only

Planned scope: original Python scripts authored specifically to parse the frozen Paper2 experiment outputs, recompute reported statistics, and generate the paper's figures.

Before listing each script here, verify that it is original author-created code and does not copy code from ORB-SLAM2, another repository, or a third-party package beyond normal library/API use. Dependencies retain their own licenses.

Exact paths: **pending inspection of the audited release package**.

## CC BY 4.0 — author-generated derived data only

Planned scope: original logs and derived CSV tables, summary tables, and figures produced by the authors' experiments, where redistribution is permitted and third-party raw content is not embedded.

The original TUM RGB-D images/depth maps and ground-truth files are not to be copied into this repository. The TUM benchmark page states that, unless otherwise noted, its data is licensed under CC BY 4.0; proper attribution to Sturm et al. must accompany relevant derived-data documentation.

Exact paths: **pending inspection of the audited release package**.

## Explicit exclusions

- ORB-SLAM2 or other third-party runtime source code, patches, binaries, and dependencies.
- Original TUM RGB-D image/depth sequences and other datasets.
- Any file with a separate license, third-party copyright, unclear provenance, or unresolved redistribution restrictions.
- Credentials, private keys, access tokens, private absolute paths, and unrelated research artifacts.

## Release gate

Do not describe the repository as a complete public reproducibility release until:
1. The exact source package is available and its file list matches the prior release audit.
2. Every file has a provenance and license classification.
3. The original analysis script(s) are reviewed for copied/third-party code.
4. The data files are confirmed to be author-generated derived outputs, with raw third-party data excluded.
5. SHA-256 manifests and offline reproduction checks pass after upload.
6. The repository tree is reviewed to ensure that the code and data licenses do not accidentally cover excluded materials.

This document is a scope record, not legal advice or a substitute for confirming authorship and co-author/institutional permissions.