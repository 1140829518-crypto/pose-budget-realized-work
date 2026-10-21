# Public-release audit

Date: 2026-10-21
Candidate repository name: `RGBD_SLAM_Realized_Work_Analysis`

## Result

**CONTENT_SAFETY_PASS — FILE-SCOPED LICENSES ASSIGNED**

The package passed the content, provenance, offline-reproduction, and security
checks described below. The original analysis script is assigned MIT and the
explicitly listed author-generated derived materials are assigned CC BY 4.0;
the controlling per-file list is `LICENSE_SCOPE.md`.

## Included and verified

- 12 frozen or derived input files, all below 0.5 MB;
- one standalone offline Python analysis script;
- dependency bounds for NumPy and Matplotlib;
- a data dictionary, reproduction instructions, availability statements,
  citation metadata, and license review;
- nine regenerated output files, including five 600 dpi, white-background,
  grid-free figures;
- SHA256 manifest covering the complete candidate package.

The reproduction script was executed successfully in the current environment.
It validated 60 formal runs, 9,480 invocations, four scenes, five budgets, and
zero formal tracking gaps. It did not invoke SLAM.

## Excluded on purpose

- the complete development repository and C++ runtime;
- ORB-SLAM2, g2o, DBoW2, and any other third-party source;
- TUM RGB-D images, depth, and ground-truth source files;
- full run manifests and environment logs containing personal absolute paths;
- binaries, shared libraries, vocabulary files, model weights, YOLO outputs,
  raw SLAM logs, sockets, and temporary files;
- Paper1 source, results, figures, tables, and artifacts;
- manuscript drafts and internal research audits.

## Security scan

Scans cover the release candidate, not the full development tree. Checked for:

- common GitHub/AWS/private-key/token/password patterns;
- user-specific Unix home/mount paths, Windows drive paths, and the local user name;
- hidden Git metadata and symbolic links;
- unexpectedly large files and archives;
- generated Python caches.

No credential, private key, absolute local path, symlink, Git history, archive,
or file above 1 MB is expected in the final package. Exact automated scan output
is recorded in the final verification section below.

## Provenance and scientific boundaries

- Formal data come only from `artifacts/v5_pose_budget_final_20261016/`.
- Statistical robustness and core-table inputs come from the enhanced manuscript's
  frozen reproducibility directory.
- Diagnostic inputs come only from `artifacts/v5_pose_saturation_20261016/` and
  are explicitly labeled separate.
- No number was changed to improve a result, and no new experiment was run.
- Figures in `generated/` are reconstructions, not new observations.

## Remaining publication metadata

1. Update `CITATION.cff` when final journal, year, volume, pages, and DOI exist.
2. Decide separately whether a GPL-compatible runtime patch will ever be
   released; none is included here.
3. Create a permanent archive/DOI only after author approval.

## Upload judgment

- **Technically safe to stage:** yes.
- **Safe to upload without secrets or third-party data:** yes, after final hash
  verification.
- **Ready for the scoped public release:** yes. This means offline analysis
  code and derived data only; it is not a full SLAM runtime release.

## Final automated verification

- offline reconstruction: PASS;
- repeated regeneration produced byte-identical generated files: PASS;
- Python source compilation: PASS;
- common secret/private-key/token pattern matches: 0;
- user-specific absolute path matches: 0;
- files larger than 1 MB: 0;
- symbolic links: 0;
- nested Git directories: 0;
- candidate size before the checksum manifest: 33 files, 2,896,221 bytes;
- source-to-release provenance hashes: all 12 copied data files identical.

## Public-repository staging verification

- target repository: `1140829518-crypto/pose-budget-realized-work`, branch
  `main`;
- original candidate manifest checked before release-metadata edits: PASS;
- frozen `data/`, generated scientific outputs, and
  `scripts/reproduce_analysis.py` remained byte-identical to the audited
  candidate: PASS;
- all 12 `PROVENANCE.csv` source, release, and recorded digests agree: PASS;
- offline in-repository reproduction: PASS (60 runs, 9,480 invocations, four
  scenes, five budgets, zero tracking gaps);
- regenerated scientific CSV/PNG outputs match the audited package: PASS;
- final staged-tree secret/private-path scan: PASS;
- final Git whitespace check: PASS.

When `--output` points to a brand-new empty directory, the first generated
`reproduction_summary.json` omits its own filename from its administrative
`generated_files` list because that file does not exist at enumeration time.
A second run includes it. All eight scientific CSV/PNG outputs are identical
on the first run, and the documented in-repository command is stable. This is
an administrative self-listing limitation, not a change to experiment data or
results; the audited analysis script was therefore not modified.
