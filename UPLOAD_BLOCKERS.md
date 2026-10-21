# Upload blockers — Paper2 reproducibility package

## Current verified repository state

The public repository exists and currently contains only the release-scope README and license documentation. The full audited candidate directory is not available in the execution environment used for this upload; no substitute data or reconstructed scientific result files have been uploaded.

## Must complete before declaring the package released

1. Transfer the exact audited directory `artifacts/v5_pose_budget_public_release_20261021/` from the research workstation into the release workflow. Do not regenerate or edit scientific results during transfer.
2. Compare the file list and SHA-256 manifest against the existing release audit.
3. Review `scripts/reproduce_analysis.py` and any helper scripts for provenance and third-party code. List only verified original scripts under the MIT scope.
4. List exact derived-data paths under CC BY 4.0. Confirm no TUM RGB-D image/depth/ground-truth files or other restricted third-party content are included.
5. Verify the license of every other artifact. Exclude files with unclear ownership or licensing.
6. Run the offline reproduction script on the uploaded package and verify deterministic outputs/hashes.
7. Re-scan the final staged tree for credentials, private keys, tokens, private absolute paths, and unrelated files.
8. Update `LICENSE_SCOPE.md` with exact file paths; only then treat the license scope as final.
9. Confirm co-author/institutional permission to release the author-generated materials.

## Explicitly not done

- No scientific CSV, log, plot, or manuscript value has been altered or synthesized.
- No SLAM experiment was run.
- No ORB-SLAM2 runtime source, binary, or original TUM sequence was uploaded.
- No claim is made that the full reproducibility package is already online.

## Current decision

**Scaffolding uploaded; complete release package blocked pending transfer and file-by-file audit.**