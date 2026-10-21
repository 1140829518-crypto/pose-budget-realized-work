# Upload gate resolution

Reviewed: 2026-10-21

The blockers recorded in the repository scaffold were resolved for this
release candidate as follows:

1. The exact audited candidate package was transferred and verified against
   its original `SHA256SUMS` before release metadata were changed.
2. `scripts/reproduce_analysis.py` was reviewed as original standalone analysis
   code; its exact MIT scope is recorded in `LICENSE_SCOPE.md`.
3. Every derived data, diagnostic, figure, and documentation path covered by
   CC BY 4.0 is listed explicitly in `LICENSE_SCOPE.md`.
4. Original TUM RGB-D files, third-party runtime code and binaries, Paper1
   materials, credentials, and private paths are absent.
5. Offline reproduction, deterministic regenerated-output comparison, source
   provenance hashes, and final-tree security checks passed before upload.

This resolution does not expand the release into an end-to-end SLAM runtime
reproduction. Adding runtime code or third-party data requires a new audit.
