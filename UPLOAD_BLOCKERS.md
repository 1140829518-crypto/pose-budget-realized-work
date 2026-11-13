# Upload gate resolution

Reviewed: 2026-11-13

Resolved for branch `release/current-experimental-code`:

- the dirty research worktree was not staged, cleaned or overwritten;
- release work occurred in an independent clone and branch;
- runtime and replay source identities are documented and hashed;
- public copies contain no dataset frames, model weights, binaries, secrets or
  private filesystem paths;
- licence scopes and upstream notices are explicit;
- historical, extension and backend batches remain separately labelled;
- offline reconstruction, Python checks, runtime counter tests, path-portable
  replay build, model-equivalence check and release-count validation passed.

Raw snapshots and source datasets remain intentionally blocked from upload.
Their absence limits end-to-end execution but does not invalidate the packaged
offline analysis or derived formal result tables.
