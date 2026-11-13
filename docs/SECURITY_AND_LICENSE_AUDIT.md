# Security and licence audit

The release candidate was scanned for private keys, common token prefixes,
password assignments and local filesystem paths. Public CSV path columns were
normalised to repository-relative or `internal://` identifiers while their
numeric results and cryptographic hashes were left unchanged.

No dataset frames, model weights, credentials, SSH material or compiled
binaries are included. Runtime-derived files preserve GPL-3.0-or-later and
upstream notices; modified g2o files retain the BSD notice; standalone offline
scripts and author-generated data use the file-scoped terms in
`LICENSE_SCOPE.md`.

This is a provenance record, not legal advice.
