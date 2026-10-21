# Data availability

## Included

This release includes author-generated derived tabular records necessary to
reproduce the paper's main offline summaries and figures:

- formal run-level results;
- formal invocation-level work and optimizer-core timing;
- scene--budget summaries and robustness tables;
- separate diagnostic stop labels and per-iteration measurements;
- experiment environment metadata without local filesystem paths.

The exact licensed paths are listed in `LICENSE_SCOPE.md` and are available
under CC BY 4.0.

## Not included

- TUM RGB-D images, depth maps, ground-truth source files, or associations;
- original run directories, trajectories, raw logs, binaries, or
  dynamic-library snapshots;
- YOLO detections, models, or weights;
- Paper1 datasets or experiment outputs.

Original TUM data must be obtained under the dataset maintainer's terms from:

https://cvg.cit.tum.de/data/datasets/rgbd-dataset

## Provenance boundary

The formal CSV files are byte-identical copies of the frozen Paper2 experiment
summaries. Diagnostic CSV files are byte-identical copies of the separate
saturation-diagnostic aggregation. Files containing personal absolute paths or
internal machine commands were intentionally excluded. `PROVENANCE.csv` records
the source-relative path and SHA-256 digest of each copied input.

No permanent archive DOI exists at this release. Add one only after the authors
create and approve an archive record.
