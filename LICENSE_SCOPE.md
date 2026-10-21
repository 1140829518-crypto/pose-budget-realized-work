# License scope

Reviewed: 2026-10-21

This repository deliberately uses file-scoped licensing. No license here
relicenses ORB-SLAM2, g2o, DBoW2, NumPy, Matplotlib, the TUM RGB-D benchmark,
or any other third-party material.

## MIT License — original analysis code

The following file was written specifically for this Paper2 release and was
reviewed as standalone offline analysis code. It does not contain copied
ORB-SLAM2, g2o, DBoW2, Paper1, or other third-party source code:

- `scripts/reproduce_analysis.py`

That file is licensed under the MIT License in `LICENSES/MIT-CODE.txt`.
Imported Python packages remain governed by their own licenses.

## CC BY 4.0 — author-generated derived material

The following files are author-generated documentation, provenance records,
derived experiment data, or figures and are licensed under Creative Commons
Attribution 4.0 International as described in
`LICENSES/CC-BY-4.0-DATA.md`.

Documentation and release metadata:

- `README.md`
- `DATA_DICTIONARY.md`
- `REPRODUCIBILITY.md`
- `DATA_AVAILABILITY.md`
- `CODE_AVAILABILITY.md`
- `LICENSE_REVIEW.md`
- `RELEASE_AUDIT.md`
- `UPLOAD_BLOCKERS.md`
- `PROVENANCE.csv`
- `CITATION.cff`
- `requirements.txt`
- `SOURCE_CANDIDATE_SHA256SUMS`
- `SHA256SUMS`

Frozen and derived formal experiment material:

- `data/formal/FINAL_RESULTS.csv`
- `data/formal/INVOCATION_RESULTS.csv`
- `data/formal/RUNTIME_REPRODUCIBILITY.csv`
- `data/formal/SCENE_BUDGET_SUMMARY.csv`
- `data/formal/SCENE_GENERALIZATION.csv`
- `data/formal/EXPERIMENT_ENVIRONMENT.json`
- `data/formal/STATISTICAL_ROBUSTNESS.csv`
- `data/formal/CORE_RESULT_TABLE.csv`

Separate diagnostic material:

- `data/diagnostic/STOP_REASONS.csv`
- `data/diagnostic/ITERATION_DIAGNOSTICS.csv`
- `data/diagnostic/ITERATION_CONTRIBUTIONS.csv`
- `data/diagnostic/PER_ROUND_ACTUAL_ITERATIONS.csv`

Recomputed outputs:

- `generated/accuracy_vs_budget.png`
- `generated/b5_b10_response.csv`
- `generated/budget_vs_poseopt_runtime.png`
- `generated/diagnostic_stop_and_decay.png`
- `generated/realized_work_vs_runtime.png`
- `generated/reproduction_summary.json`
- `generated/runtime_correlations.csv`
- `generated/scene_budget_recomputed.csv`
- `generated/work_response.png`

## License and administrative files

The license texts in `LICENSES/` reproduce or reference their respective
standard terms and are not relicensed by this scope declaration.
`.gitignore` and `.gitattributes` are administrative repository metadata.

## Explicit exclusions

The following are not included and receive no license grant from this
repository:

- ORB-SLAM2, g2o, DBoW2, or other third-party source code and binaries;
- original TUM RGB-D images, depth maps, ground truth, and associations;
- Paper1 code, results, figures, tables, or artifacts;
- instrumented runtime patches, trajectories, vocabulary files, model weights,
  raw SLAM logs, and internal machine configuration;
- credentials, private keys, access tokens, or private filesystem paths.

The packaged CSV and figure files are derived research outputs. They do not
embed or redistribute the source RGB-D frames. Users requiring original TUM
data must obtain it from the dataset maintainer under its applicable terms.

This scope record is an engineering and provenance statement, not legal advice.
