# Code availability

## Included code

`scripts/reproduce_analysis.py` is a standalone offline reconstruction utility.
It validates the packaged formal records, recomputes core tables, calculates
descriptive correlations, and generates white-background figures. It is
licensed under MIT only as specified in `LICENSE_SCOPE.md`.

## Excluded code

The instrumented ORB-SLAM2-derived runtime is not included. No C++, g2o,
DBoW2, third-party solver source, model source, or Paper1 code is redistributed.
The MIT grant does not apply to those excluded components.

## Reproduction boundary

The included code reproduces offline analysis from frozen CSV files. It does
not recreate the CSV files from RGB-D input and must not be described as full
end-to-end SLAM reproducibility.

Any later runtime-source release requires a separate GPL and third-party notice
audit and is outside this repository's scope.
