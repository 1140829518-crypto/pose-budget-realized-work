# g2o/Ceres snapshot replay

This directory contains the audited snapshot reader, model-equivalence check,
single-snapshot replay tools, four-round chained replay tools and run-level
batch driver used for the independent backend validation.

Build instructions are in the repository `INSTALL.md`. The public repository
does not include raw snapshot JSON files, executables, Ceres itself, or the
complete instrumented ORB-SLAM2-derived tree. Supply these explicitly; do not
substitute a different observation set or an output pose from the other
backend.

The source CMake file was made path-portable for publication. Solver logic is
unchanged from the audited experiment source.
