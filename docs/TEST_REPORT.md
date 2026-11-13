# Release test report

Date: 2026-11-13

| Check | Result |
|---|---|
| Historical offline reconstruction | PASS: 60 runs, 9,480 invocations |
| Release hierarchy and sensitive-path validator | PASS |
| Python syntax compilation | PASS |
| Public runner dry run | PASS: 15 planned records |
| Instrumented g2o realised-work counters | PASS |
| PoseOptimization non-interference | PASS |
| Runtime capability banner | PASS (`ACTIVE=0`, `SHADOW=1`, counters enabled) |
| Public replay CMake configuration | PASS with Ceres 2.2.0 |
| Public replay full build | PASS |
| SE(3) update and Huber cost equivalence | PASS; max Huber difference `1.7547607455981051e-07` |

The C++ checks used the already audited local dependencies and binaries. No
formal SLAM run or new scientific observation was produced.
