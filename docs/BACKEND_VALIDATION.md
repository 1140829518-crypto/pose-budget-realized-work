# Independent backend validation

The replay code reads a common frozen optimisation-problem snapshot and exposes
g2o and Ceres 2.2.0 backends. The formal protocol is
`CHAINED_OFFLINE_REPLAY_V1`: each of four rounds begins from the preceding
round's output and applies a shared outlier-update rule.

The public data contain 50 completed run-level records:

- five independent diagnostic runs;
- five budgets (`1, 3, 4, 5, 10`);
- two backends (g2o and Ceres 2.2.0);
- 7,590 call-result rows and 30,360 round-result rows;
- 3,795 paired backend calls.

Calls and rounds are nested observations. They must not be interpreted as
independent runs. The replay does not report ATE/RPE and does not establish
full online SLAM behaviour, backend numerical equivalence, or a universal
saturation threshold.

Snapshot JSON files are not redistributed because they retain dataset-derived
problem observations. Their identities and SHA256 values remain in the frozen
run order and result records so an authorised holder can verify them.
