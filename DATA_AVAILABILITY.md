# Data availability

Included derived evidence comprises:

- historical frozen 60-run/9,480-invocation records;
- separate historical diagnostic summaries;
- independent extension 60-successful-run/9,480-invocation records;
- explicitly batch-separated 120-run summaries;
- g2o/Ceres replay results for 50 completed condition records over five
  independent diagnostic runs, including nested call and round tables.

Original TUM RGB-D and ICL-NUIM sensor frames, ground truth, associations, raw
run directories, trajectories, replay snapshot JSON, binaries, model weights,
and private environment logs are not redistributed. Obtain source datasets from
their maintainers under their applicable terms.

The batches retain separate identities. Diagnostic data must not be pooled with
formal accuracy results, failed extension attempts are not counted as successful
runs, and replay calls/rounds are not independent replicates. Public path fields
were normalised without changing numeric results or cryptographic input hashes.
