# AQUALOC freeze-gated scoring contract

The old scoring implementation and registration at local commit `51cea95` are
unavailable. The new registration records the following protocol but marks
scoring blocked until the actual estimator, calibration and independent checker
have been registered and all four trials frozen. This document is a protocol,
not an executed evaluation.

1. Register a new method and input contract with exact source, upstream, native
   data, derived input, estimator configuration and calibration hashes before
   reference acquisition. Preserve every previous plan and failure. If changing
   any registered input/source, use a new version.
2. Run ordinary VIO and pressure VIO twice each. Record every planned success or
   failure, exit status, output hashes, measured runtime, initialization decision
   and excluded timestamps. Do not discard failed runs or substitute favorable
   retries. Freeze all four outcomes before fetching or reading the reference.
3. Acquire the reference only after the freeze gate passes. Record its source
   and hash separately from native sensor data. AQUALOC COLMAP/depth reference
   is not an independent hardware measurement.
4. Match integer timestamps exactly or interpolate a bracketed reference interval
   no longer than 0.1 seconds; never extrapolate. Use rigid SE(3) alignment and
   scale=1. Do not optimize time offset, scale or calibration against results.
5. Report per-trial supported windows, initialization exclusions, interpolation
   exclusions, coverage, successes/failures and runtimes. Report each trial's
   own support and the intersection common to all four trials. If a trial fails,
   an all-four comparison is unavailable; do not quietly intersect successes only.
6. Independently recompute time correspondence and rigid alignment using Horn's
   quaternion method. Compare the correspondence, transforms, sample counts and
   metrics with the primary scorer, and preserve disagreement as a failure.

The registration utility saves `scoring-plan.json` and its SHA-256 in the new
comparison plan. There is no estimator freeze or scored harbor_05 metric yet.
