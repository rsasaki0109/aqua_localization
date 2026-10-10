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

The new executable reconstruction uses its own `vio-comparison-v3` registration
and four-trial freeze. Its initial scoring plan explicitly blocks reference
acquisition until the image-number adapter is fixed. `prepare_vio_scoring.py`
then creates a separate exclusive `vio-scoring-v3` version bound to those
unchanged outcomes. It requires independent evidence for zero-based image
positions from the already reference-used harbor_06 development format. It
does not select a numbering origin from harbor_05 errors. Every reference image
position maps to the full native camera ordinal and its already audited fixed
corrected header time, including the retained unsupported first image; no
reference timestamp is optimized.

`score_vio_comparison.py` validates every frozen result/output, method/runtime
hash and scoring source snapshot before creating a reference network client.
It downloads only the registered updated official trajectory object, retaining
the raw bytes, source/size/ETag/SHA-256 receipt and the preceding freeze hash.
Its checksum is a local verified-download observation, not a publisher checksum.
The image-position mapping accepts exact integer values (including integral
scientific notation), rejects duplicates/out-of-sequence indices, and sorts
by image position without selecting correspondences by error. Primary integer
bisect matching and Kabsch alignment are independently checked by a forward
time scan and Horn quaternion alignment. The score reports individual support,
all-four common timestamps, initialization/missing-state and interpolation
exclusions, failures and runtimes. Failed scoring preserves the reference,
access-start marker, acquisition receipt and a failure report.

Synthetic known geometry verifies rigid transforms, reflection rejection,
scale fixed at one, epoch-nanosecond precision, the exact 0.1 s interpolation
boundary and no extrapolation. Mutating a frozen trajectory or removing one
trial blocks reference access. These tests provide no localization accuracy
evidence. The original frozen method remains unavailable; new results must be
attributed to the separately registered reconstruction.
