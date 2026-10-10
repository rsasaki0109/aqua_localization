# AQUALOC VIO / pressure recovery

## Recovery on 2026-10-10

The restored repository starts at `5bb429de1402be9c385507556a0a910c890bf673`.
The working tree was clean before this work; the new branch is
`codex/harbor05-input-support`. All public remote heads and pull-request heads
were checked. The user-reported old local commit `51cea95` and branch
`codex/mbes-feedback-reordering` were not found. No unreachable local object or
alternate workspace copy supplied the missing implementation. Its GitHub commit
URL returned 404. This is evidence of an unavailable commit, not proof that it
never existed privately.

The old comparison/scoring plans, failures, data, readiness reports, and
`/workspace/aqua-cloud` helpers were absent. They have not been recreated and
presented as originals. The new helpers and failure logs are separately dated.
The user does not know another recovery location.

The old failures were reported as `camera has no initial IMU support`
(`openvins_stream.py`) and `observation before native IMU support`
(`openvins_pressure_stream.py`). These are handover observations, not failures
newly executed here; the original failure files were not restored.

Missing originals include `openvins_stream.py`, `openvins_pressure_stream.py`,
`verify_openvins_stream.py`, `run_openvins.py`, `openvins_native.cpp`,
`openvins_pressure.hpp`, `score_vio_comparison.py`, `check_vio_scores.py`,
`download_verified_ranges.py`, frozen estimator configuration/calibration, and
the independent score checker. The newly written `prepare_vio_comparison.py`
and `verify_openvins_stream.py` are new implementations for the v2 contract,
not recovered originals.

Upstream OpenVINS is checked out at exactly
`69488123ed9362dd44b6f28e7f4680abbff1442b`. The standalone library was built
with ROS disabled and ArUco enabled, and dynamically loaded successfully in the
prepared ROS Jazzy container. This verifies a development prerequisite; no VIO
trajectory was produced by that check. It does not restore the pressure method.

## New input contract

`native-imu-support-v2` uses integer native header nanoseconds. Camera timestamps
receive the fixed offset **-40380655 ns**; pressure timestamps are unchanged.
No IMU samples are extrapolated and no offset is fitted to reference data.

Observations outside the closed native IMU timestamp interval are excluded from
the derived input for both comparison arms. Every observation remains in the
original native bag and the audit retains its original sensor ordinal, header
and bag timestamps, raw-message SHA-256, corrected timestamp, inclusion decision,
reason, and distance outside the interval. In addition,
the ledger also identifies both surrounding native IMU sample ordinals/timestamps
for every eligible observation (one repeated ordinal for an exact IMU timestamp).
A consuming adapter must buffer through the upper IMU sample before feeding that
observation. Timestamp-sorted bag order alone does not supply that lookahead.
For the reported harbor_05 boundary,
the new registration requires exactly camera ordinal 0 to precede IMU support
by **6537851 ns**, with no other camera or pressure exclusions. A discrepancy
fails conversion and requires a new registration; it is not silently accepted.

The supported ROS1 bag contains all IMU messages, eligible cameras, and eligible
pressure messages. Both arms use the same camera mask; the baseline ignores the
pressure topic. Camera header and derived bag timestamps use the corrected
clock. A future adapter must apply **zero additional camera offset** and disable
online camera time-offset calibration: applying the fixed correction again
inside OpenVINS would be a different, invalid input contract. Original image
data are retained. The converter rereads the derived bag
to verify header timestamps, ordering, sensor counts and native support, and
checks that the original bag hash did not change. A separate verifier checks
every native IMU/pressure raw-message hash and every eligible image hash after
restoring its native header clock. Its exclusion ledger is
published before writing the derived bag; failures preserve that ledger, the
partial bag, and a failure report. Existing output directories cannot be reused
or overwritten. This format still requires a registered estimator adapter.

The registration snapshots source files under a new `vio-comparison-v2`
directory and saves separate comparison and scoring plans before conversion.
It expressly marks estimator execution **blocked_missing_frozen_method**.
Input conversion is runnable; four estimator runs and scoring are not represented
as executed. Recovering the old method or defining a replacement requires another
version containing the actual implementation/configuration/calibration hashes.

## Native data acquisition

The only downloaded dataset object is the official harbor_05 native bag archive.
The expected archive (793867402 bytes) SHA-256 is
`b62c4075cfe7f53757c491a73cf2f6b5fb061a28b5fd01f2c1bb7096c252cc47`.
The expected native bag (1155720393 bytes) SHA-256 is
`109c97c1d1b655226b9ca1da312c154b28b3cc8464d58150b7cd4d91e6ba3ecb`.
These are **the user's observations from the old environment**, not publicly
published checksums from the distributor.

Acquisition keeps TLS verification enabled, checks HEAD size/ETag, and validates
every ranged response's status, exact range, length and ETag. Before a partial
file is reused every retained byte is checked against a contiguous previous
range ledger and its SHA-256 hashes under the same object size/ETag. Bytes without
that evidence are compared again with verified remote ranges. An incomplete or
corrupt ledger cannot justify reuse. Per-range hashes and all failed attempts
are retained. Expired public download redirects are renewed at most once per
failed request, with a fresh HTTPS HEAD that must identify the same size/ETag
and host; an unchanged or mismatching redirect fails and preserves the partial.
One advisory lock
prevents concurrent acquisition/extraction pipelines. Full SHA-256 verification
precedes archive use. Gzip is read through EOF to verify its footer; unsafe TAR
paths, links, devices and duplicate paths are rejected. Retained extraction bytes
are compared with the verified archive before resuming. Only the native bag is
extracted. All ROS messages are decoded before native readiness is published.

`ready.json` means complete native acquisition and verification, requiring 86126
ROS messages including 34576 IMU, 3458 images and 1650 pressure messages.
It does **not** mean stream conversion, estimator execution or scoring succeeded.
Initialization must wait in the foreground for this complete verification.
Do not declare initialization or publication ready with only a partial archive.

## Reproduction commands

Run Python inside the prepared container with `/workspace/.aqua-cloud/ros`:

```bash
/workspace/.aqua-cloud/ros python3 /workspace/aqua_localization/scripts/benchmarks/ensure_harbor05_data.py --root /workspace/aqua-data/independent/aqualoc_harbor_05
/workspace/.aqua-cloud/ros python3 /workspace/aqua_localization/scripts/benchmarks/prepare_vio_comparison.py --root /workspace/aqua-data/independent/aqualoc_harbor_05
/workspace/.aqua-cloud/ros python3 /workspace/aqua_localization/scripts/benchmarks/openvins_supported_bag.py --bag /workspace/aqua-data/independent/aqualoc_harbor_05/native/bag_files/harbor_sequence_5.bag --out /workspace/aqua-data/independent/aqualoc_harbor_05/vio-comparison-v2/input --plan /workspace/aqua-data/independent/aqualoc_harbor_05/vio-comparison-v2/plan.json
```

Registration/conversion output paths are exclusive. Inspect existing artifacts
before rerunning; use a new version rather than deleting history.

Tests under `tests/benchmarks` exercise the reported timestamp boundary, explicit
accounting, preserved image payload, immutable artifacts, integrity/TLS acquisition
contracts, corrupted CRC/TAR/resume inputs, locks, and registration gates. Tiny
synthetic bags test these contracts only; they provide no accuracy evidence.

## Accuracy claims

harbor_06 was already used for reference-informed development. The old reported
two-run RMSE values (~1.943 m baseline, ~1.860 m pressure, ~4.3% improvement) are
user-provided history, not reproduced results here or independent SoTA evidence.
AQUALOC's reference is same-image COLMAP with depth-based scale, not independent
hardware ground truth. harbor_05 reference has not been acquired or read by this
work. Tank data application was skipped by the user; do not contact anyone or
request contact information. Finish the registered real-data comparison before
moving to independent hardware truth and competitive methods.

## Public calibration candidate, separate from the missing frozen method

Metadata-only investigation found a published harbor configuration in
`chintha/U-VIP-SLAM`, commit
`5fc84b5c2b9da77e6f4574ff0f33a3fc1513a68a`, file
`Data/Settings_VI_Aqualoc_harbor.yaml` (SHA-256
`229db9e2f819b9109676111306c37c814358c60432d467fe1b4dcc352b45dd69`).
The file contains camera/IMU calibration and software parameters; no trajectory
or sequence reference was accessed. Its camera delay rounds to the stated
-40380655 ns. It is retained under `/workspace/aqua-cloud/calibration-source`.

This candidate does not recover the old estimator configuration or pressure
measurement/update convention. In particular it uses `/barometer_node/depth`
with a 0.5 m noise parameter, while this repository's pressure configuration
uses a 0.2 m standard deviation and a hydrostatic pressure model. Choosing one
would define a new method, not replicate the missing fixed method. The original
comparison remains blocked. Any reconstruction needs its own implementation,
measurement units/sign/frame contract, frozen source/configuration/input hashes,
four-trial and scoring registration before reference access.

## Separately reconstructed method v3

The old fixed method remains unavailable. A new executable reconstruction uses
the pinned OpenVINS SDK and the official Seafile harbor calibration files, saved
under `docs/benchmarks/calibration`. The executable is checked against these
files before registration. Official `T_cam_imu` maps IMU into camera and is used
directly in the SDK; the earlier U-VIP-SLAM configuration comment is not used
to decide transform direction. The camera model is equidistant, 640x512. The
official IMU noise densities are 0.001 rad/s/sqrt(Hz) and 0.02 m/s²/sqrt(Hz), with
random walks 0.00005 and 0.001. Additional SDK camera offset and online calibration
are disabled. Native mono8 pixels are supplied without resizing; the pinned
SDK's default histogram equalization is used by both arms.

The new pressure model requires native `FluidPressure` in Pa, uses density
1025 kg/m³, gravity 9.80665 m/s² and 0.2 m depth standard deviation from the
existing repository configuration. It does not infer units from trajectory
errors or automatically rescale pressure. A constant offset is augmented at the
first post-initialization pressure measurement with its uncertainty and full
state cross-covariance, preserving the vertical translation gauge. Subsequent
pressure measurements update the complete joint state. Pressure is assumed
co-located with the IMU; this is an explicit new assumption, not restored old
calibration. Both arms propagate at pressure times and remove only temporary
pressure clones; the baseline ignores pressure values. Native IMU is buffered
through each observation's upper support bracket. Output positions are camera
centers, with exact corrected native camera nanoseconds restored from the SDK's
fixed relative clock origin.

`build_reconstructed_openvins.bash` builds and binds source/dependency/executable
hashes. Its native self-check exercises offset augmentation uncertainty,
cross-covariance, temporary-clone removal, pressure sign, the joint Gaussian
covariance update and rejection of unregistered pressure units. The calibration
checker compares actual executable values to the official YAML, including
transform direction and the fixed timestamp correction. These are mathematical
and integration checks, not accuracy evidence. Python contract tests additionally
check exact wire timestamps/ordinals/image bytes and incomplete registration
rejection (60 tests passed, including scoring/freeze gates).

`prepare_reconstructed_comparison.py` creates exclusive `vio-comparison-v3`
comparison/scoring plans and source snapshots after complete v2 input verification.
`run_reconstructed_openvins.py` runs both arms twice, retains every failure and
output hash, and writes `freeze.json` only after all four outcomes. Source/input/
executable hashes are rechecked between trials and before freeze. No reference
is acquired by these scripts. The reference image-number adapter must be
registered separately before reference acquisition; it is not yet available.
Results from this reconstruction must never be described as replication of the
old ~4.3% development improvement or as independent hardware/SoTA evidence.
