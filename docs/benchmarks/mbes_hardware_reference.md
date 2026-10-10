# Hardware-reference preparation, 2026-10-10

The next task restored the retained harbor_05 native data, frozen artifacts,
Docker image and pinned Python environment. The managed runtime skill is now
available and was read; current metadata lists the Seaward source hosts, while
policy observation is `unknown`, not proof of enforcement. Actual verified-TLS
GET/range requests succeeded via `www.seaward.science`; bare-host requests were
intermittently 503. Free space initially increased to about 30 GB.

## Completed native acquisition and sensor isolation

Beach Pond archive: 2584672367 bytes, strong ETag
`"9a0ef86f-5c573e9a1d5bc"`. Locally observed archive SHA-256:
`c68d9448502aab9d436e4784453bf07f11663f6719678b01751e4992bfe111c8`.
Native bag: 7269784465 bytes, SHA-256
`75ba10f2c0c7f3c31d29115f9ffbaf120c61c52495660de58b2fa6033ea30206`.
These are local verified-download observations, not publisher checksums.
Four bounded download workers validated exact ranges, length and stable ETag;
retained ranges are rechecked against their hashes/current remote identity.
Full archive-to-range agreement, Python gzip EOF CRC/ISIZE, safe TAR paths and
every extracted byte were verified. The first acquisition source/log and stricter
full-gzip recheck remain retained, with one acquisition lock and no duplicate jobs.

The native bag index lists 14612478 messages across all topics. All message bodies
have **not** been decoded; reference position bodies remain unread. A new exclusive
`sensor-input-v1` registration permits only `/norbit/detections` and
`/nav/sensors/microstrain/imu/raw`. It copied 125488 point clouds and 947459 IMUs,
then independently checked all 1072947 selected packets, byte/order/time exactly.
It excludes all GNSS positions/headings, fused/projected navigation, processed
AHRS and TF. Full native data and metadata accounting for every connection remain.
The derived ROS1 bag is an input artifact, not estimator/evaluation readiness.
ROS 2 Jazzy/MCAP conversion also completed using pinned rosbags 0.10.11; its
metadata has exactly the same two topics and all 1072947 selected messages.
This checks ROS 2 topic/count preservation; full byte/order/time comparison
was performed on the ROS1 isolation bag, not claimed across ROS serialization.

Twelve targeted tests passed: retained-range corruption, gzip CRC corruption,
unsafe TAR paths, blocked non-TLS/foreign redirects, resume byte mismatch,
withheld-navigation leakage, missing sensor, exclusive registration and removal
of the legacy downloader's `--insecure`. Tests establish integrity, not accuracy.

## Reference and competitor gates

The official distribution page links publisher processing code, retained at
`8d99818e7b275983c8d7513fb0d6c56d98d2a45d`. Its EKF directly consumes projected
GNSS position and GNSS-derived heading. Therefore `/nav/processed/odometry` is
not independent truth for a method consuming those channels. The processing
repository is not a published SLAM competitor implementation.

Independent-reference candidates are raw `/nav/sensors/navsat/ubx_pos/fix`
(47369 messages) and its UBX PVT quality packet (47354). The embedded message
schema includes carrier solution state (`fixed` vs `float`), fix validity,
invalid latitude/longitude, horizontal/vertical accuracy and GPS time validity.
NavSatFix status alone cannot certify fixed RTK. Before reading positions or
quality bodies for scoring, register the method, output freeze and scoring
contract, including non-residual-dependent quality limits and quality/time
matching rules. Handle unmatched quality epochs explicitly; do not infer
validity or match by minimum trajectory error.

The publisher supplies `pos_2020.urdf` and its navigation launch loads that model.
Bind its physical transforms before evaluation; verify the actual sensor and
antenna frames and transform estimated sonar poses to the antenna location
using their orientations. Do not fit a lever arm or time offset against RTK.
RTK coverage, carrier state, clock/skew, antenna mapping and surface-vessel/vertical
accuracy limitations have **not** yet been validated. No estimator trial,
competitive result, hardware-reference RMSE or SoTA claim is produced here.

Beach Pond has already been used in this repository's development and covariance
calibration, so it cannot become new held-out evidence. Metadata-only probes also
confirmed current archive sizes for North River Confluence (2604173960 bytes),
Dutch Harbor (3285201946 bytes) and Wiggles Bank (6371618162 bytes). Those objects
were not downloaded or inspected for trajectories. Their sonar modalities,
storage budget and equivalent competitor implementations must be checked before
selecting a newly held-out dataset. MBES evidence applies to bathymetric
localization; it does not establish visual-inertial-pressure SoTA. Tank stays
skipped and no application/contact was sent.

Full artifacts: `/workspace/aqua-data/independent/mbes_beach_pond`.
Logs/official source metadata: `/workspace/aqua-next`.
See [artifact-backed preparation status](mbes_hardware_reference_preparation.json).

## Subsequent North River comparison

The preparation state above is historical. North River Confluence was subsequently
fully acquired and archive/native bytes verified. Its 8205282288-byte bag has
local SHA-256 `7ecd240caf879aadfbbdb0bc8dfc7f1bcae1aa94c3618d605bd3be7fc17afbb1`.
Only native sonar packets were estimator inputs. A physical pose regression
identified the inverse-registration composition bug; it failed before the fix
and passed afterwards. PCL maps current points into the previous sensor frame;
world sensor pose must compose that transform, rather than its inverse.

A new registration fixed the first 1600 packets and ran GICP and official
KISS-ICP 1.3.0 twice each. All four succeeded and output hashes were frozen
before RTK values were read. The native archive already contained reference
bytes; the gate applies to deserialization, not download. All 13 publisher
physical URDF transforms matched native fixed transforms before position access.
GICP retained one rejected translation step per run, and 1599 poses; KISS
produced 1600 poses. All native sensor counts and per-packet exclusion reasons
remain in the input ledger. No global navigation or processed AHRS was consumed.

V1 scoring failed: all 84838 fixes failed only the quality-clock-gap gate.
The embedded UBX schema explicitly distinguishes header receive time from GPS
`timestamp`. First-three diagnostics show GPS timestamps exactly matching fix
headers; receive timestamps differ by about 83 ms. A separately registered
scoring-v2 uses native GPS epochs with zero fitted offset and otherwise unchanged
quality/SE(3)/scale/interpolation/calibration rules. The original plan, scorer,
failure, ledger and four frozen outputs remain immutable. This repair occurred
after reference access and cannot be presented as freshly blind scoring.

See [current outcomes](north_river_comparison_results.json). Short surface-vessel
point-only evidence cannot establish underwater VIO/pressure SoTA. The official
NTNU underwater dataset subset 3 supplies cameras, IMU, pressure and independent
external MoCap, but its Box data route returned proxy CONNECT 403.
See [access evidence and prerequisites](ntnu_mocap_access.json).
