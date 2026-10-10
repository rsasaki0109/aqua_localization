# HAUD official hardware-reference qualification

The official SJTU page returned HTTP200 with TLS verification on2026-10-10.
It describes10 underwater stereo/IMU sequences in a4x2m pool, stereo30Hz672x376,
IMU400Hz and motion capture150Hz. During evaluation the submerged camera marker
is not directly visible to MoCap: its pose is inferred from an above-water
marker through a calibrated fixed arm. This is independent hardware provenance,
not image-derived COLMAP truth. The publisher's millimeter accuracy claim is
not independently verified here; calibration uncertainty and timing still need
qualification. [haud_qualification.json](haud_qualification.json) binds actual
metadata and restored-environment verification receipts.

The official distribution is
<https://pan.baidu.com/s/1NPcXUx7oJ8jh0JZtSdjjZA>, public extraction code `batc`.
The share metadata request was denied by the proxy with CONNECT403; the publisher
itself did not return an HTTP response. The next required host is `pan.baidu.com`.
No native sensors, calibration values or reference coordinates were acquired.
The official page does not list pressure; do not assume a pressure comparison
is possible. A native sensor/file listing is required before selecting methods.

[haud_acquisition_embargo_plan_v1.json](haud_acquisition_embargo_plan_v1.json)
is a metadata-stage acquisition and reference embargo registration, not a
completed executable method/input registration. Select separated raw images,
IMU, timestamps and calibration first. Evaluation ROS bags contain Vicon topics
and must not be acquired before the trial freeze under this contract. If only
mixed evaluation bags are available, report that incompatibility rather than
quietly acquiring reference coordinates. Calibrations specify marker and IMU
frames and quaternion order; qualify these before implementing comparisons.

After sensor acquisition/validation, separately register exact methods,
calibration, runtime, input and scoring versions; run every method twice and
freeze all successes/failures before acquiring GT. Score SE(3), scale1, gap<=0.1s,
with independent correspondence/Horn verification and all support/exclusion/
runtime accounting. No clock, scale or calibration adjustment to the reference.
No accuracy evaluation or SoTA claim has been made. Tank applications stay skipped.
