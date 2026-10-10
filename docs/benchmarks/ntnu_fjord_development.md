# NTNU fjord_1 sensor-only development

This sequence is development data. The publisher's accompanying `.tum` files
are ReAqROVIO estimates, not independent hardware truth. No such reference
coordinates were downloaded or read for the comparisons below. This work does
not establish an accuracy improvement or SoTA.

The native bag is pinned to Hugging Face dataset revision
`d79489fc4742ebe66f872d1b195d9f0a249925da`. All 12,456,215,809 bytes match
publisher Git LFS SHA256
`b0f2a43d049efcc139b7629fc813a14460fb3e3090a1fa6cff3af8135a3aaa6d`.
The HTTP ETag is a separate object identity, not that checksum. TLS and range
validation remained enabled. Interrupted ranges require identity and hash
checks; unmatched retained bytes are backed up before replacement. One flock
prevents duplicate acquisition processes.

The native ROS record scan verified 292,712 records and topic counts, without
deserializing withheld navigation/position bodies. Every selected stereo,
Alphasense IMU and pressure packet was copied and independently checked against
native bytes, ordering and bag times. Full selected sensor bodies were decoded
and checked while preparing replay input:

| Sensor | Native packets | Replay prefix |
| --- | ---: | ---: |
| Alphasense IMU | 63,698 | 18,032 |
| cam0 | 6,368 | 1,800 |
| cam1 | 6,367 | 0 |
| FluidPressure | 15,926 | 4,503 |

Camera counts and initial timestamps differ. This comparison deliberately uses
cam0 mono input; it does not fabricate stereo synchronization. Both cameras
remain in the native and isolated bags. A complete packet ledger names every
prefix or camera-profile exclusion. Camera correction is zero; IMU correction
is the publisher launch value -1,770,000 ns, separate from the AQUALOC clock
contract. The prefix is native cam0 ordinals 0..1799, fixed before trials or
reference access. It spans about 90 seconds, not the full sequence.

`verify_ntnu_mono.py` independently verified every replay sensor value, image,
ordinal and corrected timestamp against ROS packets. Each observation uses
the same bounded 105 ms IMU lookahead in both arms. SDK calls subtract an integer
IMU origin before converting to double seconds; output safe times map back to
exact native events. Unsupported images must be explicitly excluded in the
ledger, not silently dropped; pressure boundary/initialization exclusions
remain explicit events.

The pressure mapping is the publisher callback's `-(raw-2660)/241`. It is not
an assumption that the ROS field contains SI Pa. A first supported initialized
sample anchors the relative depth at the current safe z; subsequent samples
use that fixed anchor. Pressure update noise diagonal is explicitly 0.01 and
fixed offsets zero. Both arms consume every pressure clock tick.

The numerical core is ReAqROVIO commit
`7fc95c1fe97c8a4e89a1477d5699807718f37e43`, lightweight_filtering
`b35bfbf5d2dc9ef91ee877ce50966f34c73c44b7`, and kindr
`22a19ddd30ccfeba72bb2c35361e090bca7d16ec`. Upstream checkouts remain clean.
The adapter is an explicit modified method, not a reproduction of the missing
old `51cea95` method or the published multi-camera ReAqROVIO evaluation.

Each version registers its complete resolved configuration, source/runtime
hashes, input, four trials, timeout and scoring rules before execution. The
freeze retains every failure, partial trajectory, output hash and runtime.
SE(3), scale=1, maximum interpolation gap 0.1 s and initialization/support
rules are declared; scoring remains unexecuted without independent truth.

The original pressure constructor indexes six diagonal entries of a
three-dimensional noise matrix. A separately licensed overlay changes only
that loop bound. Versions v1-v3 still failed with nonfinite estimator state in
all four trials. v2 fixed the refractive index, and v3 used publisher underwater
equidistant calibration; neither resolved the failure. Their registrations and
failures remain immutable. The sequence has not become a fresh holdout.

Further investigation reproduced an independent SDK arithmetic defect:
equal-duration gyro samples 0 and 1 produce a merged mean -0.5 instead of 0.5.
The anchor difference has its operands reversed in both EKF and UKF mean code.
A separate v4 overlay corrects that direction. The same executable regression
fails against original code and passes against corrected code, including a
partial final interval with expected mean 1/3. UKF mean code receives the same
operand correction, but these integration regressions exercise EKF.

The mean correction alone did not resolve the real-data numerical failures.
v4 also failed in all four trials. v5 changed the supported image update mode
from iterative direct intensity to feature reprojection EKF. Its four trials
consumed all observations and retained finite state, but the SDK's strict
processing upper bound left the final camera update pending. Its freeze is
retained; it is not the final processed-prefix result. The v1-v5 log label
`total_processed_updates` is misleading: that SDK counter resets on each safe
update call and reports the last call, not a cumulative total. No processing
count is inferred from that label.

v6 uses a processing horizon one representable relative-double step beyond the
current event, while every observation timestamp stays unchanged. This handles
the SDK's open upper bound without fitting or shifting any sensor clock.
Its independent SDK boundary fixture checks that an observation at exactly
0.04 s is processed at exactly 0.04 s. The fixture initially expected an empty
timeline after processing; that check failed because SDK `clean()` intentionally
keeps one processed sentinel. The failure/source/binary remain retained. The
corrected gate checks future pending timestamps rather than container emptiness.
v6 counts processed camera and pressure epochs between successive safe states,
excluding the sentinel, and requires every queued update processed at the end.

v6 completed all four trials. Each processed 1,800 native cam0 updates and
consumed 4,503 pressure packets. The depth arm processed all 4,502 queued
pressure updates after its one initial anchor; the ablation arm queued none.
All four reached the exact native prefix endpoint
1700604866727344151 ns. Their common scoring support is
[1700604776878215326, 1700604866727344151] ns, about 89.849 seconds after
excluding initialization. Runtimes were 16.554/18.741 s for VIO and
20.466/20.583 s for pressure. These timings are cloud wall times, not a controlled
hardware speed comparison.

Final freeze SHA256:
`e4986d9b4605a959baafadbe148e279a39a22d40e4078f9b9ef3f60aff251e90`.
Exact per-trial supports, initialization exclusions, every output hash, all
earlier outcomes and validation results are in
`ntnu_fjord_development_results.json`. There is no RMSE score for this sequence.
Independent underwater hardware truth and a full competitor evaluation remain
required; the legacy official MoCap sensor shares returned 404, and the current
HF trajectory estimates do not replace that truth. No Tank application or
external contact was made.

Artifacts are retained under
`/workspace/aqua-data/development/ntnu_fjord_01`; SDK, acquisition, build and
failure logs are under `/workspace/aqua-next/ntnu`. Data completeness,
successful estimator execution and independent accuracy readiness are separate
gates. `check_ntnu_snapshot.py` verifies retained native/input/source/runtime
and all frozen outcomes without reference access or rebuilding frozen binaries.
