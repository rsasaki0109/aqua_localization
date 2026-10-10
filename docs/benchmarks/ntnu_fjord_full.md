# Full native fjord_1 development replay

This extends the frozen v6 90-second prefix to the whole native cam0 sequence,
about 318 seconds. It is still development data. Publisher `.tum` trajectories
are ReAqROVIO estimates, not independent hardware truth; no reference coordinates
were acquired or read. There is no accuracy score or SoTA claim.

The publisher-pinned bag, native record verification, selected raw sensor bag,
all six prefix registrations and every earlier failure remain unchanged. See
`ntnu_fjord_development.md` for acquisition provenance, numerical SDK patches,
and the distinction from the unavailable old `51cea95` implementation.

The new input registration `full-input-v1/plan.json` precedes conversion. It
selects all 6,368 native cam0 images, 63,698 IMU packets and 15,926 pressure
packets. All 6,367 cam1 images remain in the native and isolated bags, explicitly
outside this mono camera profile. `verify_ntnu_full.py` independently compares
every replay value, image, ordinal and timestamp to selected native ROS bodies.
Camera correction remains zero and publisher IMU correction -1,770,000 ns.
Input wire and complete packet ledger hashes are recorded in the results.

The last pressure packet, native ordinal 15,925, has timestamp
1700605095194369664 ns: 6,132,534 ns after the corrected last IMU. Its bytes and
event remain present in both arms. Its explicit disposition is
`outside_available_imu_support`, so it submits no estimator update. The first
supported pressure establishes the fixed relative-depth anchor. Consequently,
the pressure arm has 15,924 eligible depth updates; the ablation arm has zero.
No sensor timestamp, time correction, scale or calibration is fitted to results.

Near the native IMU end, the common 105 ms lookahead saturates at the available
last IMU. The SDK's normal 100 ms queue wait can then leave supported images or
pressure pending. The new adapter disables this wait **only after successfully
reading the entire registered input**. Its finalization event `E` is an execution
event bounded by the last native IMU, not a synthetic sensor measurement. It
processes queued supported observations without predicting beyond available
IMU data. Output state times remain exact native events. The last camera is
1700605095125093130 ns, within support.

An executable test against the actual SDK reproduces a supported pressure at
0.24 seconds pending under the normal wait with native IMU ending at 0.25
seconds. The complete-input finalization processes it at exactly 0.24 seconds,
without extrapolation or retiming. A separate ROS input test retains both early
and late unsupported pressure and explicitly masks unsupported camera input.
These synthetic fixtures check contracts, not real-world accuracy.

`full-comparison-v1/plan.json` registers the isolated new adapter binary, sources,
resolved configuration, immutable previous freezes, input, two trials per arm,
600-second timeout and scoring contract before any full trial. Calibration,
pressure mapping, noise, feature reprojection EKF and numerical core are the
fixed v6 method. This is a modified mono ROVIO adapter, not the published
multi-camera refractive ReAqROVIO method. The sole execution change is native
end-of-stream handling.

All outcomes, output hashes, initialization exclusions, individual supports,
common support and cloud wall times are in `ntnu_fjord_full_results.json`.

| Trial | Outcome | Wall seconds | Initialization poses excluded | Support end, ns |
| --- | --- | ---: | ---: | ---: |
| VIO 1 | success | 61.061 | 3 | 1700605095125093130 |
| VIO 2 | success | 67.974 | 3 | 1700605095125093130 |
| Pressure 1 | success | 65.992 | 12 | 1700605095174369664 |
| Pressure 2 | success | 62.455 | 12 | 1700605095174369664 |

All scoring supports begin at 1700604776878215326 ns after initialization.
The four-trial intersection is [1700604776878215326, 1700605095125093130] ns,
318.246877804 seconds. Both repeat trajectory hashes match within each arm.
All 6,368 camera updates and every queued pressure update were processed in
every trial. Cloud timings are not a controlled hardware speed comparison.
Freeze SHA256 is
`c6d359fa008151b8b23687de11f17149fc0bf0e1349d1c4268086e9bbb013929`.

`check_ntnu_full_snapshot.py` checks source/runtime/input/output hashes, every
native camera/pressure ordinal, supported processing counts, the explicit
unsupported-pressure event and lack of extrapolation. Together with the original
`check_ntnu_fjord.bash`, `check_ntnu_full.bash` validates restoration without
rebuilding frozen binaries or accessing reference coordinates.

The retained artifacts are under
`/workspace/aqua-data/development/ntnu_fjord_01/full-input-v1` and
`full-comparison-v1`; the new isolated SDK adapter build and regression binary
are under `/workspace/aqua-next/ntnu`. Complete native acquisition, successful
estimator replay and independent accuracy readiness remain separate gates.
Independent underwater hardware truth and competitive-method evaluation are
still required for the SoTA objective. The user's skipped Tank application is
unchanged; no external contact or application was made.
