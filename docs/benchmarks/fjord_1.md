# fjord_1 APE history

Translation APE (rigid SE(3) Umeyama alignment) of `/aqua_imu_loc/odometry` against
`fjord_1_baseline.tum`. Each row is one run of `scripts/bench_fjord_1.sh` plus a manual
note describing the parameter change being measured.

Rerun the harness with:

```bash
ros2 run aqua_localization bench_fjord_1.sh
BENCH_NOTE="describe the change here" ros2 run aqua_localization bench_fjord_1.sh
```

| run id | bag_rate | matched count | matched s | mean (m) | rmse (m) | note |
|--------|---------:|---------------:|----------:|---------:|---------:|------|
| 2026-05-07T21-12  |     10.0 |         17567 |    251.02 |  1134.14 |  1228.77 | baseline (no static-bias init, no yaw obs) |
| 2026-05-08T04-22  |     10.0 |         14434 |    206.19 |  1292.48 |  1385.39 | static-bias init enabled, warmup aborted (vehicle moving from t=0) |
| 2026-05-08T04-46  |     10.0 |         17845 |    254.92 |  2401.05 |  2679.49 | yaw obs enabled, var=0.05, sub=5, absolute frame (snap-to-AHRS) |
| 2026-05-08T04-49  |     10.0 |         17052 |    243.59 |  1944.88 |  2112.02 | yaw obs delta-frame, var=0.05, sub=5 |
| 2026-05-08T04-51  |     10.0 |         18230 |    245.57 |  1620.32 |  1703.66 | yaw obs delta-frame, loose anchor: var=1.0, sub=25 |
| 2026-05-08T05-23-29 |     10.0 |         13464 |    192.34 |   720.58 |   786.73 | smoke test of bench_fjord_1.sh harness (20 s record window) |
| 2026-05-08T05-24-37 | 10.0 | 9741 | 139.15 | 481.4954 | 546.6428 | post-fix smoke (insertion below table) |
| 2026-05-08T05-49-05 | 10.0 | 21102 | 301.47 | 925.9730 | 1050.6948 | AHRS yaw rate -> gyro_z bias soft observation (var=1e-3, sub=10) |
| 2026-05-08T05-50-11 | 10.0 | 20986 | 300.03 | 1618.2948 | 1772.2463 | AHRS yaw-rate -> bz, tighter var=1e-4 sub=5 |
| 2026-05-08T05-51-26 | 10.0 | 21157 | 302.23 | 792.7760 | 881.3618 | AHRS yaw-rate -> bz, looser var=1e-2 sub=20 |
| 2026-05-08T05-52-21 | 10.0 | 20912 | 298.73 | 1077.3351 | 1144.9760 | AHRS yaw-rate -> bz, very loose var=1e-1 sub=25 |
| 2026-05-08T05-53-14 | 10.0 | 21076 | 301.07 | 760.6728 | 828.7742 | AHRS yaw-rate -> bz, very loose var=1e-1 sub=25 |
| 2026-05-08T05-54-05 | 10.0 | 21131 | 301.87 | 1227.6450 | 1332.3711 | AHRS yaw-rate -> bz, very loose var=1e-1 sub=25 |
| 2026-05-08T05-55-11 | 10.0 | 21147 | 302.09 | 845.0024 | 912.8536 | AHRS yaw-rate -> bz, var=5e-3 sub=10 |
| 2026-05-08T05-56-14 | 10.0 | 21084 | 301.19 | 892.1239 | 926.9247 | AHRS yaw-rate -> bz, var=2e-2 sub=20 |
| 2026-05-08T05-57-20 | 10.0 | 20091 | 287.02 | 1015.1585 | 1081.8091 | AHRS yaw-rate -> bz, var=1e-2 sub=20 (best, repeat) |
| 2026-05-08T06-16-14 | 10.0 | 20905 | 298.65 | 1142.3554 | 1279.2628 | AHRS gyro_bias xyz, var=1e-2 sub=20 (run 1) |
| 2026-05-08T06-17-11 | 10.0 | 20915 | 298.77 | 773.0247 | 883.1948 | AHRS gyro_bias xyz, var=1e-2 sub=20 (run 2) |
| 2026-05-08T06-18-07 | 10.0 | 19591 | 280.62 | 1052.0791 | 1155.9633 | AHRS gyro_bias xyz, var=1e-2 sub=20 (run 3) |
| 2026-05-08T06-19-12 | 10.0 | 20989 | 300.03 | 1249.0770 | 1286.5116 | AHRS xyz, x/y var=5e-2 (loose) z var=1e-2 (run 1) |
| 2026-05-08T06-20-10 | 10.0 | 20893 | 298.47 | 1310.0875 | 1437.0512 | AHRS xyz, x/y var=5e-2 z var=1e-2 (run 2) |

## Honest takeaways

- The trajectory is ~142 m long; an IMU-only dead-reckoning APE of ~1.1 km after rigid alignment
  is ~8x the trajectory length. This is the expected MVP behavior without DVL/sonar/visual aiding.
- Enabling the AHRS yaw observation made things worse on `fjord_1`. Initial hypothesis was a
  body-frame convention mismatch, but `scripts/diagnose_yaw_frame.py` shows the slope between AHRS
  yaw and gyro-integrated yaw is +1.014 (same convention). The RMSE of the instantaneous difference
  is ~19° over 318 s, which is enough for a tight yaw measurement update to repeatedly re-project
  integrated body velocity and inflate horizontal APE. Full analysis: [fjord_1_yaw_frame.md](
  fjord_1_yaw_frame.md).
- `init.static_bias` correctly aborts on `fjord_1` because the bag begins after deployment is
  already underway. It is a no-op for this sequence and should be re-evaluated on tank/sim data.
- The `imu.use_ahrs_gyro_bias_z` hook feeds the difference between AHRS yaw rate and the raw
  `angular_velocity.z` into a soft observation of the gyro_z bias state. Unlike the direct yaw
  observation, it does **not** rotate the position/velocity state, so it cannot introduce
  position discontinuities. With var=1e-2, sub=20 the mean APE drops from ~1100 m to ~800–1000 m
  (run-to-run noisy at 10× replay rate, but consistently below the no-AHRS baseline). Enabled
  by default in `aqua_imu_loc/config/ntnu_fjord.yaml`.
- The 3-axis variant `imu.use_ahrs_gyro_bias_xyz` derives body angular velocity from the AHRS
  quaternion via small-angle log map and observes all three gyro biases via
  `update_gyro_bias_xyz`. On `fjord_1` it does **not** out-perform the z-only hook (run-to-run
  mean ~770–1310 m vs z-only ~790–1015 m) because AHRS roll/pitch are accelerometer-anchored and
  degrade under dynamic motion. Left disabled by default but available for tank/sim platforms.
- Future improvements that would meaningfully reduce horizontal drift on this sequence: sonar
  scan-matching odometry, DVL bottom-lock, visual-inertial fusion, or a tighter coupling of yaw
  observation with position/velocity correction.
