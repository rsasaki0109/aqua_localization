# fjord_1 yaw-frame diagnosis

Run of `scripts/diagnose_yaw_frame.py` against `/mavros/imu/data` in
`subset-fjord/fjord_1/fjord_1_ros2`:

```text
samples:                  6371
duration:                 318.51 s
final gyro-integrated yaw +18.0010 rad
final AHRS yaw delta:     +18.1852 rad
final diff (ahrs - gyro): +0.1843 rad
RMSE of diff over time:   0.3300 rad
slope ahrs vs gyro:       +1.013977
```

## Interpretation

The slope is essentially +1.0, so the MAVROS AHRS yaw and the body-frame gyro integration
share the same convention on this sequence. The earlier hypothesis that ENU/NED frame
disagreement was breaking `imu.use_orientation_yaw` is **wrong**.

The actual issue is in the residual:

- The cumulative gyro yaw and AHRS yaw delta drift apart by only 0.18 rad (~10°) over
  318 s, which is small.
- But the **RMSE of the instantaneous difference is 0.33 rad (~19°)**, meaning at any
  moment the two yaw signals can disagree by tens of degrees even though they end up
  near each other.
- When the UKF integrates body-frame velocity using its own (gyro-derived) yaw and we
  then snap yaw to the AHRS value, position accumulated under the previous yaw is now
  expressed in a yaw-rotated body frame. Repeated observations rotate the velocity
  back and forth, injecting motion that did not happen and pushing horizontal APE up.

## Why yaw obs makes APE worse despite same convention

The UKF state has no direct cross-correlation between yaw and (x, y) at startup, so a
yaw measurement update only changes yaw and gyro biases (through the cross-covariance
that develops over time). Position drift accumulated under an earlier yaw is not
retroactively corrected — it is just continued forward in a slightly different yaw frame.
With sub-second AHRS/gyro disagreements of 10–20°, the integrated body velocity gets
re-projected into different world-frame directions, producing larger horizontal drift
than letting the gyro alone integrate consistently.

## Implications

- The static-bias warmup is the right shape but aborts on `fjord_1` because deployment
  is already underway when the bag starts.
- The `imu.use_orientation_yaw` hook is not broken — it is the right answer for vehicles
  whose AHRS is tightly aligned with the gyro at every instant (e.g. simulator or a
  carefully calibrated MEMS+mag system in still water). For `fjord_1` we leave it off.
- A cleaner improvement path is to fuse AHRS-derived **yaw rate** (numerical derivative
  of AHRS yaw) into the gyro bias channel rather than yaw itself, or to rotate the
  velocity state when yaw is corrected. Both are larger UKF redesigns and are deferred.

## Reproduction

```bash
ros2 run aqua_localization diagnose_yaw_frame.py \
  aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_ros2 \
  --imu-topic /mavros/imu/data \
  --csv-out /tmp/fjord_1_yaw_diag.csv
```

The CSV has columns `t,gyro_yaw_rad,ahrs_yaw_delta_rad,diff_rad`.
