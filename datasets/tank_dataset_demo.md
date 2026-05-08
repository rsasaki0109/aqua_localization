# Tank Dataset public-data demo for `aqua_imu_loc`

This document describes how to download, convert, and replay the **Tank
Dataset** (Xu et al., IJRR 2025) through `aqua_imu_loc`. This is the first
public dataset wired through `aqua_localization` that exercises the new DVL
body-frame velocity fusion path, alongside IMU and pressure-derived depth.

## Source

- Project page: <https://senseroboticslab.github.io/underwater-tank-dataset/>
- Paper: Xu, Willners, Roe, Katagiri, Luczynski, Pétillot,
  *Tank dataset: An underwater multi-sensor dataset for SLAM evaluation*,
  IJRR 2025. <https://doi.org/10.1177/02783649251364904>
- Hardware: WaterLinked A50 DVL, MICROSTRAIN 3DM-GX5-AHRS IMU, Blue
  Robotics Bar 30 pressure sensor, custom underwater stereo camera.
- Ground truth: AprilTag-marker SLAM (`/apriltag_slam/GT`) at 20 Hz, fused
  with the on-board AQUA SLAM in `/apriltag_slam/GT_full` for some sequences.

The smallest sequence (`short_test`, ~15 s, ~50 MB) is the canonical first
target.

## Download

```bash
mkdir -p aqua_localization/datasets/public/tank_dataset
cd aqua_localization/datasets/public/tank_dataset

# pip install --user --break-system-packages gdown   # if not already installed
gdown "1U2APRrDJYpTHktil1evhvAsF__L42BYL" -O short_test.bag
```

## Convert ROS 1 → ROS 2 (custom DVL adapter)

The Tank Dataset uses the custom `waterlinked_a50_ros_driver/DVL` message
type for `/dvl/data`, which `rosbags-convert` cannot preserve without the
upstream package installed. `aqua_localization` ships a one-shot Python
adapter that decodes the DVL message inline (the message definition is
embedded in the bag itself), republishes it as
`/dvl/twist` (`geometry_msgs/TwistStamped`), and ALSO derives a synthetic
`/pressure` (`sensor_msgs/FluidPressure`) track from the source's
`nav_msgs/Odometry` depth.

```bash
ros2 run aqua_localization convert_tank_dataset_bag.py \
  --src aqua_localization/datasets/public/tank_dataset/short_test.bag \
  --dst aqua_localization/datasets/public/tank_dataset/short_test_ros2
```

Camera image topics are dropped by default. Pass `--include-cameras` to
keep them.

## Detected topics (after conversion)

| Topic                | Type                              | Messages | Rate (Hz) | Use |
|----------------------|-----------------------------------|---------:|----------:|-----|
| `/imu/data`          | `sensor_msgs/msg/Imu`             |    4 991 |   ~333    | aqua_imu_loc IMU input |
| `/depth/data`        | `nav_msgs/msg/Odometry`           |      450 |    ~30    | (passthrough; raw depth track) |
| `/pressure`          | `sensor_msgs/msg/FluidPressure`   |      450 |    ~30    | aqua_imu_loc pressure update (synthesised by the adapter) |
| `/dvl/twist`         | `geometry_msgs/msg/TwistStamped`  |      104 |     ~7    | aqua_imu_loc DVL velocity update |
| `/apriltag_slam/GT`  | `nav_msgs/msg/Odometry`           |      300 |    ~20    | reference for offline APE comparison |

## Sequence-specific config

`aqua_imu_loc/config/tank_dataset.yaml` is the Tank Dataset starter:

- subscribes to `/imu/data`, `/pressure`, and `/dvl/twist`,
- uses fresh-water density (1000 kg/m³),
- enables the AHRS gyro_z bias soft observation (Microstrain ships with a
  Madgwick AHRS; the orientation is mag-anchored on yaw),
- enables a 2 s static-bias window at the bag start,
- enables small linear drag (slow tank motion).

## Replay

Two-terminal bring-up:

```bash
# Terminal A: aqua_imu_loc with the Tank Dataset profile.
ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file $(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/tank_dataset.yaml \
  -p use_sim_time:=true

# Terminal B: bag.
ros2 bag play \
  aqua_localization/datasets/public/tank_dataset/short_test_ros2 \
  --clock
```

## Verification topics

```bash
ros2 topic hz /aqua_imu_loc/odometry           # ~333 Hz
ros2 topic echo --once /aqua_imu_loc/status
```

## Record trajectory for offline comparison

```bash
ros2 run aqua_localization record_odometry.py \
  --topic /aqua_imu_loc/odometry \
  --out /tmp/tank_short_test_aqua.tum \
  --format tum

# Reference: the AprilTag GT track included in the converted bag.
ros2 run aqua_localization record_odometry.py \
  --topic /apriltag_slam/GT \
  --out /tmp/tank_short_test_gt.tum \
  --format tum

# Quantitative comparison.
ros2 run aqua_localization compare_trajectories.py \
  /tmp/tank_short_test_gt.tum \
  /tmp/tank_short_test_aqua.tum
```

## Measured numbers (short_test sequence, 15 s)

Replaying the full short_test bag through the starter profile and comparing
the recorded `/aqua_imu_loc/odometry` against `/apriltag_slam/GT` after
Umeyama SE(3) alignment yields:

```
APE translation [m]
  count  : 5309
  mean   : 0.376
  median : 0.396
  rmse   : 0.426
  std    : 0.201
  min    : 0.013
  max    : 0.772
```

This validates the DVL fusion path end-to-end on real public data: the IMU
+ pressure + DVL combination keeps the trajectory error inside ~0.43 m
RMSE on a 15 s underwater tank trajectory. Tightening the per-sequence
covariances and biases is left as a follow-up task; the bring-up wiring
itself is verified.

## Honest status

The bring-up runs end-to-end on the converted bag with IMU + pressure + DVL
all fused into the UKF — this is the first `aqua_localization` public-data
demo that exercises the DVL body-frame velocity fusion path landed earlier.

The synthesised `/pressure` track derives from the source's
`nav_msgs/Odometry` z field assuming a positive-up convention with fresh
water density 1000 kg/m³. With `pressure.use_first_pressure_as_reference:
true` the absolute reference does not matter; only the variation is fed
into the UKF as a depth observation.
