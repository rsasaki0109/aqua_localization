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

Camera image topics are dropped by default. Pass `--include-cameras` to keep
them for visual odometry experiments. The converter writes ROS 2
Humble-friendly `sqlite3` bags by default; use `--storage mcap` only when the
target environment has the MCAP rosbag2 storage plugin installed.

```bash
ros2 run aqua_localization convert_tank_dataset_bag.py \
  --src aqua_localization/datasets/public/tank_dataset/short_test.bag \
  --dst aqua_localization/datasets/public/tank_dataset/short_test_ros2_with_cameras \
  --include-cameras
```

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

## Experimental stereo visual frontend

The package includes a lightweight ROS 2 stereo visual odometry frontend for
Tank-style compressed stereo topics. It is not a full SLAM system like
AQUA-SLAM: it has no loop closure, local map optimization, relocalization, or
IMU coupling. It is useful as the first camera-only ROS 2 baseline and as a
future position-aiding source.

```bash
# Terminal A: stereo ORB + PnP visual odometry.
ros2 run aqua_localization stereo_visual_odometry.py --ros-args \
  -p use_sim_time:=true \
  -p camera.fx:=655.0 \
  -p camera.fy:=655.0 \
  -p camera.cx:=306.0 \
  -p camera.cy:=256.0 \
  -p camera.bf:=78.89165891925023 \
  -p matching.max_stereo_descriptor_distance:=96.0 \
  -p matching.max_temporal_descriptor_distance:=96.0 \
  -p diagnostics.status_csv_path:=/tmp/tank_short_test_visual_status.csv

# Terminal B: camera-included bag.
ros2 bag play \
  aqua_localization/datasets/public/tank_dataset/short_test_ros2_with_cameras \
  --clock
```

Record the visual trajectory just like the IMU/DVL estimate:

```bash
ros2 run aqua_localization record_odometry.py \
  --topic /aqua_visual_frontend/odometry \
  --out /tmp/tank_short_test_visual_frontend.tum \
  --format tum
```

Estimate the stereo translation scale on a calibration sequence:

```bash
ros2 run aqua_localization calibrate_visual_scale.py \
  /tmp/tank_short_test_gt.tum \
  /tmp/tank_short_test_visual_frontend.tum \
  --ros-args
```

On the current `short_test` run this reports
`tracking.translation_scale:=0.169623465`. Use that value only as a diagnostic
for `short_test`; for a publishable result, calibrate on one sequence and report
accuracy on a different held-out sequence.

For held-out validation after you have access to additional Tank Dataset bags:

```bash
ros2 run aqua_localization validate_visual_scale.py \
  --calibration-reference /tmp/tank_structure_easy_gt.tum \
  --calibration-estimate /tmp/tank_structure_easy_visual.tum \
  --validation-reference /tmp/tank_medium_gt.tum \
  --validation-estimate /tmp/tank_medium_visual.tum \
  --calibration-sequence Structure_Easy \
  --validation-sequence Medium \
  --markdown
```

The script prints the scale learned on the calibration pair and a Markdown row
for the held-out validation pair. If the same path is reused for both, it emits
a warning because that is only a diagnostic fit.

To bundle the visual frontend replay, trajectory recording, scale diagnostic,
and benchmark-row generation into one command:

```bash
ros2 run aqua_localization run_tank_visual_benchmark.py \
  --bag aqua_localization/datasets/public/tank_dataset/short_test_ros2_with_cameras \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_short_test \
  --sequence short_test
```

In bag replay mode, the runner writes the recorded visual TUM file, a
`calibrate_visual_scale.py` report, a visual-frontend status CSV, a Markdown
benchmark row, drift and motion-segment reports, and a replay shell script
containing the exact ROS commands it used. If you already have a visual TUM
estimate, skip ROS replay and evaluate it directly:

```bash
ros2 run aqua_localization run_tank_visual_benchmark.py \
  --estimate /tmp/tank_short_test_visual_frontend.tum \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_short_test \
  --sequence short_test
```

The published pose is in a `visual_odom -> camera_left` frame. Treat it as an
experimental visual frontend output until a calibrated camera-to-base extrinsic
and fusion path are wired.

The frontend also publishes JSON diagnostics on `/aqua_visual_frontend/status`.
The status stream and optional CSV include per-frame `left_features`,
`right_features`, `stereo_matches`, `stereo_points`, `temporal_matches`,
`pnp_inliers`, `inlier_ratio`, `step_translation_m`, disparity statistics,
depth statistics, and the accept/reject reason. Use this before tuning
thresholds: low stereo points means image/stereo triangulation is the bottleneck,
low disparity or a long depth tail means metric scale is sensitive to pixel
noise, while low PnP inliers means temporal tracking or outlier rejection is the
bottleneck.
The `matching.max_stereo_descriptor_distance` and
`matching.max_temporal_descriptor_distance` parameters reject weak ORB Hamming
matches before triangulation and PnP; set either to `0.0` only for an ablation
run that intentionally disables that descriptor-distance filter.

Sweep those thresholds with:

```bash
ros2 run aqua_localization run_tank_visual_matching_sweep.py \
  --bag aqua_localization/datasets/public/tank_dataset/short_test_ros2_with_cameras \
  --reference /tmp/tank_short_test_gt.tum \
  --out-dir /tmp/aqua_tank_visual_matching_sweep \
  --sequence short_test \
  --translation-scale 0.169623465 \
  --pairs 64:64,80:80,96:96,112:112,disabled:disabled
```

The sweep writes `visual_matching_sweep.md` with RMSE, matched duration,
acceptance rate, median PnP inliers, and median temporal matches for each
setting. Use `--matrix` with `--stereo-distances` and `--temporal-distances` for
a full Cartesian grid when runtime is acceptable.

Summarize the CSV after a run:

```bash
ros2 run aqua_localization summarize_visual_frontend_status.py \
  /tmp/tank_short_test_visual_status.csv \
  --summary-out /tmp/tank_short_test_visual_status.md
```

The benchmark runner creates the same summary automatically in bag replay mode.

Analyze visual scale stability and drift from the reference and visual TUM files:

```bash
ros2 run aqua_localization analyze_visual_drift.py \
  /tmp/tank_short_test_gt.tum \
  /tmp/tank_short_test_visual_frontend.tum \
  --window-s 3.0 \
  --stride-s 1.0 \
  --out /tmp/tank_short_test_visual_drift.md
```

The report shows each sliding window's SE(3) RMSE, Sim(3) RMSE, and Sim(3)
scale. If window scales are stable but Sim(3) error still grows, tune drift and
geometry. If window scales vary, tune stereo scale or calibration first. The
benchmark runner writes this drift report automatically next to the scale report
and benchmark row.

Analyze short relative motion segments directly:

```bash
ros2 run aqua_localization analyze_visual_motion_segments.py \
  /tmp/tank_short_test_gt.tum \
  /tmp/tank_short_test_visual_frontend.tum \
  --segment-s 1.0 \
  --stride-s 0.5 \
  --out /tmp/tank_short_test_visual_motion_segments.md
```

This report compares each segment's visual path length against the reference
path length. Direction buckets help spot camera-frame or extrinsic-dependent
bias, while speed and segment-length statistics expose short-baseline PnP noise.
The benchmark runner writes this report automatically as
`*_visual_motion_segments.md`.

On `short_test`, the first camera-only run processed 272 stereo pairs and
accepted 271 visual odometry steps. With the nominal stereo scale it produced
1.36 m SE(3) APE RMSE against AprilTag GT over the published 11.35 s window.
The same trajectory falls to 0.096 m under Sim(3) alignment, showing that the
shape is useful but the metric scale needs calibration. For diagnostics only,
setting `tracking.translation_scale:=0.169623465` gives 0.095 m SE(3) APE RMSE
on this sequence; do not treat that same-sequence scale fit as a paper-safe
result.

The visual odometry can also be fed into `aqua_imu_loc` as a position update:

```bash
ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file $(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/tank_dataset.yaml \
  -p use_sim_time:=true \
  -p topics.visual_odometry:=/aqua_visual_frontend/odometry
```

With the same-sequence visual scale fit and a stronger visual covariance floor
(`imu.visual.position_variance_floor:=0.0025`), the fused IMU + pressure + DVL
+ visual run reaches 0.373 m SE(3) APE RMSE. This is an engineering diagnostic:
the next validation step is out-of-sequence scale calibration and a calibrated
camera-to-base transform.

## Verification topics

```bash
ros2 topic hz /aqua_imu_loc/odometry           # ~333 Hz
ros2 topic echo --once /aqua_imu_loc/status
ros2 topic echo --once /aqua_visual_frontend/status
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
  count  : 5399
  mean   : 0.380
  median : 0.401
  rmse   : 0.429
  std    : 0.200
  min    : 0.016
  max    : 0.765
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
