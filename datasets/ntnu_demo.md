# NTNU Underwater Dataset — Public Demo Note (fjord_1)

This note records the first public-data bringup of `aqua_localization` against the NTNU underwater
dataset, prepared on 2026-05-07 from the suggested first-task in `PLAN.md`.

## Source

- Repository: <https://huggingface.co/datasets/ntnu-arl/underwater-datasets>
- License: BSD-3-Clause (per the dataset card).
- Subset: `subset-fjord`
- Sequence: `fjord_1`
- Length: 142 m, 312 s
- Platform: Ariel (custom underwater robot, BlueROV2-Heavy class).
- Recording size on disk: 12.5 GB (raw ROS 1 `.bag`).

The dataset card download command used:

```python
from huggingface_hub import snapshot_download
snapshot_download(
    repo_id="ntnu-arl/underwater-datasets",
    repo_type="dataset",
    local_dir="aqua_localization/datasets/public/ntnu",
    allow_patterns=["subset-fjord/fjord_1/*", "README*", "LICENSE*", "calibrations/*"],
)
```

Raw bag location after download:

```text
aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1.bag
aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_baseline.tum
```

## Convert ROS 1 → ROS 2

The dataset is published as ROS 1 `.bag`. Convert to a small filtered rosbag2/mcap that drops the five
camera streams and keeps only what the IMU/depth localization stack needs:

```bash
rosbags-convert \
  --src aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1.bag \
  --dst aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_ros2 \
  --dst-storage mcap \
  --dst-typestore ros2_jazzy \
  --include-topic /alphasense_driver_ros/imu /mavros/imu/data /mavros/imu/static_pressure /mavros/rangefinder/rangefinder /tf
```

Filtered output is approximately 41 MB and contains 318.5 s of replay data.

## Detected Topics (after conversion)

`ros2 run aqua_localization inspect_bag_topics.py` output on the converted bag:

| Topic                              | Type                            | Messages |
|-----------------------------------|---------------------------------|---------:|
| `/alphasense_driver_ros/imu`      | `sensor_msgs/msg/Imu`           |    63698 |
| `/mavros/imu/data`                | `sensor_msgs/msg/Imu`           |     6371 |
| `/mavros/imu/static_pressure`     | `sensor_msgs/msg/FluidPressure` |    15926 |
| `/mavros/rangefinder/rangefinder` | `sensor_msgs/msg/Range`         |    15926 |
| `/tf`                             | `tf2_msgs/msg/TFMessage`        |    91597 |

Suggested role mapping:

- `imu`: `/mavros/imu/data` (50 Hz, base_link-aligned via MAVROS)
- `pressure`: `/mavros/imu/static_pressure` (native `sensor_msgs/msg/FluidPressure`, 50 Hz)
- No scalar barometer, no depth scalar, no sonar.

The bag exposes pressure as native `FluidPressure`, so the `scalar_to_pressure_node` adapter is **not**
needed for this sequence.

The high-rate `/alphasense_driver_ros/imu` is published in the Alphasense frame and would need a static
transform to `base_link` before the UKF can use it. The MAVROS IMU is already body-frame, which is what
the current `aqua_imu_loc` UKF expects, so the demo uses MAVROS IMU.

## Sequence-specific config

`aqua_imu_loc/config/ntnu_fjord.yaml` is a NTNU-specific starter that aligns the topic names
(`/mavros/imu/data` and `/mavros/imu/static_pressure`) and inherits the BlueROV2 UKF tuning baseline.
It is installed automatically by the `aqua_imu_loc` package (the whole `config/` is exported).

## Replay command (IMU + pressure only)

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_ros2 \
  use_sim_time:=true \
  imu_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/ntnu_fjord.yaml \
  enable_sonar_loc:=false \
  enable_fusion:=false \
  enable_rviz:=true
```

Notes:

- The bag publishes `/tf`. With `use_sim_time:=true` set on the stack and `ros2 bag play` driving sim
  time, the bag's TF messages and the `aqua_imu_loc` `map -> odom -> base_link` chain coexist cleanly
  because the bag's `/tf` does not contain `map -> odom`.
- `enable_sonar_loc:=false enable_fusion:=false` disables sonar and fusion. With fusion disabled,
  `aqua_imu_loc` owns the `map -> odom` and `odom -> base_link` transforms.
- This sequence is freshwater-side of the seawater density spectrum (Trondheim Fjord). The starter YAML
  uses `water_density_kg_m3: 1025.0`. Adjust if a tighter depth fit is required.

## Verification topics

While replay runs, the following topics should be observable:

```bash
ros2 topic list
ros2 topic echo /aqua_imu_loc/status --once
ros2 topic echo /aqua_imu_loc/odometry --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Record trajectory for offline comparison

`record_odometry.py` subscribes to `/aqua_imu_loc/odometry` (or any other `nav_msgs/Odometry`
topic) and writes a TUM-format file directly comparable with `fjord_1_baseline.tum`:

```bash
# In one terminal: replay and run the stack.
ros2 launch aqua_localization ntnu_fjord_demo.launch.py enable_rviz:=false

# In another terminal: capture odometry to TUM.
ros2 run aqua_localization record_odometry.py \
  --topic /aqua_imu_loc/odometry \
  --out aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_aqua_imu_loc.tum \
  --format tum
```

For a richer CSV with per-axis covariance and twist, pass `--format csv`.

`record_status.py` mirrors the same idea for `aqua_msgs/EstimatorStatus`:

```bash
ros2 run aqua_localization record_status.py \
  --topic /aqua_imu_loc/status \
  --out /tmp/fjord_1_status.csv
```

The CSV captures `accel_bias[xyz]`, `gyro_bias[xyz]`, and the AHRS hook flags so bias convergence
and observation activity can be plotted directly without RViz.

## Pre-converting scalar pressure bags (other datasets)

`fjord_1` already publishes native `FluidPressure` so the runtime adapter is not needed. For datasets
that only expose a scalar barometer/pressure/depth topic (e.g. NTNU `mclab` if a future export uses
`std_msgs/Float64`, AQUALOC after Float-only export, or simulator bags with `/depth`), the
`convert_scalar_pressure_bag.py` utility rewrites the source bag in place once, preserving the
original timestamps:

```bash
ros2 run aqua_localization convert_scalar_pressure_bag.py \
  --src /path/to/source_bag \
  --dst /path/to/source_bag_pressure \
  --scalar-topic /barometer \
  --pressure-topic /pressure \
  --mode ntnu_barometer \
  --barometer-pressure-offset <from sequence calibration> \
  --barometer-pressure-scale  <from sequence calibration> \
  --replace
```

Modes mirror `scalar_to_pressure_node`: `pressure_pa`, `depth_m`, `ntnu_barometer`. The output bag is
a regular rosbag2 directory and can be played by `replay.launch.py` (or `ntnu_fjord_demo.launch.py`)
without enabling the runtime scalar adapter.

## Quantitative comparison with the baseline

`compare_trajectories.py` is a numpy-only TUM comparator. It interpolates the estimate to the
reference timestamps and aligns with a rigid SE(3) Umeyama solution by default:

```bash
ros2 run aqua_localization compare_trajectories.py \
  aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_baseline.tum \
  /tmp/fjord_1_aqua_imu_loc.tum
```

A representative IMU-only run on `fjord_1` produced (rigid SE(3) alignment, `bag_rate:=10.0`):

| metric        | value (m) |
|---------------|----------:|
| matched count | 17567 |
| matched duration | 251.02 s |
| APE mean      | 1134.14 |
| APE median    | 1087.52 |
| APE RMSE      | 1228.77 |
| APE max       | 2043.92 |

The baseline trajectory is ~142 m long, so the IMU-only horizontal drift is roughly an order of
magnitude larger than the path itself. This is the expected MVP behavior with no DVL, sonar, visual,
or acoustic aiding — the depth channel from `/mavros/imu/static_pressure` keeps the vertical estimate
bounded while horizontal dead reckoning drifts. Use sonar matching, fusion, or future DVL/visual
inputs to bring horizontal error down.

For Sim(3) (with-scale) alignment, pass `--scale`. Pass `--save-aligned out.tum` to also save the
aligned positions for plotting.

### Plot trajectories (PNG thumbnail)

```bash
ros2 run aqua_localization plot_trajectories.py \
  aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_baseline.tum \
  /tmp/fjord_1_aqua_imu_loc.tum \
  --out aqua_localization/docs/media/public_demo_thumbnail.png
```

Three panels: XY at reference scale (baseline visible, drift exits frame), XY at estimate scale
(full drift envelope), and Z over samples (shows pressure-bound depth and the rigid-alignment
rotation of z into x/y).

### Yaw-frame diagnosis tool

Use `diagnose_yaw_frame.py` to compare gyro-integrated yaw and AHRS quaternion yaw delta:

```bash
ros2 run aqua_localization diagnose_yaw_frame.py \
  aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_ros2 \
  --imu-topic /mavros/imu/data \
  --csv-out /tmp/fjord_1_yaw_diag.csv
```

The output slope between AHRS yaw and gyro-integrated yaw answers whether the AHRS uses
the same body-frame convention. For `fjord_1` the slope is +1.014 (same convention) but
the RMSE difference is ~19°, which is enough to make a tight yaw observation thrash the
UKF position. Full analysis lives in [docs/benchmarks/fjord_1_yaw_frame.md](
../docs/benchmarks/fjord_1_yaw_frame.md).

### AHRS quaternion -> 3-axis gyro bias (alternative)

A 3-axis variant `imu.use_ahrs_gyro_bias_xyz` derives body angular velocity from the
AHRS quaternion via a small-angle log map between consecutive samples and observes all
three gyro bias states. Empirically on `fjord_1` this is no better than the z-only hook
because AHRS roll/pitch are accelerometer-anchored and degrade under dynamic motion.
The z-only hook is the default; the 3-axis hook remains available for tank/sim platforms
where roll/pitch from the AHRS are clean.

### AHRS yaw rate -> gyro_z bias soft observation (recommended on this sequence)

`aqua_imu_loc` ships a separate hook `imu.use_ahrs_gyro_bias_z` that uses the AHRS quaternion's
numerical yaw-rate as a *soft* observation of the gyro_z bias state. It **does not rotate the
position state**, so it avoids the position discontinuity that breaks the direct yaw observation.
On `fjord_1` with `var=1e-2`, `sub=20` the mean APE drops by ~10–30 % vs the no-AHRS baseline.
This is enabled by default in `aqua_imu_loc/config/ntnu_fjord.yaml`.

### Yaw observation from /mavros/imu/data

`aqua_imu_loc` ships an optional `imu.use_orientation_yaw` hook that reads the AHRS quaternion from
`sensor_msgs/Imu.orientation` and updates the UKF yaw state at a subsampled rate. The ntnu_fjord
config keeps it disabled because empirical APE on `fjord_1` got worse with the hook enabled — the
MAVROS AHRS frame disagrees with the UKF body-integration frame in a way that is not corrected by
simple delta-yaw subtraction. The infrastructure (UKF `update_yaw`, parameters, gtest) is in place
to enable per-vehicle once a frame mapping is confirmed against ground truth.

Tested values on `fjord_1` (lower is better, ~250 s replay at 10× rate):

| run                                         | APE mean (m) | APE RMSE (m) |
|---------------------------------------------|-------------:|-------------:|
| baseline (no yaw obs, no static-bias init)  |       1134.1 |       1228.8 |
| yaw obs, var=0.05, subsample=5              |       2401.1 |       2679.5 |
| yaw obs delta-frame, var=0.05, subsample=5  |       1944.9 |       2112.0 |
| yaw obs delta-frame, var=1.0, subsample=25  |       1620.3 |       1703.7 |

### Static-bias initialization on this sequence

`aqua_imu_loc` runs an optional `init.static_bias` warmup that averages gyro readings over the first
few seconds of the bag and seeds the UKF gyro-bias state. On `fjord_1` the bag begins with the
vehicle already maneuvering (the support boat deployment is not in the recording), so the warmup
correctly aborts:

```text
[aqua_imu_loc]: Static bias initialization aborted: motion detected during warmup;
gyro_bias remains zero.
```

The feature reduces yaw drift on tank/static-start datasets (simulators, BlueROV2 deck deployment).
For sequences like `fjord_1` it is a no-op safety check — the UKF still estimates biases through
process noise and depth-update cross-covariance, just from a zero prior.

## Honest status

This is a bringup-quality demo:

- The vehicle is BlueROV2-Heavy class but UKF tuning is not sequence-tuned.
- IMU bias is not yet estimated as a full error-state inertial estimator would.
- Horizontal dead reckoning will drift; the depth channel from `/mavros/imu/static_pressure` is what
  keeps the vertical estimate bounded.
- The TUM baseline at `fjord_1_baseline.tum` is from the upstream paper's ReAqROVIO baseline and is not
  used by `aqua_localization` yet. It is available for offline trajectory comparison.

## Next steps

- Capture a 60-120 s screen recording of the replay against `aqua_localization/rviz/demo.rviz` for the
  README placeholder video link.
- Compare `/aqua_imu_loc/odometry` to `fjord_1_baseline.tum` offline for a quantitative error number.
- Optionally upgrade to `/alphasense_driver_ros/imu` after wiring a static `alphasense_imu -> base_link`
  TF and validating units/conventions.
