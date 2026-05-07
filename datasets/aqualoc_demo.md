# AQUALOC public-data demo for `aqua_imu_loc`

This document describes how to download, convert, and replay an AQUALOC
sequence through `aqua_imu_loc` for the IMU + pressure dead-reckoning demo.
The dataset is hosted by LIRMM (Université de Montpellier).

## Source

- Project page: <http://www.lirmm.fr/aqualoc/>
- Download: seafile share at
  <https://seafile.lirmm.fr/d/79b03788f29148ca84e5/>
- Paper: Ferrera, Creuze, Moras, Trouvé-Peloux, *AQUALOC: An Underwater
  Dataset for Visual-Inertial-Pressure Localization*, IJRR 2019.
- Platform: LIRMM ROV "Dumbo", monocular monochromatic camera, MEMS IMU,
  pressure sensor, all on a Jetson TX2 running Ubuntu 16.04 + ROS.
- Locations: harbor of La Ciotat (a few meters depth) and two deep
  archaeological sites at 270 m and 380 m.

The harbor sequences are short (11–69 m of trajectory) and the smallest bag
in the entire dataset is `harbor_sequence_07_bag.tar.gz` (≈ 495 MB
compressed, 23.87 m of motion). Use that for the first replay.

## Download

The seafile share has a working API; pick the smallest harbor bag:

```bash
mkdir -p aqua_localization/datasets/public/aqualoc
cd aqua_localization/datasets/public/aqualoc

wget --max-redirect=20 \
  "https://seafile.lirmm.fr/d/79b03788f29148ca84e5/files/?p=/Harbor_sites_sequences/harbor_sequence_07_bag.tar.gz&dl=1" \
  -O harbor_sequence_07_bag.tar.gz

tar xzf harbor_sequence_07_bag.tar.gz
```

This unpacks a ROS 1 `.bag` file (`aqualoc_harbor_sequence_07.bag` or similar
— the tarball ships a single `.bag` per sequence).

## Convert ROS 1 → ROS 2

`aqua_localization` runs on ROS 2 Jazzy. Use the `rosbags-convert` Python
tool to migrate the bag to mcap:

```bash
pip install --user "rosbags>=0.10"

tar xzf harbor_sequence_07_bag.tar.gz   # unpacks to bag_files/harbor_sequence_7.bag
rosbags-convert --src bag_files/harbor_sequence_7.bag \
  --dst harbor_sequence_07_ros2 \
  --dst-storage mcap
```

The conversion preserves the original topic names. The result is a
`harbor_sequence_07_ros2/` directory containing `metadata.yaml` and an mcap
file.

## Detected topics (after conversion)

| Topic                       | Type                              | Messages | Rate (Hz) | Use |
|-----------------------------|-----------------------------------|---------:|----------:|-----|
| `/rtimulib_node/imu`        | `sensor_msgs/msg/Imu`             |   22 601 |   ~225    | aqua_imu_loc IMU input |
| `/rtimulib_node/mag`        | `sensor_msgs/msg/MagneticField`   |   22 601 |   ~225    | (mag track — not used) |
| `/barometer_node/pressure`  | `sensor_msgs/msg/FluidPressure`   |    1 079 |   ~10.8   | aqua_imu_loc pressure update |
| `/barometer_node/depth`     | `sensor_msgs/msg/FluidPressure`   |    1 079 |   ~10.8   | (a duplicate FluidPressure track) |
| `/camera/image_raw`         | `sensor_msgs/msg/Image`           |    2 261 |   ~22.5   | (visual track — not used by this MVP) |
| `/camera/camera_info`       | `sensor_msgs/msg/CameraInfo`      |    2 261 |   ~22.5   | (calibration) |

Although the topic is `rtimulib_node/imu`, the orientation field on this
sequence is published as the zero quaternion `(0, 0, 0, 0)` with a zero
covariance — RTIMULib does not run its AHRS fusion here, the bag carries
only raw accel + gyro + magnetometer. `aqua_imu_loc`'s AHRS hooks
(`use_orientation_yaw`, `use_ahrs_gyro_bias_*`) silently no-op on a
zero-length quaternion regardless of the parameter values, so the AQUALOC
profile keeps them disabled and the UKF integrates orientation from gyro
alone (with the static-bias initializer for the brief still window at the
start of each sequence).

## Sequence-specific config

`aqua_imu_loc/config/aqualoc.yaml` is the AQUALOC starter profile. Compared
to the BlueROV2 default, it:

- subscribes to AQUALOC's `/imu/data` and `/barometer/pressure` topics,
- disables the AHRS hooks (the AQUALOC IMU does not publish an orientation
  field — the bags carry raw accel/gyro only),
- enables the static-bias initializer (each harbor sequence starts with a
  short still window at the surface),
- uses Mediterranean seawater density (1025 kg/m³),
- enables a small linear drag coefficient (0.05) consistent with the slow,
  near-seabed ROV motion.

## Replay

Two-terminal bring-up. Adjust the bag path if you placed it elsewhere:

```bash
# Terminal A: aqua_imu_loc with the AQUALOC profile.
ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file $(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/aqualoc.yaml \
  -p use_sim_time:=true

# Terminal B: bag.
ros2 bag play \
  aqua_localization/datasets/public/aqualoc/harbor_sequence_07_ros2 \
  --clock
```

## Verification topics

```bash
ros2 topic hz /aqua_imu_loc/odometry           # ~200 Hz
ros2 topic echo --once /aqua_imu_loc/status
```

## Record trajectory for offline comparison

```bash
ros2 run aqua_localization record_odometry.py \
  --topic /aqua_imu_loc/odometry \
  --out /tmp/aqualoc_harbor_07_aqua_imu_loc.tum \
  --format tum
```

The reference Colmap-derived ground-truth file is included in the AQUALOC
calibration files archive (one `.txt` per sequence following the
`# img_number tx ty tz qx qy qz qw` format described in the dataset
README). Convert that file to the standard TUM format
(`timestamp tx ty tz qx qy qz qw`) before passing it to
`compare_trajectories.py`.

## Honest status

Harbor sequence 07 is a 23.87 m, low-speed ROV trajectory. The AQUALOC IMU
is mounted on its side: a stationary sample reads
`linear_acceleration ≈ (-0.73, -9.71, 0.63) m/s²`, i.e. gravity points along
the body's -Y axis rather than the body's -Z axis assumed by REP-145. The
profile above sets `imu.mount.rotation_rpy_rad: [-π/2, 0, 0]` to pre-rotate
each sample so the UKF sees the standard "Z up" frame; with that
rotation in place the gravity-subtraction is correct.

End-to-end accuracy on this bag is **not yet validated**. Even with the
mount rotation applied, integrated position drifts by tens of meters over
30 s of replay because the bag starts with the ROV already in motion, so
the static-bias initializer never observes a still window and the IMU
biases stay near zero. Improving this requires either a tightly-coupled
aiding source (visual or visual-inertial — AQUALOC ships the camera but the
visual front end is not yet wired into `aqua_localization`) or a longer
manually-selected still window before the bag's diving sequence starts.

The bring-up wiring (download, ROS 2 conversion, topic discovery, mount
rotation, configuration) is fully documented and the
`harbor_sequence_07_ros2` bag is available locally for whoever picks up the
visual-inertial work.

AQUALOC does not ship sonar data, so this profile does not exercise
`aqua_sonar_loc` or `aqua_fusion`. The forward-looking visual track is not
in scope for the first AQUALOC demo.
