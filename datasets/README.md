# Dataset Notes

Planned dataset adapters and examples:

- NTNU underwater navigation and sonar datasets
- AQUALOC underwater localization datasets
- OpenSonarDatasets and other public sonar point cloud collections
- BlueROV2/uuv_simulator ROS bag examples

Initial support should focus on rosbag replay for IMU and pressure topics, then add sonar point cloud registration benchmarks.

## Public Demo Target

The next public-facing milestone is a short localization demo video linked from the top-level README.

First track (IMU + pressure, **wired**): NTNU `subset-fjord/fjord_1`. See
[ntnu_demo.md](ntnu_demo.md) for the download, conversion, replay, AHRS-bias hook,
and APE measurement details.

Second track (sonar, **wired**): MBES-SLAM `beach_pond` (Norbit iWBMSh multibeam from
a real surface vessel). See [mbes_slam_demo.md](mbes_slam_demo.md) for the full
workflow including GICP backend, quality gates, and rigid/Sim(3) APE numbers.
For benchmark setup, use
[mbes_slam_beach_pond_acquisition.md](mbes_slam_beach_pond_acquisition.md) to
download, place, convert, and readiness-check the bag before recording loop
status artifacts.

See `../docs/public_demo_plan.md` for the recording checklist.
See `../docs/public_dataset_candidates.md` for the maintained shortlist and decisions.

## Replay Launch

Use the top-level replay launch to run the stack against an existing rosbag2 directory:

```bash
ros2 launch aqua_localization replay.launch.py start_bag:=true bag_path:=/path/to/rosbag2
```

To inspect a bag before replay and print suggested launch arguments:

```bash
ros2 run aqua_localization inspect_bag_topics.py /path/to/rosbag2
```

To start replay with the demo RViz layout:

```bash
ros2 launch aqua_localization replay.launch.py start_bag:=true bag_path:=/path/to/rosbag2 enable_rviz:=true
```

For IMU + pressure only bags:

```bash
ros2 launch aqua_localization replay.launch.py start_bag:=true bag_path:=/path/to/rosbag2 enable_sonar_loc:=false enable_fusion:=false
```

Expected default bag topics:

- `/imu/data`: `sensor_msgs/msg/Imu`
- `/pressure`: `sensor_msgs/msg/FluidPressure`
- `/sonar/points`: `sensor_msgs/msg/PointCloud2`

The initial BlueROV2 profile expects:

- `/mavros/imu/data`: `sensor_msgs/msg/Imu`
- `/bar30/pressure`: `sensor_msgs/msg/FluidPressure`
- `/fls/points`: `sensor_msgs/msg/PointCloud2`

The initial `uuv_simulator` profile expects rexrov-style bridged topics:

- `/rexrov/imu`: `sensor_msgs/msg/Imu`
- `/rexrov/pressure`: `sensor_msgs/msg/FluidPressure`
- `/rexrov/sonar/points`: `sensor_msgs/msg/PointCloud2`

If a dataset uses different topic names, pass remap arguments:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/path/to/rosbag2 \
  bag_imu_topic:=/mavros/imu/data \
  bag_pressure_topic:=/bar30/pressure \
  bag_sonar_points_topic:=/fls/points
```

The `stack_*_topic` arguments should normally stay at their defaults unless the node YAML files have been changed.

If a bag includes water-current estimates as `geometry_msgs/msg/TwistStamped`, enable the IMU drag-model input with:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/path/to/rosbag2 \
  current_velocity_topic:=/water_current
```

## BlueROV2 Profile

Use the vehicle-specific starter YAML files when replaying BlueROV2-style bags:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/path/to/rosbag2 \
  imu_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/bluerov2.yaml \
  sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/bluerov2.yaml \
  fusion_params_file:=$(ros2 pkg prefix aqua_fusion)/share/aqua_fusion/config/bluerov2.yaml
```

If the bag uses different topic names, either edit the `topics` section in the BlueROV2 YAML files or pass replay remap arguments such as `bag_imu_topic:=/imu/data`.

## uuv_simulator Profile

Use the simulator starter YAML files for rexrov-style bags or ROS 2 bridges:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/path/to/rosbag2 \
  imu_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/uuv_simulator.yaml \
  sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/uuv_simulator.yaml \
  fusion_params_file:=$(ros2 pkg prefix aqua_fusion)/share/aqua_fusion/config/uuv_simulator.yaml
```

If your simulator publishes depth rather than `FluidPressure`, convert it upstream to pressure or replay through a small adapter node before using `aqua_imu_loc`.

For `std_msgs/msg/Float64` depth in meters, positive downward, use the built-in adapter:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/path/to/rosbag2 \
  enable_depth_to_pressure:=true \
  depth_to_pressure_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/depth_to_pressure_uuv_simulator.yaml \
  imu_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/uuv_simulator.yaml \
  sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/uuv_simulator.yaml \
  fusion_params_file:=$(ros2 pkg prefix aqua_fusion)/share/aqua_fusion/config/uuv_simulator.yaml
```

If the bag depth topic is not `/rexrov/depth`, pass `bag_depth_topic:=/your/depth/topic` or edit `depth_to_pressure_uuv_simulator.yaml`.

## Scalar Pressure/Barometer Adapter

Some public datasets publish barometer, pressure, or depth as `std_msgs/msg/Float64` or
`std_msgs/msg/Float32` instead of `sensor_msgs/msg/FluidPressure`. Use `scalar_to_pressure_node`
for those bags:

```bash
ros2 launch aqua_localization replay.launch.py \
  start_bag:=true \
  bag_path:=/path/to/rosbag2 \
  enable_scalar_to_pressure:=true \
  bag_scalar_pressure_topic:=/barometer \
  scalar_to_pressure_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/scalar_to_pressure_ntnu.yaml
```

Supported modes in `scalar_to_pressure.yaml`:

- `pressure_pa`: scalar is already absolute pressure in pascals
- `depth_m`: scalar is positive-down depth in meters
- `ntnu_barometer`: scalar is converted with the NTNU dataset-card barometer formula
