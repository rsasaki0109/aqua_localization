# MVP Checklist

This checklist tracks the first usable `aqua_localization` milestone.

## Done

- ROS 2 Humble/Jazzy `ament_cmake` package structure builds with `colcon`.
- `aqua_msgs` provides structured status messages:
  - `EstimatorStatus`
  - `ScanMatchingStatus`
  - `FusionStatus`
- `aqua_imu_loc` runs an additive UKF for IMU + pressure/depth dead reckoning.
- `aqua_imu_loc` publishes odometry, status, and `map -> odom -> base_link` TF when it owns TF.
- `aqua_imu_loc` exposes `/aqua_imu_loc/reset`.
- `aqua_imu_loc` includes `depth_to_pressure_node` for `std_msgs/Float64` positive-down depth adapters.
- `aqua_imu_loc` includes `scalar_to_pressure_node` for scalar pressure, depth, and NTNU-style barometer adapters.
- `aqua_imu_loc` supports simple underwater dynamics hooks:
  - gravity
  - static or externally updated water-current velocity
  - linear drag against relative water velocity
  - constant buoyancy acceleration correction
- `aqua_sonar_loc` validates sonar `PointCloud2`, republishes accepted clouds, and publishes scan matching odometry.
- `aqua_sonar_loc` supports `noop`, PCL ICP, and PCL GICP backends.
- `aqua_sonar_loc` applies post-registration quality gates (`max_fitness_score`, `max_translation_step_m`, `max_rotation_step_rad`).
- `aqua_sonar_loc` ships a submap front end (`scan_matching.submap_size > 1` with optional `use_motion_prior`).
- `aqua_imu_loc` exposes optional AHRS hooks (yaw observation, gyro_z bias from AHRS yaw rate, 3-axis gyro bias) and a static-bias initializer.
- `aqua_fusion` loosely fuses IMU/depth odometry with fresh sonar odometry.
- Top-level launch starts IMU, sonar, and fusion nodes.
- Replay launch supports `ros2 bag play`, topic remapping, `use_sim_time`, and subsystem toggles.
- Replay launch can optionally start the depth-to-pressure adapter.
- Replay and bringup launch can optionally start RViz with `aqua_localization/rviz/demo.rviz`.
- Bag inspection script suggests replay launch arguments from rosbag2 `metadata.yaml`.
- Public dataset demo video plan is tracked in `docs/public_demo_plan.md`.
- Public dataset shortlist and first-demo decision are tracked in `docs/public_dataset_candidates.md`.
- BlueROV2 starter profiles are available under each package's `config/bluerov2.yaml`.
- `uuv_simulator` starter profiles are available under each package's `config/uuv_simulator.yaml`.
- NTNU `fjord_1` starter profile in `aqua_imu_loc/config/ntnu_fjord.yaml` (with AHRS gyro_z bias hook tuned).
- MBES-SLAM `beach_pond` starter profile in `aqua_sonar_loc/config/mbes_slam.yaml` (GICP + quality gates).
- Public-data demos are validated end-to-end:
  - NTNU `subset-fjord/fjord_1` IMU + pressure replay through `aqua_imu_loc` (see `datasets/ntnu_demo.md`).
  - MBES-SLAM `beach_pond` multibeam replay through `aqua_sonar_loc` GICP (see `datasets/mbes_slam_demo.md`).
- Trajectory recording (`record_odometry.py`, `record_status.py`) and Umeyama-aligned APE comparison (`compare_trajectories.py`, `plot_trajectories.py`) ship in the metapackage.
- NTNU `fjord_1` benchmark runner (`bench_fjord_1.sh`) appends results to `docs/benchmarks/fjord_1.md`.
- TF ownership is automatic in top-level launch:
  - fusion enabled: `aqua_fusion` owns TF
  - fusion disabled: `aqua_imu_loc` owns TF
- Runtime tests cover all three main nodes.

## Verify

```bash
colcon build --symlink-install
colcon test --packages-select aqua_imu_loc aqua_sonar_loc aqua_fusion --event-handlers console_direct+
source install/setup.bash
ros2 launch aqua_localization aqua_localization.launch.py
```

For bag replay:

```bash
ros2 launch aqua_localization replay.launch.py start_bag:=true bag_path:=/path/to/rosbag2
```

## Still Research/Next Milestones

- IMU/DVL motion-prior subscription in `aqua_sonar_loc` to make submap mode useful on
  geometrically degenerate single-fan multibeam data (the submap registration code is
  already shipped; only the prior wiring is missing).
- Microstrain-IMU-only `aqua_imu_loc` profile for the MBES-SLAM bag (no pressure, depth=0).
- `aqua_fusion` end-to-end run on a real public bag (it currently has unit + runtime tests but no public-data benchmark).
- ESKF backend with error-state IMU propagation and bias handling.
- Validated sonar covariance estimation.
- NDT scan matching backend.
- DVL, visual odometry, and acoustic positioning inputs.
- Tightly coupled sonar residual fusion.
- AQUALOC + additional MBES-SLAM/OpenSonarDatasets adapters.
- Demo screen recording (60–120 s) replacing the static thumbnails.
