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
- `aqua_sonar_loc` accepts an external IMU/DVL motion prior on a `nav_msgs/Odometry` topic (`motion_prior.topic`) and uses the relative SE(3) between bracketing samples as the registration initial guess.
- `aqua_imu_loc` supports IMU-only operation (`topics.pressure: ""`) with a configurable surface-vessel pseudo-depth hook (`imu.surface_assumption`) so a boat with no pressure sensor can dead-reckon without z drifting unbounded.
- MBES-SLAM `beach_pond` IMU-only `aqua_imu_loc` profile (`aqua_imu_loc/config/mbes_slam.yaml`) feeds `/aqua_imu_loc/odometry` into `aqua_sonar_loc` (`aqua_sonar_loc/config/mbes_slam.yaml` `motion_prior.topic`); end-to-end replay confirms the registration initial guess flows through and the estimate moves out from origin (further accuracy tuning is tracked separately).
- `aqua_imu_loc` exposes optional AHRS hooks (yaw observation, gyro_z bias from AHRS yaw rate, 3-axis gyro bias) and a static-bias initializer.
- `aqua_fusion` loosely fuses IMU/depth odometry with fresh sonar odometry.
- `aqua_fusion` MBES-SLAM `beach_pond` profile (`aqua_fusion/config/mbes_slam.yaml`) ships and is exercised end-to-end alongside IMU + sonar; the loose-coupling weighted average follows the IMU input on this geometrically degenerate dataset (the trajectory improvement requires both inputs to be reasonable, see Next Milestones for the tightly-coupled follow-up).
- `aqua_imu_loc` supports a static IMU mounting rotation (`imu.mount.rotation_rpy_rad`) so bags whose IMU axes do not match REP-145 (e.g. AQUALOC harbor sequences read gravity on -Y) can be replayed with the correct gravity subtraction.
- `aqua_sonar_loc` publishes a fitness/inliers-derived diagonal pose covariance when `scan_matching.covariance.enable_estimation` is true (default: legacy 0.25 m² / 0.10 rad² for backward compatibility). Position variance scales as `position_scale * fitness_score / inliers` and rotation variance as `rotation_scale * fitness_score / (inliers * characteristic_range_m^2)`, both clamped to `[*_floor, *_cap]`. Sanity-checked on MBES-SLAM `beach_pond` (model produces a 1-sample 0.25 m² for the init fan and floor-clamped values for subsequent fans, confirming the wiring; per-platform chi-square calibration against ground-truth error is future work).
- `aqua_imu_loc` accepts a tightly-coupled sonar position observation (`topics.sonar_odometry` + `imu.sonar.{position_variance_floor,max_age_s}`). The 3D position from `/aqua_sonar_loc/odometry` is fed as a measurement update on the UKF position state, and via the existing position↔bias cross-covariance the registration residual closes the IMU bias loop. MBES profile pre-wired (`aqua_imu_loc/config/mbes_slam.yaml`); sanity-checked end-to-end on the bag (loose-coupling drift of ±40 m in the previous fusion path drops to ~17 m now that sonar pulls the IMU state on every accepted fan instead of being weighted-averaged after the fact).
- AQUALOC harbor sequence 07 starter profile (`aqua_imu_loc/config/aqualoc.yaml`) and bring-up doc (`datasets/aqualoc_demo.md`) ship; the bag download + ROS 2 conversion + topic discovery flow is fully documented. End-to-end accuracy on this bag is not yet validated (still-window-free start means the static-bias initializer cannot observe sensor biases).
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

- Tightly-coupled IMU/sonar fusion so the trajectory output is no longer
  IMU-dead-reckoning-only on bags without GPS/DVL aiding. Today the MBES-SLAM
  IMU-only profile uses AHRS yaw + 3-axis gyro-bias hooks and a surface-vessel
  pseudo-depth assumption, but the residual horizontal velocity error over an
  8 s slice is still large and the visible estimate trail in the demo GIF does
  not track the bag's reference. Wiring is correct end-to-end (the registration
  initial guess flows through and accepted scans produce visible motion); what
  is missing is sonar-residual feedback into the IMU bias states.
- `aqua_fusion` end-to-end run on a real public bag (it currently has unit + runtime tests but no public-data benchmark).
- ESKF backend with error-state IMU propagation and bias handling. (The
  current additive-UKF backend now closes the tightly-coupled sonar-position
  observation loop, but an error-state formulation would be the principled
  way to handle attitude observations from `aqua_sonar_loc` and to add
  delayed-state smoothing once visual or DVL aiding lands.)
- Per-platform chi-square calibration of the sonar covariance scales against
  ground-truth pose error. The fitness/inliers model and the parameter knobs
  ship today; what is missing is the offline tuning loop that records
  `(estimate, covariance, GT)` triples and adjusts `position_scale`,
  `rotation_scale`, and the floors so ~95% of per-step pose errors fall
  within 2σ.
- NDT scan matching backend.
- DVL, visual odometry, and acoustic positioning inputs.
- Tightly coupled sonar residual fusion.
- AQUALOC + additional MBES-SLAM/OpenSonarDatasets adapters.
- Demo screen recording (60–120 s) replacing the static thumbnails.
