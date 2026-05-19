# Architecture Notes

`aqua_localization` separates dead reckoning, sonar registration, and estimator fusion so each part can evolve independently.

## Runtime Graph

The current bringup launch starts three runtime nodes:

```text
aqua_imu_loc
  /imu/data                    sensor_msgs/msg/Imu            input
  /pressure                    sensor_msgs/msg/FluidPressure  input
  /aqua_imu_loc/odometry       nav_msgs/msg/Odometry          output
  /aqua_imu_loc/status         aqua_msgs/msg/EstimatorStatus  output
  /aqua_imu_loc/reset          std_srvs/srv/Trigger           service

aqua_sonar_loc
  /sonar/points                sensor_msgs/msg/PointCloud2    input
  /aqua_sonar_loc/points_filtered
                               sensor_msgs/msg/PointCloud2    output
  /aqua_sonar_loc/odometry     nav_msgs/msg/Odometry          output
  /aqua_sonar_loc/status       aqua_msgs/msg/ScanMatchingStatus
                                                              output

aqua_fusion
  /aqua_imu_loc/odometry       nav_msgs/msg/Odometry          input
  /aqua_sonar_loc/odometry     nav_msgs/msg/Odometry          input
  /aqua_fusion/odometry        nav_msgs/msg/Odometry          output
  /aqua_fusion/status          aqua_msgs/msg/FusionStatus     output
```

The optional pose-graph backend adds a keyframe trajectory and an explicit
loop-closure input:

```text
aqua_pose_graph
  /aqua_imu_loc/odometry or
  /aqua_fusion/odometry          nav_msgs/msg/Odometry                  input
  /aqua_pose_graph/loop_constraint
                                 aqua_msgs/msg/PoseGraphLoopConstraint input
  /aqua_pose_graph/path          nav_msgs/msg/Path                      output
  /aqua_pose_graph/keyframe_count
                                 std_msgs/msg/UInt32                   output
  /aqua_pose_graph/loop_constraint_count
                                 std_msgs/msg/UInt32                   output
```

The full stack can be launched with:

```bash
ros2 launch aqua_localization aqua_localization.launch.py
```

For rosbag replay:

```bash
ros2 launch aqua_localization replay.launch.py start_bag:=true bag_path:=/path/to/rosbag2
```

Each subsystem can be disabled from the top-level launch:

```bash
ros2 launch aqua_localization aqua_localization.launch.py enable_sonar_loc:=false
```

## MVP Boundaries

The current MVP validates the runtime graph and estimator data contracts rather than final navigation accuracy. It is suitable for:

- replaying IMU and pressure bags through `aqua_imu_loc`
- validating pressure-derived depth updates and estimator reset behavior
- feeding sonar `PointCloud2` data through preprocessing and ICP/noop scan matching
- publishing loosely fused odometry from IMU/depth and sonar odometry
- checking TF ownership and status topics during bringup

The MVP deliberately keeps the following as future work:

- ESKF production backend
- scan matcher covariance estimation from sonar geometry
- GICP/NDT and feature-based sonar matching
- DVL, visual odometry, and acoustic positioning factors
- tightly coupled sonar residual fusion
- dataset benchmark tooling and vehicle-specific calibration profiles

## Frame Contract

- `map`: global frame for future sonar/global corrections
- `odom`: locally continuous frame
- `base_link`: robot body frame
- `sonar_link`: sonar sensor frame

The TF tree is always:

```text
map -> odom
odom -> base_link
```

The standalone `aqua_imu_loc` launch publishes both edges for pure dead reckoning, with `map -> odom` as identity.
The top-level `aqua_localization` launch switches TF ownership automatically:

- `enable_fusion:=true`: `aqua_imu_loc publish.tf=false`, `aqua_fusion publish.tf=true`
- `enable_fusion:=false`: `aqua_imu_loc publish.tf=true`

This keeps one publisher for each TF edge while allowing fused odometry to own `odom -> base_link` in the full stack.

## IMU And Pressure Path

`aqua_imu_loc` is the main dead-reckoning package. The current estimator backend is an additive UKF with state:

```text
[x, y, z, vx, vy, vz, roll, pitch, yaw, accel_bias_xyz, gyro_bias_xyz]
```

The implementation is split into small testable components:

- `AdditiveUkf`: prediction and depth update
- `PressureDepthConverter`: `FluidPressure` in pascals to positive-down depth in meters
- `ImuPreprocessor`: IMU `dt` acceptance, `dt` clamp, and finite-value checks
- `imu_loc_node`: ROS subscriptions, publications, parameters, and TF

`/aqua_imu_loc/status` reports the active estimator backend, initialization state, update count, last prediction interval, and covariance traces for quick health monitoring.

`/aqua_imu_loc/reset` resets the UKF state to zero, restores initial covariance, clears IMU timestamp state, clears status counters, and reinitializes the pressure reference policy. This is useful when seeking in rosbag replay or restarting a localization segment without restarting the node.

Depth is positive downward at the sensor model boundary. The odometry state follows an ENU-style convention where positive `z` is upward, so the pressure measurement model is:

```text
depth = -z
```

The prediction model already has extension hooks for:

- gravity
- estimated water-current velocity
- simple linear drag against relative water velocity
- constant buoyancy acceleration correction

Future work should add an ESKF backend and make water current a state or external measured input.

## Sonar Path

`aqua_sonar_loc` currently provides the front half of the sonar localization pipeline:

- `SonarCloudPreprocessor`: validates `PointCloud2` x/y/z fields, checks finite points, applies a simple range gate, and enforces a minimum point count
- `ScanMatcher`: abstract scan matching interface
- `IcpScanMatcher`: PCL ICP backend with accumulated odometry transform
- `NoopScanMatcher`: identity backend for graph validation and debugging
- `sonar_loc_node`: ROS point cloud input, accepted cloud output, and sonar odometry output

The default backend is `icp`. The first accepted cloud initializes the ICP target and publishes identity odometry. Later clouds are aligned against the previous accepted cloud, and the incremental motion is accumulated into `/aqua_sonar_loc/odometry`.

`/aqua_sonar_loc/status` reports cloud acceptance counts, scan matcher backend, convergence state, fitness score, and status text.

Available scan matching backends:

- ICP
- noop

Planned scan matching backends:

- GICP
- NDT
- feature-based matching for structured sonar returns

## Pose Graph And Loop Closure Path

`aqua_pose_graph` maintains a g2o `VertexSE3` keyframe graph from upstream
odometry. Consecutive keyframes are connected with odometry edges, and
external front ends can now inject loop closures by publishing
`aqua_msgs/msg/PoseGraphLoopConstraint` to
`/aqua_pose_graph/loop_constraint`.

The loop-constraint message carries:

- `from_id`, `to_id`: existing pose-graph keyframe IDs,
- `relative_pose`: the measured transform from `from_id` to `to_id`,
- `information`: row-major 6x6 information matrix for
  `[x, y, z, roll, pitch, yaw]`,
- `optimize_after_insert`: whether to run g2o immediately after insertion.

This keeps the graph backend independent of the loop-detection front end.
For MBES data, the intended front end is: persist a bathymetric submap per
keyframe, search older submaps after a temporal/spatial exclusion window,
run submap-vs-submap GICP/NDT, and publish only geometrically consistent
relative transforms as loop constraints.

## Fusion Path

`aqua_fusion` currently implements a loosely coupled odometry fuser:

- subscribes to IMU/depth odometry
- stores the latest sonar odometry
- rejects stale sonar updates using `fusion.max_sonar_age_s`
- blends fresh sonar position into IMU odometry using `fusion.sonar_pose_weight`
- optionally uses sonar orientation directly
- publishes `/aqua_fusion/odometry`
- publishes `/aqua_fusion/status`

This is intentionally conservative. Sonar registration covariance is not yet validated. Tightly coupled residual fusion is reserved for a later implementation stage.

`/aqua_fusion/status` reports fusion mode, whether sonar was available, whether sonar was used, sonar age, blend weight, and status text.

## Configuration

Each package installs one YAML file under `config/params.yaml`. The main parameter groups are:

- `topics`: input and output topic names
- `services`: service names
- `frames`: frame IDs
- `publish`: TF/output behavior where applicable
- `ukf`: UKF sigma point and covariance parameters
- `pressure`: pressure-to-depth conversion
- `dynamics`: underwater model hooks. `aqua_imu_loc` accepts an optional
  `topics.current_velocity` `geometry_msgs/msg/TwistStamped` input in the `odom` frame.
- `preprocessing`: sonar point cloud acceptance settings
- `scan_matching`: sonar registration backend and solver settings
- `fusion`: loosely coupled fusion settings

Keep parameters externalized. Dataset-specific and vehicle-specific tuning should be added as additional YAML files rather than hard-coded in nodes.

Current vehicle starter profiles:

- `aqua_imu_loc/config/bluerov2.yaml`: MAVROS/ArduSub-style IMU, Bar30 pressure, conservative underwater dynamics
- `aqua_imu_loc/config/depth_to_pressure.yaml`: generic positive-down depth to `FluidPressure` adapter parameters
- `aqua_imu_loc/config/depth_to_pressure_uuv_simulator.yaml`: rexrov-style `/rexrov/depth` to `/rexrov/pressure` adapter parameters
- `aqua_imu_loc/config/scalar_to_pressure.yaml`: generic scalar pressure/depth/barometer to `FluidPressure` adapter parameters
- `aqua_imu_loc/config/scalar_to_pressure_ntnu.yaml`: NTNU public-data starter scalar barometer adapter parameters
- `aqua_sonar_loc/config/bluerov2.yaml`: FLS point cloud input and close-range ICP settings
- `aqua_fusion/config/bluerov2.yaml`: conservative sonar pose blending for early BlueROV2 trials
- `aqua_imu_loc/config/uuv_simulator.yaml`: rexrov-style IMU/pressure topics and tighter simulated pressure updates
- `aqua_sonar_loc/config/uuv_simulator.yaml`: rexrov-style sonar point cloud input and wider-range ICP settings
- `aqua_fusion/config/uuv_simulator.yaml`: simulator-oriented sonar pose blending for replay experiments

## Bag Replay

`aqua_localization/launch/replay.launch.py` can start the stack and optionally execute `ros2 bag play`.
`aqua_localization/scripts/inspect_bag_topics.py` reads rosbag2 `metadata.yaml`, detects likely IMU, pressure, depth, sonar point cloud, and water-current topics, and prints a suggested `replay.launch.py` command.

Important launch arguments:

- `bag_path`: rosbag2 directory
- `start_bag`: starts `ros2 bag play` when true
- `loop_bag`: adds `--loop` to bag playback
- `bag_rate`: playback rate passed to `--rate`
- `use_sim_time`: sets `use_sim_time` on stack nodes
- `current_velocity_topic`: optional `geometry_msgs/msg/TwistStamped` water-current velocity input
- `enable_depth_to_pressure`: starts the optional depth-to-pressure adapter
- `bag_depth_topic`, `stack_depth_topic`: remap depth input when the adapter is enabled
- `enable_scalar_to_pressure`: starts the optional scalar pressure/depth/barometer adapter
- `bag_scalar_pressure_topic`, `stack_scalar_pressure_topic`: remap scalar adapter input
- `enable_imu_loc`, `enable_sonar_loc`, `enable_fusion`: subsystem toggles

The default is `start_bag:=false`, so the launch file can also be used to start only the stack while replay is controlled from another terminal.

## Test Coverage

Current unit-test coverage focuses on contracts that are easy to break during estimator work:

- UKF prediction, depth update, angle normalization, and drag behavior
- pressure-to-depth conversion and first-sample reference handling
- pressure reference reset behavior on converter reconfiguration
- depth-to-pressure adapter conversion and rejection behavior
- scalar-to-pressure adapter conversion and rejection behavior
- IMU `dt` and finite-value preprocessing
- sonar cloud acceptance and rejection behavior
- scan matcher backend factory, `noop` result contract, and ICP initialization behavior
- loosely coupled fusion blending, age rejection, and covariance floors

Current runtime-test coverage starts the real ROS nodes in-process and verifies:

- `aqua_imu_loc`: IMU/pressure input, odometry output, status output, TF output, and reset service
- `aqua_sonar_loc`: `PointCloud2` input, filtered cloud output, scan matching odometry, and status output
- `aqua_fusion`: IMU/sonar odometry input, fused odometry output, fusion status, and TF output

The main regression command is:

```bash
colcon test --packages-select aqua_imu_loc aqua_sonar_loc aqua_fusion --event-handlers console_direct+
```

## Custom Messages

`aqua_msgs` currently defines:

- `EstimatorStatus`: estimator health and covariance summary
- `FusionStatus`: loosely coupled fusion policy status
- `ScanMatchingStatus`: sonar preprocessing and scan matching status

These messages are intentionally lightweight and suitable for dashboards, bag inspection, and regression tests.
