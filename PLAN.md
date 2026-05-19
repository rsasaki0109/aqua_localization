# aqua_localization Plan and State of the Stack

This document is the canonical "where we are, where we are going" for
`aqua_localization`. It is rewritten each release; the previous handover-
style plan that targeted the v0 MVP is preserved in git history (last seen
at commit `f56bd01`, just before the v0.2 release).

Current date: 2026-05-09.
Latest release: [v0.2](https://github.com/rsasaki0109/aqua_localization/releases/tag/v0.2).

## Project Goal

A ROS 2 localization stack for underwater robots, designed around two
equally-important localization paths:

- high-rate dead reckoning from IMU and pressure / depth sensors,
- sonar point cloud registration from forward-looking sonar (FLS) or
  multibeam sonar.

The target robots are BlueROV2-class ROVs, custom AUVs, and simulator
platforms such as `uuv_simulator`. ROS 2 Humble and Jazzy are the primary
supported distributions. The headline path uses real public underwater
datasets — no simulation or synthetic bag in the canonical demo flow.

The project's identity:

- not a thin wrapper around `robot_localization`,
- the main IMU/depth estimator is a self-implemented additive UKF (an
  ESKF backend is on the roadmap),
- pressure/depth fusion is first-class and high priority,
- sonar scan matching is also first-class, not an afterthought,
- the architecture is ready for DVL, visual odometry, acoustic positioning,
  and tighter sonar coupling.

## Releases And What Each One Shipped

### [v0.1](https://github.com/rsasaki0109/aqua_localization/releases/tag/v0.1) — MVP snapshot

- ROS 2 ament_cmake packages: `aqua_msgs`, `aqua_imu_loc`,
  `aqua_sonar_loc`, `aqua_fusion`, `aqua_localization` metapackage.
- 15-state additive UKF in `aqua_imu_loc`: position / velocity / RPY /
  accel-bias / gyro-bias.
- Pressure / depth measurement update with seawater density conversion.
- Tightly-coupled DVL body-frame velocity update and tightly-coupled
  sonar 3D position update (registration → IMU bias loop via sigma-
  point cross-covariance).
- Optional AHRS hooks: yaw observation, gyro_z bias from AHRS yaw rate,
  3-axis gyro bias.
- Static-bias initializer with motion-detection abort.
- IMU mount rotation (`imu.mount.rotation_rpy_rad`) for non-REP-145
  sensor mounting.
- Surface-vessel pseudo-depth hook for boats without pressure.
- Configurable underwater dynamics: gravity, water-current, linear drag,
  buoyancy.
- `aqua_sonar_loc` with PCL ICP / GICP / NDT / `noop` backends, factory
  dispatched. Quality gates (`max_fitness_score`,
  `max_translation_step_m`, `max_rotation_step_rad`). Submap front end
  (concatenates last K accepted fans) with constant-velocity prior or
  external IMU/DVL motion prior.
- Fitness/inliers-derived diagonal pose covariance (off by default for
  backward compatibility, on with one flag).
- `aqua_fusion` loose coupling of IMU/depth odometry with sonar
  odometry, exercised end-to-end on MBES-SLAM `beach_pond`.
- `aqua_pose_graph` SE(3) keyframe backend with external loop-constraint
  input, plus an experimental MBES submap-vs-submap loop-closure front end.
- Web replay paths: rerun.io headless `.rrd` export with curated 3D +
  plots blueprint, plus a Lichtblick (Apache-2.0 fork of Foxglove
  Studio) layout JSON and Playwright headless driver.
- 92 unit + runtime test results were passing in the latest local validation.

### [v0.2](https://github.com/rsasaki0109/aqua_localization/releases/tag/v0.2) — pose graph backend + sonar covariance calibration

- New `aqua_pose_graph` package: SE(3) keyframe graph on g2o.
  Subscribes to upstream odometry, extracts keyframes when relative
  motion exceeds configurable thresholds, connects them with `EdgeSE3`
  constraints whose information matrix is read from the upstream
  `pose.covariance`. Runs g2o Levenberg-Marquardt optimisation on
  demand (`/aqua_pose_graph/optimize`) or every N keyframes. Publishes
  the optimised trajectory as `nav_msgs/Path` on
  `/aqua_pose_graph/path`. External front ends can inject loop constraints
  through `/aqua_pose_graph/loop_constraint`; the MBES package now ships an
  experimental submap-matching front end. 5 gtests pass.
- Chi-square calibration of sonar pose covariance:
  `aqua_localization/scripts/calibrate_sonar_covariance.py`. Reports
  observed Mahalanobis^2 distribution, recommended `position_scale`
  factor to match chi-square 3-dof targets (3.0 / 7.815 for mean /
  95th percentile), and saturation warnings when sigma is floor- or
  cap-bound. MBES-SLAM `beach_pond` calibrated end-to-end against
  `/nav/processed/odometry`: pre-tune position_scale=1.0 produced d^2
  mean=1834 vs target 3 (single-fan multibeam is geometrically
  degenerate, real residuals ~10 m vs floor-clamped sigma 0.10 m^2).
  Profile now ships `position_floor=25 m^2 / scale=100 / cap=400 m^2`.
- `aqua_pose_graph` wired into the top-level launch (off by default).
  Smoke-tested on Tank Dataset short_test: 361 keyframes, populated
  `/aqua_pose_graph/path`.

## Public-Data Demo Matrix

Four public underwater datasets, four rerun.io renderings, four matching
recorder + export scripts. Every entry is reproducible from a single
command per dataset.

| Dataset | What it shows | Recorder | rerun export |
|---------|---------------|----------|--------------|
| Tank Dataset `short_test` | DVL fusion vs AprilTag GT, **0.43 m APE RMSE** on 15 s | `record_tank_demo.sh` | `rerun_export.py` |
| MBES-SLAM `beach_pond` | Multibeam fans accumulated into a depth-coloured bathymetric scan | `record_mbes_demo.sh` | `rerun_export_mbes.py` |
| NTNU `subset-fjord/fjord_1` | Dataset SLAM baseline through a 7 m fjord dive | `record_ntnu_demo.sh` | `rerun_export_ntnu.py` |
| AQUALOC `harbor_07` | LIRMM "Dumbo" ROV underwater camera + pressure depth track | `record_aqualoc_demo.sh` | `rerun_export_aqualoc.py` |

Static screenshots are committed under `docs/media/*_rerun.png`.

## Architecture

```
                +---------------------+
                |   IMU + pressure    |
                |   + DVL + AHRS      |
                +----------+----------+
                           |
                           v
                +----------+----------+
                |    aqua_imu_loc     |  additive UKF, tightly-coupled DVL
                |  (15-state UKF)     |  + sonar-position residual updates
                +----------+----------+
                           |
                           |  /aqua_imu_loc/odometry (with covariance)
                           v
       +-------------------+-------------------+
       |                   |                   |
       v                   v                   v
+------+--------+   +------+-------+   +------+------------+
|  aqua_fusion  |   | aqua_sonar_  |   |  aqua_pose_graph  |
|  loose-coupl  |<--|     loc      |   |  (g2o SE(3) KF    |
|  IMU + sonar  |   |  ICP/GICP/   |   |   graph + loop    |
|     fusion    |   |  NDT,        |   |   constraints)    |
+---------------+   |  submap FE,  |   +------+------------+
                    |  motion prior|          |
                    +------+-------+          |
                           |                  v
                           |          /aqua_pose_graph/path
                           v
                  /aqua_sonar_loc/odometry
                  (fitness/inliers covariance)
```

TF ownership is automatic in the top-level launch:

- `aqua_fusion` owns `map -> odom -> base_link` when fusion is enabled,
- `aqua_imu_loc` owns it otherwise.

## Verified Commands

```bash
# Build everything.
colcon build --symlink-install

# Run unit + runtime tests.
colcon test --packages-select \
  aqua_imu_loc aqua_sonar_loc aqua_fusion aqua_pose_graph aqua_localization \
  --event-handlers console_direct+

# Source and launch (default: imu_loc + sonar_loc + fusion, pose graph off).
source install/setup.bash
ros2 launch aqua_localization aqua_localization.launch.py

# Top-level launch with pose graph on.
ros2 launch aqua_localization aqua_localization.launch.py \
  enable_pose_graph:=true
```

## Recording And Visualising A Public-Data Demo

Each dataset recorder produces a results-included `.mcap` that bundles
the source sensor topics with `aqua_imu_loc` and (where applicable)
`aqua_sonar_loc` outputs:

```bash
ros2 run aqua_localization record_tank_demo.sh
ros2 run aqua_localization record_mbes_demo.sh
ros2 run aqua_localization record_ntnu_demo.sh
ros2 run aqua_localization record_aqualoc_demo.sh
```

Then export to a rerun.io recording and screenshot:

```bash
ros2 run aqua_localization rerun_export.py \
  --bag aqua_localization/datasets/public/tank_dataset/demo_with_estimate \
  --out /tmp/tank.rrd
rerun --screenshot-to /tmp/tank.png --window-size 1920x1080 /tmp/tank.rrd
```

The Lichtblick path (committable layout JSON,
`docs/foxglove/aqua_tank_demo.json`) is parallel:

```bash
ros2 run aqua_localization lichtblick_screenshot.py \
  --bag aqua_localization/datasets/public/tank_dataset/demo_with_estimate \
  --layout docs/foxglove/aqua_tank_demo.json \
  --out docs/media/tank_dataset_lichtblick.png
```

## Honest Limitations As Of v0.2

- IMU-only dead reckoning drifts roughly an order of magnitude on bags
  without DVL/visual aiding. NTNU `fjord_1` and AQUALOC `harbor_07`
  exhibit hundreds-of-meters XY drift; depth `z(t)` tracks well thanks
  to the pressure-update loop.
- Single-fan multibeam registration is geometrically degenerate.
  Tightly-coupled sonar feedback narrows MBES-SLAM `beach_pond` fusion
  drift from ±40 m loose-coupling to ~17 m, but the per-fan residuals
  are still ~10 m magnitude.
- The pose graph backend and experimental MBES loop-closure front end now
  generate and consume loop constraints, but the MBES thresholds,
  candidate reliability, and information matrix still need a full real-bag
  tuning pass.
- Per-platform sonar covariance calibration produced sensible numbers
  on MBES with 22 accepted fans. More accepted fans (e.g. via an
  OpenSonarDatasets bag with longer overlapping geometry) would tighten
  the statistical estimate.
- `aqua_fusion` has unit + runtime tests but no per-platform accuracy
  benchmark history (separate from the bench_fjord_1 harness which is
  IMU-only).

## v0.3+ Roadmap

The headline next milestone is **reliable real-data loop closure**. The
pose graph backend and experimental MBES front end are in place; what
remains is measurement evidence, stronger candidate selection, and
calibrated constraints.

Two natural directions, can be developed in parallel:

1. **Bathymetric submap-vs-submap loop closure (MBES path).** The first
   front end persists accumulated multibeam submaps at pose-graph
   keyframes, searches older odometry-near candidates, runs ICP/GICP/NDT,
   and injects accepted constraints into `aqua_pose_graph`. Next work is
   real-bag threshold sweeps, false-positive analysis, descriptor-based
   candidate filtering, and information-matrix calibration.
2. **Visual loop closure (AQUALOC path).** OpenCV ORB features + a
   bag-of-words descriptor (DBoW2 or a lighter-weight equivalent) per
   keyframe. On each new keyframe, query the descriptor index for
   matches above a similarity threshold, compute essential matrix +
   triangulated scale (depth from pressure or DVL), and inject the
   constraint. First target dataset is AQUALOC `harbor_07` (clear
   harbor water, dense seafloor texture).

Other roadmap items:

- ESKF backend with error-state IMU propagation. The current additive
  UKF works but an error-state formulation is the principled way to
  handle attitude observations from sonar and to add delayed-state
  smoothing once visual or DVL aiding lands.
- `aqua_fusion` per-platform benchmark runner (matching the existing
  `bench_fjord_1.sh` pattern).
- Magnetometer-based yaw observation (NTNU and AQUALOC both publish
  raw magnetometers but we currently only use the AHRS yaw-rate hook).
- Acoustic positioning inputs (USBL / SBL) — currently no public
  dataset shipping these is on our shortlist, but the UKF measurement
  framework is ready.
- Additional dataset adapters: OpenSonarDatasets, a second AQUALOC
  sequence, possibly UDualCam-Bottom or marine BlueROV2 logs.

## Suggested First Tasks For The Next Iteration

In priority order:

1. **RViz + status-driven MBES loop-closure tuning.** Use
   `rviz/mbes_loop_closure.rviz`, `/mbes_loop_closure/status`, and
   `/mbes_loop_closure/markers` on MBES-SLAM `beach_pond`; export
   accepted/rejected/no-candidate counts and correction distributions to
   drive threshold choices instead of tuning by eye alone.
2. **Loop-closure frontend refactor.** Split `mbes_loop_closure_node.cpp`
   into submap management, candidate selection, registration, gate
   evaluation, status/marker publishing, and constraint publishing before
   adding descriptor or robust-constraint logic.
3. **`aqua_fusion` benchmark runner** matching the
   `bench_fjord_1.sh` pattern, plus a `docs/benchmarks/` table per
   public dataset.
4. **Visual loop closure on AQUALOC** (probably depends on adopting a
   DBoW-like descriptor library — evaluate before committing).
5. **ESKF backend** as a separate node behind a `localization.backend`
   parameter so additive UKF and error-state run in parallel during
   the transition.

## Files To Read First

If you are picking this project up cold, in this order:

- `README.md` — top-level positioning, package list, public-data demo
  matrix, web replay paths.
- `docs/mvp_checklist.md` — the canonical "Done" / "Next Milestones"
  list, kept in sync with each PR.
- `docs/foxglove/README.md` — the recording-and-replay workflow that
  underpins every demo asset.
- `aqua_pose_graph/include/aqua_pose_graph/pose_graph.hpp` and the
  matching `src/pose_graph.cpp` — short, self-contained, the cleanest
  example of what shipping into this repo looks like.
- `aqua_imu_loc/src/additive_ukf.cpp` — the largest single source file,
  the place where DVL / sonar / pressure measurement updates live.

## Workspace Note

The repository sits at
`/media/sasaki/aiueo/ai_coding_ws/aqua_loc_ws/aqua_localization`. The
colcon workspace root is one directory up. Standard ROS 2 Jazzy
sourcing applies; no project-specific shell wrappers are required for
this iteration.

PR workflow is mandatory — direct pushes to `main` are rejected by
GitHub branch protection. Feature branches are squash-merged via
`gh pr merge --squash --delete-branch`.
