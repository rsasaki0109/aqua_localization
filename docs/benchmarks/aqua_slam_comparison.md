# AQUA-SLAM Comparison Plan

This is the focused benchmark plan for comparing `aqua_localization` against
[SenseRoboticsLab/AQUA-SLAM](https://github.com/SenseRoboticsLab/AQUA-SLAM).
AQUA-SLAM is the closest current open-source underwater SLAM baseline for the
Tank Dataset, so it should be treated as a serious competitor, not a generic
checkbox baseline.

Checked on 2026-05-20:

- Repository: <https://github.com/SenseRoboticsLab/AQUA-SLAM>
- License file: GPL-3.0
- GitHub metadata from `gh repo view`: 131 stars, 14 forks
- Publication listed in README: "AQUA-SLAM: Tightly-Coupled Underwater
  Acoustic-Visual-Inertial SLAM with Sensor Calibration," IEEE Transactions on
  Robotics, 2025

## What AQUA-SLAM Does

AQUA-SLAM is an underwater SLAM system that integrates:

- DVL
- IMU
- stereo cameras
- loop detection
- sensor calibration/extrinsic parameters
- RViz visualization

Its README points users to the Tank Dataset, includes per-sequence launch files
such as `blue_gx5_StructureEasy.launch`, and notes that different sequences may
need different extrinsic calibration files. It also documents a known issue:
long sequences may randomly crash due to multithreading problems.

Upstream source inspection shows the primary odometry output topic is
`/AQUA_SLAM/orb_odom` (`nav_msgs/Odometry`), published from
`src/RosHandling.cpp`.

An initial Docker run on `short_test.bag` was completed on 2026-05-20. AQUA-SLAM
initialized, created keyframes, published 234 odometry samples, and reached
0.0194 m APE RMSE after rigid SE(3) alignment against `/apriltag_slam/GT`. The
same output has 3.5186 m raw-frame RMSE before alignment, so benchmark tables
must state the alignment mode explicitly.

## Direct Comparison

| Axis | AQUA-SLAM | `aqua_localization` | Current read |
|------|-----------|---------------------|--------------|
| Primary goal | Full underwater acoustic-visual-inertial SLAM | ROS 2 underwater localization, sonar registration, pose graph, replay tooling | AQUA-SLAM is stronger for full stereo+DVL+IMU SLAM today. |
| ROS generation | ROS 1 / catkin workflow in Docker | ROS 2 Humble/Jazzy colcon packages | `aqua_localization` is stronger for ROS 2 adoption. |
| Main sensors | DVL, IMU, stereo cameras | IMU, pressure, DVL, sonar point clouds / MBES | Different sensor emphasis; Tank Dataset is the common overlap. |
| Dataset story | Tank Dataset focused | Tank, MBES-SLAM, NTNU, AQUALOC | `aqua_localization` is broader across public datasets. |
| License | GPL-3.0 repository license | Apache-2.0 | `aqua_localization` is easier for permissive downstream reuse. |
| Visual SLAM | Core capability | Experimental stereo ORB + PnP odometry frontend, no loop closure or visual-inertial coupling yet | AQUA-SLAM still wins here until visual odometry is fused and benchmarked. |
| MBES bathymetry | Not the core public story | MBES replay, registration, pose graph, experimental loop closure | `aqua_localization` has the stronger MBES-specific path. |
| Reproducibility surface | Docker and per-sequence launch files | ROS 2 launches, dataset docs, rerun exports, benchmark scripts | Needs measured setup-time comparison. |
| Known limitations | README mentions random long-sequence crash risk | README documents drift and uncalibrated MBES loop closure | Both are honest; compare stability during replay. |

## Initial Measured Result

The first measured head-to-head row is tracked in
[`tank_aqua_slam.md`](tank_aqua_slam.md). Current status:

| Dataset | Sequence | System | Alignment | Samples | Matched s | RMSE m | Status |
|---------|----------|--------|-----------|--------:|----------:|-------:|--------|
| Tank Dataset | `short_test` | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0194 | measured |
| Tank Dataset | `short_test` | `aqua_localization` | SE(3) | 5399 | 14.94 | 0.4291 | measured, lighter sensor stack |
| Tank Dataset | `short_test` | `aqua_visual_frontend` | SE(3) | 200 | 11.25 | 0.0947 | measured with same-sequence scale fit |
| Tank Dataset | `short_test` | `aqua_localization+visual` | SE(3) | 5399 | 14.94 | 0.3726 | fused visual position update, same-sequence scale fit |
| Tank Dataset | `short_test` | `aqua_localization+visual` | SE(3) | 5424 | 14.95 | 0.3228 | base-frame visual odometry, same-sequence scale/extrinsic diagnostics, replay rate 0.25, visual coverage 300/300 |

This result makes AQUA-SLAM the accuracy target to beat on Tank visual-DVL-IMU
sequences. The fair development path is to either add a visual frontend for a
sensor-equivalent Tank comparison or scope the paper claim toward permissive
ROS 2 localization and MBES-specific replay strengths.

## Fair Head-to-Head Tasks

### 1. Tank Dataset Full SLAM

This is the most important comparison.

Use the same Tank Dataset sequence, ideally `Structure_Easy.bag` first because
AQUA-SLAM documents that path.

Measure:

- trajectory APE RMSE / mean / median
- depth RMSE if reference depth is available
- tracking loss / invalid output periods
- runtime factor
- setup steps and required manual calibration edits
- crash/restart count over repeated runs

Expected current outcome:

- AQUA-SLAM may win trajectory accuracy because it uses stereo visual features
  and tightly-coupled acoustic-visual-inertial optimization.
- `aqua_localization` can still win ROS 2 portability, permissive licensing,
  simpler export tooling, and pressure/DVL fusion transparency.

Paper-safe claim before running the benchmark:

> AQUA-SLAM is the full underwater SLAM baseline on Tank Dataset. The first
> `aqua_localization` benchmark should report whether its ROS 2 localization
> stack is competitive as a lighter, more modular alternative, not claim
> superiority over AQUA-SLAM's full visual-inertial-acoustic system.

### 2. Tank Dataset Sensor-Ablation Study

If the Tank Dataset topics allow it, run:

| Mode | AQUA-SLAM | `aqua_localization` |
|------|-----------|---------------------|
| IMU only | not the main target | supported through `aqua_imu_loc` |
| IMU + pressure | not the main target | core path |
| IMU + pressure + DVL | partial overlap | core path |
| IMU + DVL + stereo | core path | missing tightly-coupled stereo frontend |
| IMU + DVL + sonar/MBES | not the Tank Dataset core | MBES path on MBES-SLAM, not Tank |

This avoids an unfair comparison where one system is denied its main sensor.

### 3. MBES-SLAM `beach_pond`

This is not a fair AQUA-SLAM head-to-head unless AQUA-SLAM can be configured for
the available camera/DVL/IMU topics and reference trajectory. Treat it as a
separate `aqua_localization` strength: MBES replay, sonar registration, and
loop-closure diagnostics.

## Implementation Plan

1. **Add an AQUA-SLAM runner note.**
   Document the exact Docker, vocabulary download, Tank Dataset placement, and
   `rosbag play` commands from the AQUA-SLAM README.
2. **Identify AQUA-SLAM output odometry topic.**
   Use `/AQUA_SLAM/orb_odom` first. Confirm with `rostopic hz` during replay.
3. **Write `record_aqua_slam_tank_baseline.md`.**
   Keep this as documentation first unless a reproducible ROS 1 container can
   be automated cleanly from this repository.
4. **Convert AQUA-SLAM output to TUM.**
   Reuse the same TUM format consumed by `compare_trajectories.py`, then create
   rows with `trajectory_benchmark_row.py`.
5. **Run `aqua_localization` on the same Tank sequence.**
   Use the closest available input mode and clearly state which sensors are
   enabled.
6. **Generate a comparison table.**
   Start with [`tank_aqua_slam.md`](tank_aqua_slam.md), then include accuracy,
   runtime, setup steps, license, ROS generation, and failure modes.

## AQUA-SLAM TUM Export

Inside the AQUA-SLAM ROS 1 Docker container, after launching an AQUA-SLAM
config and starting `rosbag play`, record the odometry CSV:

```bash
rostopic echo -p /AQUA_SLAM/orb_odom > /tmp/aqua_slam_orb_odom.csv
```

Then convert that CSV with this repository's ROS 2-side helper:

```bash
ros2 run aqua_localization ros1_odometry_csv_to_tum.py \
  --csv /tmp/aqua_slam_orb_odom.csv \
  --out /tmp/tank_short_test_aqua_slam.tum \
  --time-unit auto
```

The output can be passed directly to `compare_trajectories.py` or
`trajectory_benchmark_row.py`.

## Decision: Where Can We Beat It?

Likely current wins:

- ROS 2 Humble/Jazzy support
- Apache-2.0 licensing
- broader public dataset coverage
- pressure/DVL/sonar replay tooling
- MBES-specific visualization and loop-status diagnostics
- README/demo polish and reproducible exports

Likely current losses:

- full stereo+DVL+IMU underwater SLAM accuracy on Tank Dataset
- published T-RO-level validation
- tightly-coupled online calibration
- mature visual loop closure

High-value development to close the gap:

1. Tank Dataset `Structure_Easy` benchmark harness for `aqua_localization`.
2. AQUA-SLAM output recording and TUM conversion instructions.
3. Fair comparison table in `docs/benchmarks/tank_aqua_slam.md`.
4. Calibrate `stereo_visual_odometry.py` scale on one Tank sequence with
   `calibrate_visual_scale.py`, then validate the visual-aided fusion on a
   held-out Tank sequence with `validate_visual_scale.py`.
5. MBES paper path separated from stereo visual SLAM claims.

## Source Notes

- AQUA-SLAM README: <https://github.com/SenseRoboticsLab/AQUA-SLAM>
- AQUA-SLAM paper link from README:
  <https://arxiv.org/pdf/2503.11420>
- Tank Dataset link from README:
  <https://senseroboticslab.github.io/underwater-tank-dataset/>
