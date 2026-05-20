# Open-Source Comparison Plan

This document defines a conservative path for comparing `aqua_localization`
against existing open-source localization, SLAM, and underwater simulation
tools. The goal is to create publishable evidence without claiming victory on
tasks that are not equivalent.

## Candidate Baselines

| Project | What it is | Fair comparison role | Not a fair claim |
|---------|------------|----------------------|------------------|
| [`robot_localization`](https://docs.ros.org/en/jade/api/robot_localization/html/state_estimation_nodes.html) | ROS EKF/UKF state-estimation nodes for nonlinear mobile-robot sensor fusion. | Baseline for IMU, pressure-as-Z, DVL velocity, and odometry fusion on public bags. | Do not compare MBES loop closure or sonar registration directly; those are outside its scope. |
| [AQUA-SLAM](https://github.com/SenseRoboticsLab/AQUA-SLAM) | Underwater acoustic-visual-inertial SLAM using DVL, IMU, and stereo cameras. | Closest head-to-head underwater SLAM baseline on the Tank Dataset when both systems are given a documented input mode. | Do not claim current `aqua_localization` beats it on full visual-inertial-acoustic SLAM until a replayable Tank Dataset benchmark exists. |
| [`RTAB-Map`](https://docs.ros.org/en/rolling/p/rtabmap/index.html) | Open-source RGB-D / visual / LiDAR SLAM library and ROS package. | Baseline for camera or LiDAR-capable sequences, especially future AQUALOC visual loop-closure work. | Do not claim sonar-only superiority unless RTAB-Map is given a valid visual/LiDAR input baseline. |
| [HoloOcean](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/sensors/sensors.html) | Open-source underwater simulator with sensors including DVL, IMU, sonar, and depth. | Simulation reproducibility target for controlled sensor-ablation studies. | Do not treat simulator performance as real-ocean benchmark evidence. |
| [Stonefish](https://stonefish.readthedocs.io/) | Marine robotics simulator with underwater vehicle and sonar simulation support. | Secondary simulation target when testing launch portability and sensor assumptions. | Do not compare as a localization stack unless a concrete estimator baseline is configured. |
| Dataset-provided trajectories | Reference or baseline odometry included with public datasets. | Ground-truth or reference trajectory for APE/RPE comparison when available. | Do not call these OSS competitors unless the estimator code and configuration are available. |

## Win Conditions Worth Publishing

The strongest paper path is not "beats every OSS package." It is:

1. **Underwater-specific sensor fusion is easier to reproduce.**
   Show one-command replay/export paths for pressure, DVL, IMU, and sonar
   topics on multiple public bags.
2. **Real-bag accuracy improves over configured baselines.**
   Compare against `robot_localization` for generic fusion and AQUA-SLAM for
   Tank Dataset underwater SLAM on the same sequence, time window, and
   trajectory metric.
3. **MBES registration and loop-closure diagnostics are measurable.**
   Report accepted/rejected/no-candidate loop counts, descriptor gate behavior,
   registration fitness, correction magnitude, and false-positive review notes.
4. **The limitations are explicit.**
   Publish cases where generic visual/LiDAR SLAM is the right tool and cases
   where single-fan MBES geometry remains degenerate.

## Benchmark Matrix

| Dataset | Primary comparison | Metric | Baselines to add | Paper value |
|---------|--------------------|--------|------------------|-------------|
| Tank Dataset `short_test` | DVL + pressure + IMU fusion against AprilTag ground truth | Translation APE, depth RMSE, runtime factor | `robot_localization` EKF and UKF configs | Cleanest fusion benchmark because DVL and reference are present. |
| MBES-SLAM `beach_pond` | Sonar registration and fusion against dataset reference odometry | APE, sonar residual summaries, loop status counts | no-sonar fusion, dataset reference, later `robot_localization` fusion | Best visual README evidence and strongest MBES-specific story. |
| NTNU `subset-fjord/fjord_1` | IMU/pressure-only drift and yaw-frame behavior | APE, depth RMSE, yaw-drift diagnosis | `robot_localization` IMU-only config | Useful negative/control case showing why aiding is needed. |
| AQUALOC `harbor_07` | Pressure depth and future visual loop closure | Depth RMSE, visual odometry availability, APE if reference is usable | RTAB-Map visual baseline when camera calibration/replay is ready | Best future comparison against visual SLAM rather than sonar SLAM. |

## Minimum Fairness Rules

- Use the same bag segment, start time, and duration for every estimator.
- Use the same input topics for each comparison group.
- Report whether alignment is raw, rigid SE(3), or Sim(3).
- Keep per-dataset tuning separate from held-out evaluation segments when
  enough data exists.
- Record runtime factor and dropped/invalid output samples, not only APE.
- Publish the exact launch/config files used for every baseline.
- Mark any unavailable sensor path as "not applicable," not as a failure.

## Metrics

| Metric | Why |
|--------|-----|
| Translation APE RMSE/mean/median | Main trajectory error metric already supported by `compare_trajectories.py`. |
| Depth RMSE | Underwater-specific value; pressure should help even when XY drifts. |
| Relative pose error | Better for local drift once trajectory tools support windowed comparisons. |
| Runtime factor | Separates offline tuning from real-time feasibility. |
| Setup steps | Important OSS adoption signal: fewer manual conversions/config edits matter. |
| Loop status counts | MBES-specific evidence for accepted, rejected, and no-candidate loop attempts. |
| False-positive audit notes | Required before claiming reliable loop closure on real sonar data. |

## Concrete PR Sequence

1. **Add `robot_localization` baseline configs for Tank Dataset.**
   Map pressure to Z pose, DVL to twist, and IMU to angular velocity /
   orientation where valid. Export `/odometry/filtered` to TUM and compare
   against the AprilTag reference.
2. **Create a generic benchmark runner wrapper.**
   Extend the current NTNU-specific harness into a script that accepts bag,
   launch file, odometry topic, reference TUM, output directory, and note.
3. **Add a comparison table generator.**
   Convert benchmark markdown rows into a compact table for README and paper
   drafts.
4. **Add MBES loop-status benchmark artifacts.**
   Pair APE with descriptor sweep summaries, accepted/rejected counts, and a
   short false-positive review checklist.
5. **Add AQUA-SLAM head-to-head on Tank Dataset.**
   Follow [`aqua_slam_comparison.md`](aqua_slam_comparison.md), record the
   AQUA-SLAM output trajectory, and compare with `aqua_localization` on the
   same Tank Dataset sequence before making any paper claim.
6. **Add RTAB-Map only when visual inputs are ready.**
   Use AQUALOC for this; do not force RTAB-Map into sonar-only comparisons.

## Paper Claim Template

A defensible claim should look like this:

> On public underwater ROS bags with pressure, DVL, IMU, and sonar topics,
> `aqua_localization` provides reproducible dataset bring-up, underwater-specific
> fusion/registration tools, and benchmark exports. On the Tank Dataset
> `short_test` sequence, it should be compared against a configured
> `robot_localization` EKF/UKF baseline using the same inputs and APE/depth
> metrics. On MBES-SLAM `beach_pond`, sonar registration and loop-closure
> diagnostics should be evaluated separately from generic sensor-fusion
> baselines.

This is narrower than "beats existing OSS," but it is much easier to defend in
a paper review.

## References Checked

- AQUA-SLAM repository and README:
  <https://github.com/SenseRoboticsLab/AQUA-SLAM>
- AQUA-SLAM paper linked from the repository:
  <https://arxiv.org/pdf/2503.11420>
- `robot_localization` state-estimation nodes:
  <https://docs.ros.org/en/jade/api/robot_localization/html/state_estimation_nodes.html>
- `robot_localization` package overview:
  <https://index.ros.org/p/robot_localization/>
- RTAB-Map ROS package documentation:
  <https://docs.ros.org/en/rolling/p/rtabmap/index.html>
- HoloOcean sensor documentation:
  <https://byu-holoocean.github.io/holoocean-docs/v2.3.0/sensors/sensors.html>
- Stonefish documentation:
  <https://stonefish.readthedocs.io/>
