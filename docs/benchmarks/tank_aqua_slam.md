# Tank Dataset AQUA-SLAM Head-to-Head

This file is the comparison table target for `aqua_localization` vs
[AQUA-SLAM](https://github.com/SenseRoboticsLab/AQUA-SLAM) on Tank Dataset
sequences.

An initial AQUA-SLAM trajectory has been recorded for `short_test.bag` with the
upstream Docker workflow and converted into the same TUM trajectory format used
by this repository. The current result is a first reproducibility anchor, not a
paper claim, because the two systems use different sensor frontends and
AQUA-SLAM only publishes after its visual-inertial initialization.

Upstream AQUA-SLAM publishes its main estimate on `/AQUA_SLAM/orb_odom`
(`nav_msgs/Odometry`). Record that ROS 1 topic with `rostopic echo -p`, then
convert it with `ros1_odometry_csv_to_tum.py`.

## Current Anchor Result

`aqua_localization` already has a public Tank Dataset anchor on `short_test`
using IMU + pressure + DVL and AprilTag SLAM ground truth:

| Dataset | Sequence | System | Inputs | Alignment | RMSE m | Source |
|---------|----------|--------|--------|-----------|-------:|--------|
| Tank Dataset | `short_test` | `aqua_localization` | IMU + pressure + DVL | SE(3) | 0.429 | [`datasets/tank_dataset_demo.md`](../../datasets/tank_dataset_demo.md) |

The first AQUA-SLAM measurement below uses `short_test.bag` and the upstream
`underwater_orbslam3_blue_gx5_short.yaml` configuration.

## Initial AQUA-SLAM Measurement

Recorded on 2026-05-20:

| Dataset | Sequence | System | Inputs | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | `short_test` | AQUA-SLAM | stereo + IMU + DVL | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | Docker image `orb_dvl2_ros_noetic`, `/AQUA_SLAM/orb_odom` |

Raw-frame diagnostic for the same output, before SE(3) alignment:

| Dataset | Sequence | System | Alignment | Samples | RMSE m | Note |
|---------|----------|--------|-----------|--------:|-------:|------|
| Tank Dataset | `short_test` | AQUA-SLAM | none | 234 | 3.5186 | Confirms the odometry and AprilTag GT frames must be aligned before APE reporting. |

This short clip is a useful sanity check because AQUA-SLAM initializes, creates
keyframes, and publishes odometry, but it is only 11.65 matched seconds.

## Current `aqua_localization` Measurement

Recorded on 2026-05-20 with ROS 2 Humble, the converted `short_test` bag, and
the Tank Dataset IMU + pressure + DVL profile:

| Dataset | Sequence | System | Inputs | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | `short_test` | `aqua_localization` | IMU + pressure + DVL | SE(3) | 5399 | 14.94 | 0.3796 | 0.4014 | 0.4291 | 0.7652 | same AprilTag GT export |

This is a fair current-stack result for the lighter ROS 2 localization mode,
but it is not sensor-equivalent to AQUA-SLAM's stereo + IMU + DVL frontend.
AQUA-SLAM is the accuracy target on this visual-DVL-IMU sequence; the immediate
`aqua_localization` win path is either validating and fusing the experimental
`stereo_visual_odometry.py` frontend on Tank or moving the claim to ROS 2
reproducibility, permissive licensing, and MBES/sonar datasets where this
repository has stronger tooling.

## Experimental Visual Frontend Measurement

Recorded on 2026-05-20 with the new `stereo_visual_odometry.py` frontend on the
camera-included `short_test` conversion:

| Dataset | Sequence | System | Inputs | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | `short_test` | `aqua_visual_frontend` | stereo only | SE(3) | 200 | 11.35 | 1.2667 | 1.2280 | 1.3649 | 2.2035 | nominal stereo scale |
| Tank Dataset | `short_test` | `aqua_visual_frontend` | stereo only | Sim(3) | 200 | 11.35 | 0.0826 | 0.0786 | 0.0958 | 0.2458 | scale diagnostic, not paper-safe |
| Tank Dataset | `short_test` | `aqua_visual_frontend` | stereo only | SE(3) | 200 | 11.25 | 0.0815 | 0.0792 | 0.0947 | 0.2416 | `tracking.translation_scale=0.169623465`, same-sequence scale fit |
| Tank Dataset | `short_test` | `aqua_localization+visual` | IMU + pressure + DVL + stereo | SE(3) | 5399 | 14.94 | 0.3384 | 0.2928 | 0.3726 | 0.7497 | visual position update, same-sequence scale fit, variance floor 0.0025 |

The frontend is already better than the IMU + pressure + DVL row once metric
scale is calibrated, but it still does not beat AQUA-SLAM's 0.0194 m RMSE and
the fused result only improves the current stack from 0.4291 m to 0.3726 m.
The next serious step is out-of-sequence scale calibration plus a calibrated
camera-to-base transform before claiming a visual-inertial-DVL result.

Generate the diagnostic visual scale with:

```bash
ros2 run aqua_localization calibrate_visual_scale.py \
  /tmp/tank_short_test_gt.tum \
  /tmp/tank_short_test_visual_frontend.tum \
  --ros-args
```

For paper-safe reporting, the calibration TUM and validation TUM must come from
different sequences.

Once full Tank Dataset access is available, use `validate_visual_scale.py` to
separate calibration and held-out validation:

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

For single-sequence visual frontend experiments, `run_tank_visual_benchmark.py`
can replay a camera bag, record `/aqua_visual_frontend/odometry`, emit the scale
diagnostic, and save a Markdown benchmark row:

```bash
ros2 run aqua_localization run_tank_visual_benchmark.py \
  --bag /path/to/tank_sequence_ros2_with_cameras \
  --reference /tmp/tank_sequence_gt.tum \
  --out-dir /tmp/aqua_tank_visual_sequence \
  --sequence tank_sequence
```

When a visual TUM file has already been recorded, pass `--estimate` instead of
`--bag` to regenerate the scale report and benchmark row without replaying ROS.
The bag replay mode also saves `*_visual_frontend_status.csv`, which contains
per-frame feature counts, stereo match counts, triangulated point counts,
temporal match counts, PnP inliers, inlier ratio, accepted/rejected state, and
the reject reason. Use that CSV to decide whether the next tuning pass should
focus on image features, stereo geometry, temporal matching, or PnP gates.
It also emits `*_visual_frontend_status.md` via
`summarize_visual_frontend_status.py`, so each visual benchmark run carries a
short tuning report next to the trajectory metrics.

The public Tank Dataset page currently exposes `short_test` as sample data and
requires the download form for the full sequence set, so this table keeps
Structure_Easy and Medium rows as targets until those bags are available.

## Head-to-Head Table

Rows are generated with `trajectory_benchmark_row.py` so both systems use the
same APE implementation.

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | AQUA-SLAM Docker, short_test, /AQUA_SLAM/orb_odom |
| Tank Dataset | short_test | aqua_localization | SE(3) | 5399 | 14.94 | 0.3796 | 0.4014 | 0.4291 | 0.7652 | ROS 2 Humble, IMU+pressure+DVL, same AprilTag GT export |
| Tank Dataset | short_test | aqua_visual_frontend | SE(3) | 200 | 11.25 | 0.0815 | 0.0792 | 0.0947 | 0.2416 | stereo ORB+PnP, same-sequence scale fit from calibrate_visual_scale.py |
| Tank Dataset | short_test | aqua_localization+visual | SE(3) | 5399 | 14.94 | 0.3384 | 0.2928 | 0.3726 | 0.7497 | visual position update, same-sequence scale fit |
| Tank Dataset | Medium | aqua_visual_frontend | TBD | TBD | TBD | TBD | TBD | TBD | TBD | held-out validation after scale calibration on Structure_Easy |
| Tank Dataset | Structure_Easy | AQUA-SLAM | TBD | TBD | TBD | TBD | TBD | TBD | TBD | record AQUA-SLAM output topic to TUM |
| Tank Dataset | Structure_Easy | aqua_localization | TBD | TBD | TBD | TBD | TBD | TBD | TBD | run closest available input mode |

## Generate Rows

Use the same reference TUM for both systems:

```bash
# In the AQUA-SLAM ROS 1 Docker container while the Tank bag is playing.
rostopic echo -p /AQUA_SLAM/orb_odom > /tmp/aqua_slam_orb_odom.csv

# In this ROS 2 workspace after making the CSV visible on the host.
ros2 run aqua_localization ros1_odometry_csv_to_tum.py \
  --csv /tmp/aqua_slam_orb_odom.csv \
  --out /tmp/tank_short_test_aqua_slam.tum \
  --time-unit auto
```

```bash
ros2 run aqua_localization trajectory_benchmark_row.py \
  --reference /tmp/tank_short_test_gt.tum \
  --estimate /tmp/tank_short_test_aqua_slam.tum \
  --dataset "Tank Dataset" \
  --sequence short_test \
  --system AQUA-SLAM \
  --note "AQUA-SLAM Docker, short_test, /AQUA_SLAM/orb_odom" \
  --header
```

```bash
ros2 run aqua_localization trajectory_benchmark_row.py \
  --reference /tmp/tank_short_test_gt.tum \
  --estimate /tmp/tank_short_test_aqua_localization.tum \
  --dataset "Tank Dataset" \
  --sequence short_test \
  --system aqua_localization \
  --note "ROS 2, same short_test reference"
```

Use rigid SE(3) alignment by default. Use `--scale` only if the paper section
explicitly reports Sim(3), and use `--no-align` only for raw-frame diagnostics.

## What Counts as Winning

A paper-safe win should include at least one of:

- lower APE RMSE on the same sequence and same reference trajectory
- similar APE with easier ROS 2 reproduction and fewer manual calibration edits
- better stability across repeated runs, especially if the AQUA-SLAM known
  long-sequence crash appears
- better permissive reuse story because `aqua_localization` is Apache-2.0
- better MBES/sonar replay diagnostics on non-Tank datasets

A claim that does **not** count yet:

- "beats AQUA-SLAM" based only on README features
- comparing `short_test` against AQUA-SLAM's `Structure_Easy`
- denying AQUA-SLAM stereo images while claiming full SLAM superiority
