# Tank Dataset AQUA-SLAM Head-to-Head

This file is the comparison table target for `aqua_localization` vs
[AQUA-SLAM](https://github.com/SenseRoboticsLab/AQUA-SLAM) on Tank Dataset
sequences.

An initial AQUA-SLAM trajectory has been recorded for `short_test.bag` with the
upstream Docker workflow and converted into the same TUM trajectory format used
by this repository. The current result is a first reproducibility anchor, not a
paper claim, because the matching `aqua_localization` run still needs to be
re-recorded under the exact same benchmark command.

Upstream AQUA-SLAM publishes its main estimate on `/AQUA_SLAM/orb_odom`
(`nav_msgs/Odometry`). Record that ROS 1 topic with `rostopic echo -p`, then
convert it with `ros1_odometry_csv_to_tum.py`.

## Current Anchor Result

`aqua_localization` already has a public Tank Dataset anchor on `short_test`
using IMU + pressure + DVL and AprilTag SLAM ground truth:

| Dataset | Sequence | System | Inputs | Alignment | RMSE m | Source |
|---------|----------|--------|--------|-----------|-------:|--------|
| Tank Dataset | `short_test` | `aqua_localization` | IMU + pressure + DVL | SE(3) | 0.426 | [`datasets/tank_dataset_demo.md`](../../datasets/tank_dataset_demo.md) |

The first AQUA-SLAM measurement below uses `short_test.bag` and the upstream
`underwater_orbslam3_blue_gx5_short.yaml` configuration. It should be compared
against a freshly recorded `aqua_localization` row before making a win/loss
claim.

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
keyframes, and publishes odometry, but it is only 11.65 matched seconds. The
next benchmark step is to rerun `aqua_localization` on the same converted
`short_test` bag and then move both systems to longer Tank sequences.

## Head-to-Head Table

Populate this table with `trajectory_benchmark_row.py` after both estimates are
available for the same sequence.

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | AQUA-SLAM Docker, short_test, /AQUA_SLAM/orb_odom |
| Tank Dataset | short_test | aqua_localization | SE(3) | TBD | TBD | TBD | TBD | TBD | TBD | rerun same sequence with current stack |
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
