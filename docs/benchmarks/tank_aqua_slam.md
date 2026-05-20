# Tank Dataset AQUA-SLAM Head-to-Head

This file is the comparison table target for `aqua_localization` vs
[AQUA-SLAM](https://github.com/SenseRoboticsLab/AQUA-SLAM) on Tank Dataset
sequences.

No AQUA-SLAM trajectory has been recorded into this repository yet. The first
goal is to make both systems write TUM trajectories for the same sequence and
then use the same comparison command.

## Current Anchor Result

`aqua_localization` already has a public Tank Dataset anchor on `short_test`
using IMU + pressure + DVL and AprilTag SLAM ground truth:

| Dataset | Sequence | System | Inputs | Alignment | RMSE m | Source |
|---------|----------|--------|--------|-----------|-------:|--------|
| Tank Dataset | `short_test` | `aqua_localization` | IMU + pressure + DVL | SE(3) | 0.426 | [`datasets/tank_dataset_demo.md`](../../datasets/tank_dataset_demo.md) |

This is not a direct AQUA-SLAM win/loss row because AQUA-SLAM's README starts
from `Structure_Easy.bag`, not `short_test.bag`.

## Head-to-Head Table

Populate this table with `trajectory_benchmark_row.py` after both estimates are
available for the same sequence.

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | Structure_Easy | AQUA-SLAM | TBD | TBD | TBD | TBD | TBD | TBD | TBD | record AQUA-SLAM output topic to TUM |
| Tank Dataset | Structure_Easy | aqua_localization | TBD | TBD | TBD | TBD | TBD | TBD | TBD | run closest available input mode |

## Generate Rows

Use the same reference TUM for both systems:

```bash
ros2 run aqua_localization trajectory_benchmark_row.py \
  --reference /tmp/tank_structure_easy_gt.tum \
  --estimate /tmp/tank_structure_easy_aqua_slam.tum \
  --dataset "Tank Dataset" \
  --sequence Structure_Easy \
  --system AQUA-SLAM \
  --note "AQUA-SLAM Docker, documented launch" \
  --header
```

```bash
ros2 run aqua_localization trajectory_benchmark_row.py \
  --reference /tmp/tank_structure_easy_gt.tum \
  --estimate /tmp/tank_structure_easy_aqua_localization.tum \
  --dataset "Tank Dataset" \
  --sequence Structure_Easy \
  --system aqua_localization \
  --note "ROS 2, documented launch"
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
