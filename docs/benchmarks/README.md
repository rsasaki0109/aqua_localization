# Benchmarks

This directory stores reproducible benchmark notes for public underwater
datasets and baseline comparisons.

## Current Results

- [`fjord_1.md`](fjord_1.md): NTNU `subset-fjord/fjord_1` APE history for
  `/aqua_imu_loc/odometry` against the dataset baseline trajectory.
- [`fjord_1_yaw_frame.md`](fjord_1_yaw_frame.md): yaw-frame diagnosis for the
  same NTNU sequence.
- [`tank_aqua_slam.md`](tank_aqua_slam.md): first AQUA-SLAM Docker measurement
  on Tank Dataset `short_test`, current `aqua_localization` rows, and the RMSE
  gap to the baseline.
- [`mbes_beach_pond_loop_status.md`](mbes_beach_pond_loop_status.md):
  first MBES-SLAM `beach_pond` tuning measurement with loop-status counts,
  descriptor sweep output, and false-positive audit notes.

## Real-Bag Evaluation Manifest

[`real_bag_evaluation_manifest.json`](real_bag_evaluation_manifest.json) pins
the dataset cases that should become repeatable benchmark evidence before any
"beats another OSS" claim is made. The rendered run sheet lives at
[`real_bag_evaluation.md`](real_bag_evaluation.md). Regenerate it with:

```bash
ros2 run aqua_localization benchmark_manifest_report.py \
  docs/benchmarks/real_bag_evaluation_manifest.json \
  --out /tmp/aqua_real_bag_evaluation.md \
  --check-ready \
  --check-doc-artifacts
```

Use `--status measured`, `--status scaffolded`, or `--status planned` to focus
the report before a benchmark session.

## Comparison Planning

- [`oss_comparison.md`](oss_comparison.md): protocol for comparing
  `aqua_localization` against existing open-source localization and SLAM tools
  without overstating results.
- [`aqua_slam_comparison.md`](aqua_slam_comparison.md): focused comparison
  plan for SenseRoboticsLab/AQUA-SLAM, the closest current underwater SLAM
  baseline.
- [`tank_aqua_slam.md`](tank_aqua_slam.md): Tank Dataset head-to-head table and
  commands for AQUA-SLAM, including ROS 1 `/AQUA_SLAM/orb_odom` export into TUM
  format. Use `benchmark_gap_report.py` on this page to compute the current
  RMSE gap and the percentage improvement needed to tie the baseline.

The comparison plan is intentionally conservative. It separates sensor-fusion,
visual/LiDAR SLAM, underwater simulation, and MBES replay work so each baseline
is evaluated on a fair input set.
