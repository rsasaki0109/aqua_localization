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
