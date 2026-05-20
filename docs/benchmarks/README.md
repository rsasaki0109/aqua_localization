# Benchmarks

This directory stores reproducible benchmark notes for public underwater
datasets and baseline comparisons.

## Current Results

- [`fjord_1.md`](fjord_1.md): NTNU `subset-fjord/fjord_1` APE history for
  `/aqua_imu_loc/odometry` against the dataset baseline trajectory.
- [`fjord_1_yaw_frame.md`](fjord_1_yaw_frame.md): yaw-frame diagnosis for the
  same NTNU sequence.

## Comparison Planning

- [`oss_comparison.md`](oss_comparison.md): protocol for comparing
  `aqua_localization` against existing open-source localization and SLAM tools
  without overstating results.

The comparison plan is intentionally conservative. It separates sensor-fusion,
visual/LiDAR SLAM, underwater simulation, and MBES replay work so each baseline
is evaluated on a fair input set.
