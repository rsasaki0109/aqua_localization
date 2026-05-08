#!/usr/bin/env bash
# Tank Dataset short_test demo bag recorder.
#
# Records a results-included `.mcap` that bundles the source sensors with
# `aqua_imu_loc` output topics, ready for rerun.io / Lichtblick replay.
#
# Pre-requisites:
#   - colcon workspace built (this script sources `install/setup.bash`)
#   - source bag at $TANK_SRC (see TANK_SRC default below)
#
# Usage:
#   ./record_tank_demo.sh
#   TANK_OUT=/tmp/my_run ./record_tank_demo.sh
#
# After it finishes:
#   ./aqua_localization/scripts/rerun_export.py \
#     --bag "$TANK_OUT" --out docs/media/tank_dataset.rrd

set -euo pipefail

WORKSPACE="${WORKSPACE:-/media/sasaki/aiueo/ai_coding_ws/aqua_loc_ws}"
TANK_SRC="${TANK_SRC:-$WORKSPACE/aqua_localization/datasets/public/tank_dataset/short_test_ros2}"
TANK_OUT="${TANK_OUT:-$WORKSPACE/aqua_localization/datasets/public/tank_dataset/demo_with_estimate}"
PROFILE="${PROFILE:-$WORKSPACE/install/aqua_imu_loc/share/aqua_imu_loc/config/tank_dataset.yaml}"

cd "$WORKSPACE"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

rm -rf "$TANK_OUT"

ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file "$PROFILE" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_tank_imu.log 2>&1 &
IMU_PID=$!

sleep 2

ros2 bag record -s mcap -o "$TANK_OUT" \
  --topics /imu/data /pressure /dvl/twist /apriltag_slam/GT \
           /aqua_imu_loc/odometry /aqua_imu_loc/status \
           /tf /tf_static \
  > /tmp/aqua_record_tank_bag.log 2>&1 &
REC_PID=$!

sleep 2

ros2 bag play "$TANK_SRC" --clock > /tmp/aqua_record_tank_play.log 2>&1

sleep 2

kill -INT "$REC_PID" "$IMU_PID" 2>/dev/null || true
sleep 3
kill -TERM "$REC_PID" "$IMU_PID" 2>/dev/null || true
sleep 1

ls -la "$TANK_OUT"
echo "Tank demo bag recorded to $TANK_OUT"
