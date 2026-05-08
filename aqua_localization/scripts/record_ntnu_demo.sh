#!/usr/bin/env bash
# NTNU subset-fjord/fjord_1 demo bag recorder.
#
# Records a results-included `.mcap` for rerun.io / Lichtblick replay.
# The dataset's own SLAM baseline trajectory is the separate
# fjord_1_baseline.tum file (passed to rerun_export_ntnu.py via
# --baseline-tum); this script only records inputs + aqua_imu_loc output.
#
# Usage:
#   ./record_ntnu_demo.sh
#   NTNU_DURATION=120 ./record_ntnu_demo.sh

set -euo pipefail

WORKSPACE="${WORKSPACE:-/media/sasaki/aiueo/ai_coding_ws/aqua_loc_ws}"
NTNU_SRC="${NTNU_SRC:-$WORKSPACE/aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_ros2}"
NTNU_OUT="${NTNU_OUT:-$WORKSPACE/aqua_localization/datasets/public/ntnu/demo_with_estimate}"
PROFILE="${PROFILE:-$WORKSPACE/install/aqua_imu_loc/share/aqua_imu_loc/config/ntnu_fjord.yaml}"
NTNU_DURATION="${NTNU_DURATION:-90}"

cd "$WORKSPACE"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

rm -rf "$NTNU_OUT"

ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file "$PROFILE" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_ntnu_imu.log 2>&1 &
IMU_PID=$!

sleep 2

ros2 bag record -s mcap -o "$NTNU_OUT" \
  --topics /mavros/imu/data /mavros/imu/static_pressure /mavros/rangefinder/rangefinder \
           /aqua_imu_loc/odometry /aqua_imu_loc/status \
           /tf /tf_static \
  > /tmp/aqua_record_ntnu_bag.log 2>&1 &
REC_PID=$!

sleep 2

ros2 bag play "$NTNU_SRC" --clock --playback-duration "$NTNU_DURATION" \
  > /tmp/aqua_record_ntnu_play.log 2>&1

sleep 3

kill -INT "$REC_PID" "$IMU_PID" 2>/dev/null || true
sleep 4
kill -TERM "$REC_PID" "$IMU_PID" 2>/dev/null || true
sleep 1

ls -la "$NTNU_OUT"
echo "NTNU demo bag recorded to $NTNU_OUT"
