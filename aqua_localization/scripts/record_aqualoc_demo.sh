#!/usr/bin/env bash
# AQUALOC harbor_07 demo bag recorder.
#
# Records a results-included `.mcap` for rerun.io / Lichtblick replay,
# including the underwater camera feed. Capped to 60 s by default since
# the source bag is ~720 MB across 113 s of camera frames.
#
# Usage:
#   ./record_aqualoc_demo.sh
#   AQUALOC_DURATION=120 ./record_aqualoc_demo.sh

set -euo pipefail

WORKSPACE="${WORKSPACE:-/media/sasaki/aiueo/ai_coding_ws/aqua_loc_ws}"
AQUALOC_SRC="${AQUALOC_SRC:-$WORKSPACE/aqua_localization/datasets/public/aqualoc/harbor_sequence_07_ros2}"
AQUALOC_OUT="${AQUALOC_OUT:-$WORKSPACE/aqua_localization/datasets/public/aqualoc/demo_with_estimate}"
PROFILE="${PROFILE:-$WORKSPACE/install/aqua_imu_loc/share/aqua_imu_loc/config/aqualoc.yaml}"
AQUALOC_DURATION="${AQUALOC_DURATION:-60}"

cd "$WORKSPACE"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

rm -rf "$AQUALOC_OUT"

ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file "$PROFILE" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_aqualoc_imu.log 2>&1 &
IMU_PID=$!

sleep 2

ros2 bag record -s mcap -o "$AQUALOC_OUT" \
  --topics /rtimulib_node/imu /rtimulib_node/mag \
           /camera/image_raw /camera/camera_info \
           /barometer_node/pressure /barometer_node/depth \
           /aqua_imu_loc/odometry /aqua_imu_loc/status \
           /tf /tf_static \
  > /tmp/aqua_record_aqualoc_bag.log 2>&1 &
REC_PID=$!

sleep 2

ros2 bag play "$AQUALOC_SRC" --clock --playback-duration "$AQUALOC_DURATION" \
  > /tmp/aqua_record_aqualoc_play.log 2>&1

sleep 3

kill -INT "$REC_PID" "$IMU_PID" 2>/dev/null || true
sleep 4
kill -TERM "$REC_PID" "$IMU_PID" 2>/dev/null || true
sleep 1

ls -la "$AQUALOC_OUT"
echo "AQUALOC demo bag recorded to $AQUALOC_OUT"
