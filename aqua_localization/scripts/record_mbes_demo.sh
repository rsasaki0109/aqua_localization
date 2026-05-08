#!/usr/bin/env bash
# MBES-SLAM beach_pond demo bag recorder.
#
# Records a results-included `.mcap` that bundles the source multibeam fans,
# IMU, reference odometry, and `aqua_imu_loc` + `aqua_sonar_loc` outputs,
# ready for rerun.io / Lichtblick replay.
#
# Usage:
#   ./record_mbes_demo.sh
#   MBES_DURATION=120 ./record_mbes_demo.sh

set -euo pipefail

WORKSPACE="${WORKSPACE:-/media/sasaki/aiueo/ai_coding_ws/aqua_loc_ws}"
MBES_SRC="${MBES_SRC:-$WORKSPACE/aqua_localization/datasets/public/mbes_slam/beach_pond_ros2}"
MBES_OUT="${MBES_OUT:-$WORKSPACE/aqua_localization/datasets/public/mbes_slam/demo_with_estimate}"
IMU_PROFILE="${IMU_PROFILE:-$WORKSPACE/install/aqua_imu_loc/share/aqua_imu_loc/config/mbes_slam.yaml}"
SONAR_PROFILE="${SONAR_PROFILE:-$WORKSPACE/install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_slam.yaml}"
MBES_DURATION="${MBES_DURATION:-60}"

cd "$WORKSPACE"
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

rm -rf "$MBES_OUT"

ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file "$IMU_PROFILE" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_mbes_imu.log 2>&1 &
IMU_PID=$!

ros2 run aqua_sonar_loc sonar_loc_node --ros-args \
  --params-file "$SONAR_PROFILE" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_mbes_sonar.log 2>&1 &
SON_PID=$!

sleep 3

ros2 bag record -s mcap -o "$MBES_OUT" \
  --topics /norbit/detections /nav/processed/odometry \
           /nav/processed/microstrain/imu/madgwick /nav/sensors/microstrain/imu/raw \
           /aqua_imu_loc/odometry /aqua_imu_loc/status \
           /aqua_sonar_loc/odometry /aqua_sonar_loc/status \
           /aqua_sonar_loc/points_filtered \
           /tf /tf_static \
  > /tmp/aqua_record_mbes_bag.log 2>&1 &
REC_PID=$!

sleep 2

ros2 bag play "$MBES_SRC" --clock --playback-duration "$MBES_DURATION" \
  > /tmp/aqua_record_mbes_play.log 2>&1

sleep 3

kill -INT "$REC_PID" "$IMU_PID" "$SON_PID" 2>/dev/null || true
sleep 4
kill -TERM "$REC_PID" "$IMU_PID" "$SON_PID" 2>/dev/null || true
sleep 1

ls -la "$MBES_OUT"
echo "MBES demo bag recorded to $MBES_OUT"
