#!/usr/bin/env bash
# MBES-SLAM beach_pond demo bag recorder.
#
# Records a results-included `.mcap` that bundles the source multibeam fans,
# IMU, reference odometry, `aqua_imu_loc` + `aqua_sonar_loc` outputs, and the
# optional pose-graph / MBES loop-closure diagnostics ready for rerun.io /
# Lichtblick replay.
#
# Usage:
#   ./record_mbes_demo.sh
#   MBES_DURATION=120 ./record_mbes_demo.sh

set -euo pipefail

WORKSPACE="${WORKSPACE:-/media/sasaki/aiueo/ai_coding_ws/aqua_loc_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
MBES_SRC="${MBES_SRC:-$WORKSPACE/aqua_localization/datasets/public/mbes_slam/beach_pond_ros2}"
MBES_OUT="${MBES_OUT:-$WORKSPACE/aqua_localization/datasets/public/mbes_slam/demo_with_estimate}"
IMU_PROFILE="${IMU_PROFILE:-$WORKSPACE/install/aqua_imu_loc/share/aqua_imu_loc/config/mbes_slam.yaml}"
SONAR_PROFILE="${SONAR_PROFILE:-$WORKSPACE/install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_slam.yaml}"
POSE_GRAPH_PROFILE="${POSE_GRAPH_PROFILE:-$WORKSPACE/install/aqua_pose_graph/share/aqua_pose_graph/config/params.yaml}"
MBES_LOOP_PROFILE="${MBES_LOOP_PROFILE:-$WORKSPACE/install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_loop_closure.yaml}"
MBES_DURATION="${MBES_DURATION:-60}"
RECORD_STORAGE="${RECORD_STORAGE:-mcap}"
RECORD_TOPIC_FLAG="${RECORD_TOPIC_FLAG---topics}"
PLAY_DURATION_ARG="${PLAY_DURATION_ARG---playback-duration}"
POSE_GRAPH_KEYFRAME_TRANSLATION_M="${POSE_GRAPH_KEYFRAME_TRANSLATION_M:-}"
POSE_GRAPH_KEYFRAME_ROTATION_RAD="${POSE_GRAPH_KEYFRAME_ROTATION_RAD:-}"
MBES_LOOP_MIN_POINTS="${MBES_LOOP_MIN_POINTS:-}"
MBES_LOOP_VOXEL_LEAF_M="${MBES_LOOP_VOXEL_LEAF_M:-}"
MBES_LOOP_MIN_KEYFRAME_SEPARATION="${MBES_LOOP_MIN_KEYFRAME_SEPARATION:-}"
MBES_LOOP_MAX_DISTANCE_M="${MBES_LOOP_MAX_DISTANCE_M:-}"
MBES_LOOP_MAX_FITNESS_SCORE="${MBES_LOOP_MAX_FITNESS_SCORE:-}"
MBES_LOOP_MAX_CORRECTION_TRANSLATION_M="${MBES_LOOP_MAX_CORRECTION_TRANSLATION_M:-}"
MBES_LOOP_MAX_CORRECTION_ROTATION_RAD="${MBES_LOOP_MAX_CORRECTION_ROTATION_RAD:-}"
MBES_LOOP_DESCRIPTOR_MAX_CENTROID_DISTANCE_M="${MBES_LOOP_DESCRIPTOR_MAX_CENTROID_DISTANCE_M:-}"
MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO="${MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO:-}"
MBES_LOOP_DESCRIPTOR_MIN_POINT_COUNT_RATIO="${MBES_LOOP_DESCRIPTOR_MIN_POINT_COUNT_RATIO:-}"

POSE_GRAPH_PARAM_ARGS=()
if [[ -n "$POSE_GRAPH_KEYFRAME_TRANSLATION_M" ]]; then
  POSE_GRAPH_PARAM_ARGS+=("-p" "keyframe.translation_m:=$POSE_GRAPH_KEYFRAME_TRANSLATION_M")
fi
if [[ -n "$POSE_GRAPH_KEYFRAME_ROTATION_RAD" ]]; then
  POSE_GRAPH_PARAM_ARGS+=("-p" "keyframe.rotation_rad:=$POSE_GRAPH_KEYFRAME_ROTATION_RAD")
fi

MBES_LOOP_PARAM_ARGS=()
if [[ -n "$MBES_LOOP_MIN_POINTS" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "submaps.min_points:=$MBES_LOOP_MIN_POINTS")
fi
if [[ -n "$MBES_LOOP_VOXEL_LEAF_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "submaps.voxel_leaf_m:=$MBES_LOOP_VOXEL_LEAF_M")
fi
if [[ -n "$MBES_LOOP_MIN_KEYFRAME_SEPARATION" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "candidates.min_keyframe_separation:=$MBES_LOOP_MIN_KEYFRAME_SEPARATION")
fi
if [[ -n "$MBES_LOOP_MAX_DISTANCE_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "candidates.max_distance_m:=$MBES_LOOP_MAX_DISTANCE_M")
fi
if [[ -n "$MBES_LOOP_MAX_FITNESS_SCORE" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "gates.max_fitness_score:=$MBES_LOOP_MAX_FITNESS_SCORE")
fi
if [[ -n "$MBES_LOOP_MAX_CORRECTION_TRANSLATION_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "gates.max_correction_translation_m:=$MBES_LOOP_MAX_CORRECTION_TRANSLATION_M")
fi
if [[ -n "$MBES_LOOP_MAX_CORRECTION_ROTATION_RAD" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "gates.max_correction_rotation_rad:=$MBES_LOOP_MAX_CORRECTION_ROTATION_RAD")
fi
if [[ -n "$MBES_LOOP_DESCRIPTOR_MAX_CENTROID_DISTANCE_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "descriptor.max_centroid_distance_m:=$MBES_LOOP_DESCRIPTOR_MAX_CENTROID_DISTANCE_M")
fi
if [[ -n "$MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "descriptor.max_extent_ratio:=$MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO")
fi
if [[ -n "$MBES_LOOP_DESCRIPTOR_MIN_POINT_COUNT_RATIO" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "descriptor.min_point_count_ratio:=$MBES_LOOP_DESCRIPTOR_MIN_POINT_COUNT_RATIO")
fi

cd "$WORKSPACE"
# shellcheck disable=SC1091
set +u
source "$ROS_SETUP"
# shellcheck disable=SC1091
source install/setup.bash
set -u

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

ros2 run aqua_pose_graph pose_graph_node --ros-args \
  --params-file "$POSE_GRAPH_PROFILE" \
  "${POSE_GRAPH_PARAM_ARGS[@]}" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_mbes_pose_graph.log 2>&1 &
PG_PID=$!

ros2 run aqua_sonar_loc mbes_loop_closure_node --ros-args \
  --params-file "$MBES_LOOP_PROFILE" \
  "${MBES_LOOP_PARAM_ARGS[@]}" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_mbes_loop_closure.log 2>&1 &
LOOP_PID=$!

sleep 3

ros2 bag record -s "$RECORD_STORAGE" -o "$MBES_OUT" \
  ${RECORD_TOPIC_FLAG:+$RECORD_TOPIC_FLAG} /norbit/detections /nav/processed/odometry \
           /nav/processed/microstrain/imu/madgwick /nav/sensors/microstrain/imu/raw \
           /aqua_imu_loc/odometry /aqua_imu_loc/status \
           /aqua_sonar_loc/odometry /aqua_sonar_loc/status \
           /aqua_sonar_loc/points_filtered \
           /aqua_pose_graph/path /aqua_pose_graph/keyframe \
           /aqua_pose_graph/keyframe_count \
           /aqua_pose_graph/loop_constraint \
           /aqua_pose_graph/loop_constraint_count \
           /mbes_loop_closure/status \
           /tf /tf_static \
  > /tmp/aqua_record_mbes_bag.log 2>&1 &
REC_PID=$!

sleep 2

if [[ -n "$PLAY_DURATION_ARG" ]]; then
  ros2 bag play "$MBES_SRC" --clock "$PLAY_DURATION_ARG" "$MBES_DURATION" \
    > /tmp/aqua_record_mbes_play.log 2>&1
else
  timeout "${MBES_DURATION}s" ros2 bag play "$MBES_SRC" --clock \
    > /tmp/aqua_record_mbes_play.log 2>&1 || true
fi

sleep 3

kill -INT "$REC_PID" "$IMU_PID" "$SON_PID" "$PG_PID" "$LOOP_PID" 2>/dev/null || true
sleep 4
kill -TERM "$REC_PID" "$IMU_PID" "$SON_PID" "$PG_PID" "$LOOP_PID" 2>/dev/null || true
sleep 1

ls -la "$MBES_OUT"
echo "MBES demo bag recorded to $MBES_OUT"
