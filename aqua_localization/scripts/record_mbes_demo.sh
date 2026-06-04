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

WORKSPACE="${WORKSPACE:-aqua_loc_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
LOCAL_SETUP="${LOCAL_SETUP:-install/setup.bash}"
MBES_SRC="${MBES_SRC:-$WORKSPACE/aqua_localization/datasets/public/mbes_slam/beach_pond_ros2}"
MBES_OUT="${MBES_OUT:-$WORKSPACE/aqua_localization/datasets/public/mbes_slam/demo_with_estimate}"
IMU_PROFILE="${IMU_PROFILE:-}"
SONAR_PROFILE="${SONAR_PROFILE:-}"
POSE_GRAPH_PROFILE="${POSE_GRAPH_PROFILE:-}"
MBES_LOOP_PROFILE="${MBES_LOOP_PROFILE:-}"
IMU_QOS_SENSOR_DEPTH="${IMU_QOS_SENSOR_DEPTH:-}"
SONAR_QOS_SENSOR_DEPTH="${SONAR_QOS_SENSOR_DEPTH:-}"
IMU_SONAR_ODOMETRY_TOPIC_WAS_SET=0
if [[ -n "${IMU_SONAR_ODOMETRY_TOPIC+x}" ]]; then
  IMU_SONAR_ODOMETRY_TOPIC_WAS_SET=1
else
  IMU_SONAR_ODOMETRY_TOPIC=""
fi
MBES_DURATION="${MBES_DURATION:-60}"
RECORD_STORAGE="${RECORD_STORAGE:-mcap}"
RECORD_TOPIC_FLAG="${RECORD_TOPIC_FLAG:-}"
RECORD_READY_TIMEOUT_S="${RECORD_READY_TIMEOUT_S:-75}"
RECORD_READY_STRICT="${RECORD_READY_STRICT:-0}"
RECORD_READY_TOPICS="${RECORD_READY_TOPICS:-/aqua_imu_loc/odometry /aqua_sonar_loc/points_filtered /aqua_pose_graph/keyframe /mbes_loop_closure/status}"
PLAY_START_DELAY_S="${PLAY_START_DELAY_S:-25}"
PLAY_RATE="${PLAY_RATE:-}"
PLAY_READ_AHEAD_QUEUE_SIZE="${PLAY_READ_AHEAD_QUEUE_SIZE:-}"
PLAY_START_OFFSET_S="${PLAY_START_OFFSET_S:-}"
PLAY_WAIT_FOR_ALL_ACKED_MS="${PLAY_WAIT_FOR_ALL_ACKED_MS:-}"
PLAY_DISABLE_KEYBOARD_CONTROLS="${PLAY_DISABLE_KEYBOARD_CONTROLS:-1}"
PLAY_TIMEOUT_MARGIN_S="${PLAY_TIMEOUT_MARGIN_S:-0}"
PLAY_DURATION_ARG="${PLAY_DURATION_ARG:-}"
PLAY_TOPIC_ARGS="${PLAY_TOPIC_ARGS:-}"
POSE_GRAPH_KEYFRAME_TRANSLATION_M="${POSE_GRAPH_KEYFRAME_TRANSLATION_M:-}"
POSE_GRAPH_KEYFRAME_ROTATION_RAD="${POSE_GRAPH_KEYFRAME_ROTATION_RAD:-}"
POSE_GRAPH_ODOMETRY_TOPIC="${POSE_GRAPH_ODOMETRY_TOPIC:-}"
POSE_GRAPH_OPTIMIZATION_ITERATIONS="${POSE_GRAPH_OPTIMIZATION_ITERATIONS:-}"
POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE="${POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE:-}"
POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA="${POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA:-}"
MBES_LOOP_MIN_POINTS="${MBES_LOOP_MIN_POINTS:-}"
MBES_LOOP_VOXEL_LEAF_M="${MBES_LOOP_VOXEL_LEAF_M:-}"
MBES_LOOP_SUBMAP_FINALIZE_DELAY_KEYFRAMES="${MBES_LOOP_SUBMAP_FINALIZE_DELAY_KEYFRAMES:-}"
MBES_LOOP_MIN_KEYFRAME_SEPARATION="${MBES_LOOP_MIN_KEYFRAME_SEPARATION:-}"
MBES_LOOP_MAX_DISTANCE_M="${MBES_LOOP_MAX_DISTANCE_M:-}"
MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT="${MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT:-}"
MBES_LOOP_CANDIDATE_DESCRIPTOR_CENTROID_SCALE_M="${MBES_LOOP_CANDIDATE_DESCRIPTOR_CENTROID_SCALE_M:-}"
MBES_LOOP_CANDIDATE_DESCRIPTOR_EXTENT_SCALE="${MBES_LOOP_CANDIDATE_DESCRIPTOR_EXTENT_SCALE:-}"
MBES_LOOP_CANDIDATE_DESCRIPTOR_POINT_RATIO_SCALE="${MBES_LOOP_CANDIDATE_DESCRIPTOR_POINT_RATIO_SCALE:-}"
MBES_LOOP_MAX_FITNESS_SCORE="${MBES_LOOP_MAX_FITNESS_SCORE:-}"
MBES_LOOP_MAX_CORRECTION_TRANSLATION_M="${MBES_LOOP_MAX_CORRECTION_TRANSLATION_M:-}"
MBES_LOOP_MAX_CORRECTION_ROTATION_RAD="${MBES_LOOP_MAX_CORRECTION_ROTATION_RAD:-}"
MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M="${MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M:-}"
MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD="${MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD:-}"
MBES_LOOP_TRANSLATION_SIGMA_M="${MBES_LOOP_TRANSLATION_SIGMA_M:-}"
MBES_LOOP_ROTATION_SIGMA_RAD="${MBES_LOOP_ROTATION_SIGMA_RAD:-}"
MBES_LOOP_OPTIMIZE_AFTER_INSERT="${MBES_LOOP_OPTIMIZE_AFTER_INSERT:-}"
MBES_LOOP_DESCRIPTOR_MAX_CENTROID_DISTANCE_M="${MBES_LOOP_DESCRIPTOR_MAX_CENTROID_DISTANCE_M:-}"
MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO="${MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO:-}"
MBES_LOOP_DESCRIPTOR_MIN_POINT_COUNT_RATIO="${MBES_LOOP_DESCRIPTOR_MIN_POINT_COUNT_RATIO:-}"
MBES_LOOP_CONSISTENCY_MAX_TRANSLATION_DELTA_M="${MBES_LOOP_CONSISTENCY_MAX_TRANSLATION_DELTA_M:-}"
MBES_LOOP_CONSISTENCY_MAX_ROTATION_DELTA_RAD="${MBES_LOOP_CONSISTENCY_MAX_ROTATION_DELTA_RAD:-}"
MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT="${MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT:-}"
MBES_LOOP_SELECTION_ALLOWLIST_CSV="${MBES_LOOP_SELECTION_ALLOWLIST_CSV:-}"
MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S="${MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S:-}"
MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA="${MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA:-}"
MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M="${MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M:-}"
MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD="${MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD:-}"
MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M="${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M:-}"
MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA="${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA:-}"
MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA="${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA:-}"
MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES="${MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES:-}"

PIDS_TO_CLEAN=()
CLEANED_UP=0

cleanup_processes() {
  if [[ "$CLEANED_UP" == "1" ]]; then
    return 0
  fi
  CLEANED_UP=1
  if [[ "${#PIDS_TO_CLEAN[@]}" -eq 0 ]]; then
    return 0
  fi

  kill -INT "${PIDS_TO_CLEAN[@]}" 2>/dev/null || true
  sleep 4
  kill -TERM "${PIDS_TO_CLEAN[@]}" 2>/dev/null || true
  sleep 1
}

trap cleanup_processes EXIT

validate_ros_domain_id() {
  if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
    return 0
  fi
  if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || (( ROS_DOMAIN_ID > 232 )); then
    echo \
      "ROS_DOMAIN_ID must be an integer from 0 to 232 for this replay: $ROS_DOMAIN_ID" \
      >&2
    exit 1
  fi
}

resolve_profile() {
  local configured="$1"
  local package="$2"
  local relative_path="$3"
  local fallback="$4"
  if [[ -n "$configured" ]]; then
    printf '%s\n' "$configured"
    return 0
  fi
  local prefix
  if prefix=$(ros2 pkg prefix "$package" 2>/dev/null); then
    printf '%s/share/%s/%s\n' "$prefix" "$package" "$relative_path"
    return 0
  fi
  printf '%s\n' "$fallback"
}

require_readable_file() {
  local label="$1"
  local path="$2"
  if [[ ! -r "$path" ]]; then
    echo "$label does not exist or is not readable: $path" >&2
    exit 1
  fi
}

IMU_PARAM_ARGS=()
if [[ -n "$IMU_QOS_SENSOR_DEPTH" ]]; then
  IMU_PARAM_ARGS+=("-p" "qos.sensor_depth:=$IMU_QOS_SENSOR_DEPTH")
fi
if [[ "$IMU_SONAR_ODOMETRY_TOPIC_WAS_SET" == "1" ]]; then
  if [[ -z "$IMU_SONAR_ODOMETRY_TOPIC" ]]; then
    IMU_PARAM_ARGS+=("-p" "topics.sonar_odometry:=''")
  else
    IMU_PARAM_ARGS+=("-p" "topics.sonar_odometry:=$IMU_SONAR_ODOMETRY_TOPIC")
  fi
fi

SONAR_PARAM_ARGS=()
if [[ -n "$SONAR_QOS_SENSOR_DEPTH" ]]; then
  SONAR_PARAM_ARGS+=("-p" "qos.sensor_depth:=$SONAR_QOS_SENSOR_DEPTH")
fi

POSE_GRAPH_PARAM_ARGS=()
if [[ -n "$POSE_GRAPH_ODOMETRY_TOPIC" ]]; then
  POSE_GRAPH_PARAM_ARGS+=("-p" "topics.odometry:=$POSE_GRAPH_ODOMETRY_TOPIC")
fi
if [[ -n "$POSE_GRAPH_KEYFRAME_TRANSLATION_M" ]]; then
  POSE_GRAPH_PARAM_ARGS+=("-p" "keyframe.translation_m:=$POSE_GRAPH_KEYFRAME_TRANSLATION_M")
fi
if [[ -n "$POSE_GRAPH_KEYFRAME_ROTATION_RAD" ]]; then
  POSE_GRAPH_PARAM_ARGS+=("-p" "keyframe.rotation_rad:=$POSE_GRAPH_KEYFRAME_ROTATION_RAD")
fi
if [[ -n "$POSE_GRAPH_OPTIMIZATION_ITERATIONS" ]]; then
  POSE_GRAPH_PARAM_ARGS+=("-p" "optimization.iterations:=$POSE_GRAPH_OPTIMIZATION_ITERATIONS")
fi
if [[ -n "$POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE" ]]; then
  POSE_GRAPH_PARAM_ARGS+=("-p" "loop_constraints.robust_kernel.type:=$POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE")
fi
if [[ -n "$POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA" ]]; then
  POSE_GRAPH_PARAM_ARGS+=("-p" "loop_constraints.robust_kernel.delta:=$POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA")
fi

MBES_LOOP_PARAM_ARGS=()
if [[ -n "$MBES_LOOP_MIN_POINTS" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "submaps.min_points:=$MBES_LOOP_MIN_POINTS")
fi
if [[ -n "$MBES_LOOP_VOXEL_LEAF_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "submaps.voxel_leaf_m:=$MBES_LOOP_VOXEL_LEAF_M")
fi
if [[ -n "$MBES_LOOP_SUBMAP_FINALIZE_DELAY_KEYFRAMES" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "submaps.finalize_delay_keyframes:=$MBES_LOOP_SUBMAP_FINALIZE_DELAY_KEYFRAMES")
fi
if [[ -n "$MBES_LOOP_MIN_KEYFRAME_SEPARATION" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "candidates.min_keyframe_separation:=$MBES_LOOP_MIN_KEYFRAME_SEPARATION")
fi
if [[ -n "$MBES_LOOP_MAX_DISTANCE_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "candidates.max_distance_m:=$MBES_LOOP_MAX_DISTANCE_M")
fi
if [[ -n "$MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "candidates.descriptor_weight:=$MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT")
fi
if [[ -n "$MBES_LOOP_CANDIDATE_DESCRIPTOR_CENTROID_SCALE_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "candidates.descriptor_centroid_scale_m:=$MBES_LOOP_CANDIDATE_DESCRIPTOR_CENTROID_SCALE_M")
fi
if [[ -n "$MBES_LOOP_CANDIDATE_DESCRIPTOR_EXTENT_SCALE" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "candidates.descriptor_extent_scale:=$MBES_LOOP_CANDIDATE_DESCRIPTOR_EXTENT_SCALE")
fi
if [[ -n "$MBES_LOOP_CANDIDATE_DESCRIPTOR_POINT_RATIO_SCALE" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "candidates.descriptor_point_count_ratio_scale:=$MBES_LOOP_CANDIDATE_DESCRIPTOR_POINT_RATIO_SCALE")
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
if [[ -n "$MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "gates.min_plan_view_separation_m:=$MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M")
fi
if [[ -n "$MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "gates.max_short_plan_view_rotation_rad:=$MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD")
fi
if [[ -n "$MBES_LOOP_TRANSLATION_SIGMA_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.translation_sigma_m:=$MBES_LOOP_TRANSLATION_SIGMA_M")
fi
if [[ -n "$MBES_LOOP_ROTATION_SIGMA_RAD" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.rotation_sigma_rad:=$MBES_LOOP_ROTATION_SIGMA_RAD")
fi
if [[ -n "$MBES_LOOP_OPTIMIZE_AFTER_INSERT" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.optimize_after_insert:=$MBES_LOOP_OPTIMIZE_AFTER_INSERT")
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
if [[ -n "$MBES_LOOP_CONSISTENCY_MAX_TRANSLATION_DELTA_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.consistency.max_correction_translation_delta_m:=$MBES_LOOP_CONSISTENCY_MAX_TRANSLATION_DELTA_M")
fi
if [[ -n "$MBES_LOOP_CONSISTENCY_MAX_ROTATION_DELTA_RAD" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.consistency.max_correction_rotation_delta_rad:=$MBES_LOOP_CONSISTENCY_MAX_ROTATION_DELTA_RAD")
fi
if [[ -n "$MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.consistency.min_support_count:=$MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT")
fi
if [[ -n "$MBES_LOOP_SELECTION_ALLOWLIST_CSV" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.allowlist_csv:=$MBES_LOOP_SELECTION_ALLOWLIST_CSV")
fi
if [[ -n "$MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.match_timestamp_window_s:=$MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S")
fi
if [[ -n "$MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.match_max_fitness_delta:=$MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA")
fi
if [[ -n "$MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.match_max_translation_delta_m:=$MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M")
fi
if [[ -n "$MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.match_max_rotation_delta_rad:=$MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD")
fi
if [[ -n "$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.match_max_descriptor_centroid_delta_m:=$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M")
fi
if [[ -n "$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.match_max_descriptor_extent_ratio_delta:=$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA")
fi
if [[ -n "$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.match_max_descriptor_point_count_ratio_delta:=$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA")
fi
if [[ -n "$MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES" ]]; then
  MBES_LOOP_PARAM_ARGS+=("-p" "loop.selection.prioritize_candidates:=$MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES")
fi

wait_for_recorder_ready() {
  if [[ "$RECORD_READY_TIMEOUT_S" == "0" ]]; then
    return 0
  fi

  read -r -a ready_topics <<< "$RECORD_READY_TOPICS"
  if [[ "${#ready_topics[@]}" -eq 0 ]]; then
    return 0
  fi

  local deadline=$((SECONDS + RECORD_READY_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    local missing=0
    for topic in "${ready_topics[@]}"; do
      if ! grep -q "Subscribed to topic '$topic'" /tmp/aqua_record_mbes_bag.log 2>/dev/null; then
        missing=1
        break
      fi
    done
    if [[ "$missing" == "0" ]]; then
      echo "MBES recorder ready: essential output topics subscribed"
      return 0
    fi
    if ! kill -0 "$REC_PID" 2>/dev/null; then
      echo "MBES recorder exited before essential output topics were subscribed" >&2
      return 1
    fi
    sleep 1
  done

  echo \
    "MBES recorder readiness timeout after ${RECORD_READY_TIMEOUT_S}s; starting replay anyway" \
    >&2
  if [[ "$RECORD_READY_STRICT" == "1" ]]; then
    echo "strict recorder readiness is enabled; aborting replay" >&2
    return 1
  fi
}

compute_play_timeout_s() {
  awk \
    -v duration="$MBES_DURATION" \
    -v delay="$PLAY_START_DELAY_S" \
    -v rate="${PLAY_RATE:-1.0}" \
    -v margin="$PLAY_TIMEOUT_MARGIN_S" \
    'BEGIN {
      if (duration <= 0.0 || delay < 0.0 || rate <= 0.0 || margin < 0.0) {
        exit 1
      }
      timeout = duration / rate + delay + margin
      printf "%d", int(timeout + 0.999999)
    }'
}

validate_ros_domain_id

cd "$WORKSPACE"
# shellcheck disable=SC1091
set +u
source "$ROS_SETUP"
if [[ -n "$LOCAL_SETUP" ]]; then
  # shellcheck disable=SC1091
  source "$LOCAL_SETUP"
fi
set -u

IMU_PROFILE=$(resolve_profile \
  "$IMU_PROFILE" aqua_imu_loc config/mbes_slam.yaml \
  "$WORKSPACE/install/aqua_imu_loc/share/aqua_imu_loc/config/mbes_slam.yaml")
SONAR_PROFILE=$(resolve_profile \
  "$SONAR_PROFILE" aqua_sonar_loc config/mbes_slam.yaml \
  "$WORKSPACE/install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_slam.yaml")
POSE_GRAPH_PROFILE=$(resolve_profile \
  "$POSE_GRAPH_PROFILE" aqua_pose_graph config/params.yaml \
  "$WORKSPACE/install/aqua_pose_graph/share/aqua_pose_graph/config/params.yaml")
MBES_LOOP_PROFILE=$(resolve_profile \
  "$MBES_LOOP_PROFILE" aqua_sonar_loc config/mbes_loop_closure.yaml \
  "$WORKSPACE/install/aqua_sonar_loc/share/aqua_sonar_loc/config/mbes_loop_closure.yaml")

require_readable_file IMU_PROFILE "$IMU_PROFILE"
require_readable_file SONAR_PROFILE "$SONAR_PROFILE"
require_readable_file POSE_GRAPH_PROFILE "$POSE_GRAPH_PROFILE"
require_readable_file MBES_LOOP_PROFILE "$MBES_LOOP_PROFILE"

rm -rf "$MBES_OUT"

ros2 run aqua_imu_loc imu_loc_node --ros-args \
  --params-file "$IMU_PROFILE" \
  "${IMU_PARAM_ARGS[@]}" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_mbes_imu.log 2>&1 &
IMU_PID=$!
PIDS_TO_CLEAN+=("$IMU_PID")

ros2 run aqua_sonar_loc sonar_loc_node --ros-args \
  --params-file "$SONAR_PROFILE" \
  "${SONAR_PARAM_ARGS[@]}" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_mbes_sonar.log 2>&1 &
SON_PID=$!
PIDS_TO_CLEAN+=("$SON_PID")

ros2 run aqua_pose_graph pose_graph_node --ros-args \
  --params-file "$POSE_GRAPH_PROFILE" \
  "${POSE_GRAPH_PARAM_ARGS[@]}" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_mbes_pose_graph.log 2>&1 &
PG_PID=$!
PIDS_TO_CLEAN+=("$PG_PID")

ros2 run aqua_sonar_loc mbes_loop_closure_node --ros-args \
  --params-file "$MBES_LOOP_PROFILE" \
  "${MBES_LOOP_PARAM_ARGS[@]}" \
  -p use_sim_time:=true \
  > /tmp/aqua_record_mbes_loop_closure.log 2>&1 &
LOOP_PID=$!
PIDS_TO_CLEAN+=("$LOOP_PID")

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
           /aqua_pose_graph/optimization_count \
           /aqua_pose_graph/optimization_chi2 \
           /mbes_loop_closure/status \
           /tf /tf_static \
  > /tmp/aqua_record_mbes_bag.log 2>&1 &
REC_PID=$!
PIDS_TO_CLEAN+=("$REC_PID")

wait_for_recorder_ready

PLAY_DELAY_ARGS=()
if [[ "$PLAY_START_DELAY_S" != "0" ]]; then
  PLAY_DELAY_ARGS=("--delay" "$PLAY_START_DELAY_S")
fi
PLAY_COMMON_ARGS=("--clock")
if [[ -n "$PLAY_RATE" ]]; then
  PLAY_COMMON_ARGS+=("--rate" "$PLAY_RATE")
fi
if [[ -n "$PLAY_READ_AHEAD_QUEUE_SIZE" ]]; then
  PLAY_COMMON_ARGS+=("--read-ahead-queue-size" "$PLAY_READ_AHEAD_QUEUE_SIZE")
fi
if [[ -n "$PLAY_START_OFFSET_S" ]]; then
  PLAY_COMMON_ARGS+=("--start-offset" "$PLAY_START_OFFSET_S")
fi
if [[ -n "$PLAY_WAIT_FOR_ALL_ACKED_MS" ]]; then
  PLAY_COMMON_ARGS+=("--wait-for-all-acked" "$PLAY_WAIT_FOR_ALL_ACKED_MS")
fi
if [[ "$PLAY_DISABLE_KEYBOARD_CONTROLS" == "1" ]]; then
  PLAY_COMMON_ARGS+=("--disable-keyboard-controls")
fi

if [[ -n "$PLAY_DURATION_ARG" ]]; then
  ros2 bag play "$MBES_SRC" "${PLAY_COMMON_ARGS[@]}" "${PLAY_DELAY_ARGS[@]}" \
    "$PLAY_DURATION_ARG" "$MBES_DURATION" \
    ${PLAY_TOPIC_ARGS:+$PLAY_TOPIC_ARGS} \
    > /tmp/aqua_record_mbes_play.log 2>&1
else
  if ! PLAY_TIMEOUT_S=$(compute_play_timeout_s); then
    echo "invalid playback timing: duration=$MBES_DURATION delay=$PLAY_START_DELAY_S rate=${PLAY_RATE:-1.0} margin=$PLAY_TIMEOUT_MARGIN_S" >&2
    exit 1
  fi
  timeout "${PLAY_TIMEOUT_S}s" ros2 bag play "$MBES_SRC" "${PLAY_COMMON_ARGS[@]}" \
    "${PLAY_DELAY_ARGS[@]}" \
    ${PLAY_TOPIC_ARGS:+$PLAY_TOPIC_ARGS} \
    > /tmp/aqua_record_mbes_play.log 2>&1 || true
fi

sleep 3
cleanup_processes

ls -la "$MBES_OUT"
echo "MBES demo bag recorded to $MBES_OUT"
