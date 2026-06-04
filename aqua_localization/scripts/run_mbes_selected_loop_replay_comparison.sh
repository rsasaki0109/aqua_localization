#!/usr/bin/env bash
# Run normal and batch-selected MBES loop replays, then compare trajectory metrics.
#
# Set DRY_RUN=1 to print the two benchmark commands and comparison command.

set -euo pipefail

WORKSPACE="${WORKSPACE:-$(pwd)}"
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BENCHMARK_SCRIPT="$SCRIPT_DIR/run_mbes_loop_benchmark.sh"
OUT_ROOT="${OUT_ROOT:-/tmp/aqua_mbes_selected_loop_comparison}"
NORMAL_OUT_DIR="${NORMAL_OUT_DIR:-$OUT_ROOT/normal}"
SELECTED_OUT_DIR="${SELECTED_OUT_DIR:-$OUT_ROOT/selected}"
NORMAL_MBES_OUT="${NORMAL_MBES_OUT:-$OUT_ROOT/normal_replay_bag}"
SELECTED_MBES_OUT="${SELECTED_MBES_OUT:-$OUT_ROOT/selected_replay_bag}"
NORMAL_SELECTED_CSV="${NORMAL_SELECTED_CSV:-$NORMAL_OUT_DIR/mbes_beach_pond_batch_selected_loops.csv}"
NORMAL_METRICS_OUT="${NORMAL_METRICS_OUT:-$NORMAL_OUT_DIR/mbes_beach_pond_loop_trajectory_metrics.md}"
SELECTED_METRICS_OUT="${SELECTED_METRICS_OUT:-$SELECTED_OUT_DIR/mbes_beach_pond_loop_trajectory_metrics.md}"
SELECTED_STATUS_CSV="${SELECTED_STATUS_CSV:-$SELECTED_OUT_DIR/mbes_beach_pond_loop_status.csv}"
COMPARISON_OUT="${COMPARISON_OUT:-$OUT_ROOT/mbes_selected_loop_replay_comparison.md}"
ALLOWLIST_AUDIT_OUT="${ALLOWLIST_AUDIT_OUT:-$OUT_ROOT/mbes_selected_loop_allowlist_audit.md}"
SELECTED_COVERAGE_OUT="${SELECTED_COVERAGE_OUT:-$OUT_ROOT/mbes_selected_loop_coverage.md}"
SELECTED_COVERAGE_TIMESTAMP_WINDOW_S="${SELECTED_COVERAGE_TIMESTAMP_WINDOW_S:-1.0}"
DESCRIPTOR_RETRIEVAL_OUT="${DESCRIPTOR_RETRIEVAL_OUT:-$OUT_ROOT/mbes_descriptor_retrieval.md}"
DESCRIPTOR_RETRIEVAL_CURRENT_TIMESTAMP_WINDOW_S="${DESCRIPTOR_RETRIEVAL_CURRENT_TIMESTAMP_WINDOW_S:-${MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S:-5.0}}"
DESCRIPTOR_RETRIEVAL_CANDIDATE_TIMESTAMP_WINDOW_S="${DESCRIPTOR_RETRIEVAL_CANDIDATE_TIMESTAMP_WINDOW_S:-1.0}"
DESCRIPTOR_RETRIEVAL_CENTROID_SCALE_M="${DESCRIPTOR_RETRIEVAL_CENTROID_SCALE_M:-${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M:-0.6}}"
DESCRIPTOR_RETRIEVAL_EXTENT_SCALE="${DESCRIPTOR_RETRIEVAL_EXTENT_SCALE:-${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA:-0.15}}"
DESCRIPTOR_RETRIEVAL_POINT_RATIO_SCALE="${DESCRIPTOR_RETRIEVAL_POINT_RATIO_SCALE:-${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA:-0.10}}"
REMAPPED_SELECTED_CSV="${REMAPPED_SELECTED_CSV:-$OUT_ROOT/mbes_selected_loop_remapped_allowlist.csv}"
REMAPPED_SELECTED_REPORT="${REMAPPED_SELECTED_REPORT:-$OUT_ROOT/mbes_selected_loop_remap.md}"
SELECTED_REMAP_TIMESTAMP_WINDOW_S="${SELECTED_REMAP_TIMESTAMP_WINDOW_S:-1.0}"
SELECTED_GEOMETRY_AUDIT_REQUIRE_COMPLETE="${SELECTED_GEOMETRY_AUDIT_REQUIRE_COMPLETE:-0}"
NORMAL_ROS_DOMAIN_ID="${NORMAL_ROS_DOMAIN_ID:-$((20 + RANDOM % 90))}"
SELECTED_ROS_DOMAIN_ID="${SELECTED_ROS_DOMAIN_ID:-$((120 + RANDOM % 90))}"
ALLOWLIST_AUDIT_STRICT="${ALLOWLIST_AUDIT_STRICT:-1}"
DRY_RUN="${DRY_RUN:-0}"
ALLOWLIST_AUDIT_ARGS=()

validate_ros_domain_id() {
  local name="$1"
  local value="$2"
  if ! [[ "$value" =~ ^[0-9]+$ ]] || (( value > 232 )); then
    echo "$name must be an integer from 0 to 232 for this replay: $value" >&2
    exit 1
  fi
}

validate_ros_domain_id NORMAL_ROS_DOMAIN_ID "$NORMAL_ROS_DOMAIN_ID"
validate_ros_domain_id SELECTED_ROS_DOMAIN_ID "$SELECTED_ROS_DOMAIN_ID"

if [[ "$ALLOWLIST_AUDIT_STRICT" == "1" ]]; then
  ALLOWLIST_AUDIT_ARGS+=("--strict")
fi
if [[ -n "${MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S+x}" ]]; then
  ALLOWLIST_AUDIT_ARGS+=(
    "--signature-timestamp-window-s"
    "$MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S"
  )
fi
if [[ -n "${MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA+x}" ]]; then
  ALLOWLIST_AUDIT_ARGS+=(
    "--signature-max-fitness-delta"
    "$MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA"
  )
fi
if [[ -n "${MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M+x}" ]]; then
  ALLOWLIST_AUDIT_ARGS+=(
    "--signature-max-translation-delta-m"
    "$MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M"
  )
fi
if [[ -n "${MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD+x}" ]]; then
  ALLOWLIST_AUDIT_ARGS+=(
    "--signature-max-rotation-delta-rad"
    "$MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD"
  )
fi
if [[ -n "${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M+x}" ]]; then
  ALLOWLIST_AUDIT_ARGS+=(
    "--signature-max-descriptor-centroid-delta-m"
    "$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M"
  )
fi
if [[ -n "${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA+x}" ]]; then
  ALLOWLIST_AUDIT_ARGS+=(
    "--signature-max-descriptor-extent-ratio-delta"
    "$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA"
  )
fi
if [[ -n "${MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA+x}" ]]; then
  ALLOWLIST_AUDIT_ARGS+=(
    "--signature-max-descriptor-point-count-ratio-delta"
    "$MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA"
  )
fi
BENCHMARK_ENV_ARGS=()
for optional_name in \
  POSE_GRAPH_ODOMETRY_TOPIC \
  POSE_GRAPH_OPTIMIZATION_ITERATIONS \
  POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE \
  POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA \
  MBES_LOOP_MIN_POINTS \
  MBES_LOOP_VOXEL_LEAF_M \
  MBES_LOOP_MIN_KEYFRAME_SEPARATION \
  MBES_LOOP_MAX_DISTANCE_M \
  MBES_LOOP_MAX_FITNESS_SCORE \
  MBES_LOOP_MAX_CORRECTION_TRANSLATION_M \
  MBES_LOOP_MAX_CORRECTION_ROTATION_RAD \
  MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M \
  MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD \
  MBES_LOOP_TRANSLATION_SIGMA_M \
  MBES_LOOP_ROTATION_SIGMA_RAD \
  MBES_LOOP_OPTIMIZE_AFTER_INSERT \
  MBES_LOOP_DESCRIPTOR_MAX_CENTROID_DISTANCE_M \
  MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO \
  MBES_LOOP_DESCRIPTOR_MIN_POINT_COUNT_RATIO \
  MBES_LOOP_CONSISTENCY_MAX_TRANSLATION_DELTA_M \
  MBES_LOOP_CONSISTENCY_MAX_ROTATION_DELTA_RAD \
  MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT \
  MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S \
  MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA \
  MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M \
  MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD \
  MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_CENTROID_DELTA_M \
  MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_EXTENT_RATIO_DELTA \
  MBES_LOOP_SELECTION_MATCH_MAX_DESCRIPTOR_POINT_COUNT_RATIO_DELTA \
  MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES \
  MBES_LOOP_BATCH_CONSISTENCY_MAX_FITNESS_SCORE
do
  if [[ -n "${!optional_name+x}" ]]; then
    BENCHMARK_ENV_ARGS+=("$optional_name=${!optional_name}")
  fi
done

run_cmd() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  if [[ "$DRY_RUN" == "1" ]]; then
    return 0
  fi
  "$@"
}

run_env_cmd() {
  local -a env_args=()
  while [[ "$#" -gt 0 ]]; do
    case "$1" in
      --)
        shift
        break
        ;;
      *)
        env_args+=("$1")
        shift
        ;;
    esac
  done

  printf '+'
  for arg in "${env_args[@]}"; do
    printf ' %q' "$arg"
  done
  printf ' %q' "$@"
  printf '\n'
  if [[ "$DRY_RUN" == "1" ]]; then
    return 0
  fi
  env "${env_args[@]}" "$@"
}

csv_data_row_count() {
  awk 'NR > 1 && $0 !~ /^[[:space:]]*$/ {count += 1} END {print count + 0}' "$1"
}

mkdir -p "$OUT_ROOT"

run_env_cmd \
  "ROS_DOMAIN_ID=$NORMAL_ROS_DOMAIN_ID" \
  "${BENCHMARK_ENV_ARGS[@]}" \
  "WORKSPACE=$WORKSPACE" \
  "OUT_DIR=$NORMAL_OUT_DIR" \
  "MBES_OUT=$NORMAL_MBES_OUT" \
  "NOTE=normal replay for selected-loop comparison" \
  -- "$BENCHMARK_SCRIPT"

if [[ "$DRY_RUN" != "1" ]]; then
  if [[ ! -f "$NORMAL_SELECTED_CSV" ]]; then
    echo "missing selected loop CSV from normal replay: $NORMAL_SELECTED_CSV" >&2
    exit 1
  fi
  NORMAL_SELECTED_ROWS=$(csv_data_row_count "$NORMAL_SELECTED_CSV")
  if (( NORMAL_SELECTED_ROWS == 0 )); then
    echo "selected loop CSV has no data rows: $NORMAL_SELECTED_CSV" >&2
    exit 1
  fi
fi

run_env_cmd \
  "ROS_DOMAIN_ID=$SELECTED_ROS_DOMAIN_ID" \
  "${BENCHMARK_ENV_ARGS[@]}" \
  "WORKSPACE=$WORKSPACE" \
  "OUT_DIR=$SELECTED_OUT_DIR" \
  "MBES_OUT=$SELECTED_MBES_OUT" \
  "MBES_LOOP_SELECTION_ALLOWLIST_CSV=$NORMAL_SELECTED_CSV" \
  "GEOMETRY_AUDIT_REQUIRE_COMPLETE=$SELECTED_GEOMETRY_AUDIT_REQUIRE_COMPLETE" \
  "NOTE=selected-loop replay using $NORMAL_SELECTED_CSV" \
  -- "$BENCHMARK_SCRIPT"

run_cmd ros2 run aqua_localization compare_mbes_loop_metric_reports.py \
  --normal "$NORMAL_METRICS_OUT" \
  --selected "$SELECTED_METRICS_OUT" \
  --out "$COMPARISON_OUT"

run_cmd ros2 run aqua_localization diagnose_mbes_selected_loop_coverage.py \
  --selected "$NORMAL_SELECTED_CSV" \
  --status "$SELECTED_STATUS_CSV" \
  --out "$SELECTED_COVERAGE_OUT" \
  --timestamp-window-s "$SELECTED_COVERAGE_TIMESTAMP_WINDOW_S"

run_cmd ros2 run aqua_localization evaluate_mbes_descriptor_retrieval.py \
  --selected "$NORMAL_SELECTED_CSV" \
  --status "$SELECTED_STATUS_CSV" \
  --out "$DESCRIPTOR_RETRIEVAL_OUT" \
  --current-timestamp-window-s "$DESCRIPTOR_RETRIEVAL_CURRENT_TIMESTAMP_WINDOW_S" \
  --candidate-timestamp-window-s "$DESCRIPTOR_RETRIEVAL_CANDIDATE_TIMESTAMP_WINDOW_S" \
  --descriptor-centroid-scale-m "$DESCRIPTOR_RETRIEVAL_CENTROID_SCALE_M" \
  --descriptor-extent-scale "$DESCRIPTOR_RETRIEVAL_EXTENT_SCALE" \
  --descriptor-point-ratio-scale "$DESCRIPTOR_RETRIEVAL_POINT_RATIO_SCALE"

run_cmd ros2 run aqua_localization remap_mbes_selected_loop_allowlist.py \
  --selected "$NORMAL_SELECTED_CSV" \
  --status "$SELECTED_STATUS_CSV" \
  --out "$REMAPPED_SELECTED_CSV" \
  --report-out "$REMAPPED_SELECTED_REPORT" \
  --timestamp-window-s "$SELECTED_REMAP_TIMESTAMP_WINDOW_S"

run_cmd ros2 run aqua_localization check_mbes_loop_allowlist_replay.py \
  --allowlist "$NORMAL_SELECTED_CSV" \
  --status "$SELECTED_STATUS_CSV" \
  --out "$ALLOWLIST_AUDIT_OUT" \
  "${ALLOWLIST_AUDIT_ARGS[@]}"

cat <<EOF

MBES selected-loop replay comparison artifacts:
  normal out dir:       $NORMAL_OUT_DIR
  selected out dir:     $SELECTED_OUT_DIR
  normal ROS_DOMAIN_ID: $NORMAL_ROS_DOMAIN_ID
  selected ROS_DOMAIN_ID: $SELECTED_ROS_DOMAIN_ID
  selected loop CSV:    $NORMAL_SELECTED_CSV
  normal metrics:       $NORMAL_METRICS_OUT
  selected metrics:     $SELECTED_METRICS_OUT
  comparison report:    $COMPARISON_OUT
  coverage report:      $SELECTED_COVERAGE_OUT
  descriptor retrieval: $DESCRIPTOR_RETRIEVAL_OUT
  remapped allowlist:   $REMAPPED_SELECTED_CSV
  remap report:         $REMAPPED_SELECTED_REPORT
  allowlist audit:       $ALLOWLIST_AUDIT_OUT
EOF
