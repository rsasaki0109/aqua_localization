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
SELECTED_GEOMETRY_AUDIT_REQUIRE_COMPLETE="${SELECTED_GEOMETRY_AUDIT_REQUIRE_COMPLETE:-0}"
NORMAL_ROS_DOMAIN_ID="${NORMAL_ROS_DOMAIN_ID:-$((20 + RANDOM % 90))}"
SELECTED_ROS_DOMAIN_ID="${SELECTED_ROS_DOMAIN_ID:-$((120 + RANDOM % 90))}"
ALLOWLIST_AUDIT_STRICT="${ALLOWLIST_AUDIT_STRICT:-1}"
DRY_RUN="${DRY_RUN:-0}"
ALLOWLIST_AUDIT_ARGS=()
if [[ "$ALLOWLIST_AUDIT_STRICT" == "1" ]]; then
  ALLOWLIST_AUDIT_ARGS+=("--strict")
fi
BENCHMARK_ENV_ARGS=()
for optional_name in POSE_GRAPH_ODOMETRY_TOPIC; do
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
  allowlist audit:       $ALLOWLIST_AUDIT_OUT
EOF
