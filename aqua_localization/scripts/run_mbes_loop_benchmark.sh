#!/usr/bin/env bash
# Run the MBES-SLAM beach_pond loop-status benchmark artifact pipeline.
#
# This orchestrates:
#   1. Source bag readiness check.
#   2. Results-included replay recording with MBES loop diagnostics.
#   3. Loop-status CSV/summary/descriptor-sweep export.
#   4. Markdown benchmark-row generation.
#
# Set DRY_RUN=1 to print the commands without executing them.

set -euo pipefail

WORKSPACE="${WORKSPACE:-$(pwd)}"
MBES_SRC="${MBES_SRC:-$WORKSPACE/datasets/public/mbes_slam/beach_pond_ros2}"
MBES_OUT="${MBES_OUT:-/tmp/aqua_mbes_beach_pond_with_loop_status}"
MBES_DURATION="${MBES_DURATION:-120}"
OUT_DIR="${OUT_DIR:-/tmp/aqua_mbes_loop_benchmark}"
DATASET="${DATASET:-MBES-SLAM}"
SEQUENCE="${SEQUENCE:-beach_pond}"
NOTE="${NOTE:-real replay, duration ${MBES_DURATION}s}"
MIN_DURATION_S="${MIN_DURATION_S:-60}"
DRY_RUN="${DRY_RUN:-0}"

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
RECORD_SCRIPT="$SCRIPT_DIR/record_mbes_demo.sh"

READINESS_OUT="$OUT_DIR/mbes_beach_pond_readiness.md"
STATUS_CSV="$OUT_DIR/mbes_beach_pond_loop_status.csv"
SUMMARY_OUT="$OUT_DIR/mbes_beach_pond_loop_status.md"
DESCRIPTOR_SWEEP_OUT="$OUT_DIR/mbes_beach_pond_descriptor_sweep.md"
ROW_OUT="$OUT_DIR/mbes_beach_pond_benchmark_row.md"
AUDIT_OUT="$OUT_DIR/mbes_beach_pond_loop_audit.md"
AUDIT_PLOT_OUT="$OUT_DIR/mbes_beach_pond_loop_audit.png"
RECORD_ENV_ARGS=(
  "WORKSPACE=$WORKSPACE"
  "MBES_SRC=$MBES_SRC"
  "MBES_OUT=$MBES_OUT"
  "MBES_DURATION=$MBES_DURATION"
)

for optional_name in \
  ROS_SETUP RECORD_STORAGE RECORD_TOPIC_FLAG PLAY_DURATION_ARG \
  POSE_GRAPH_KEYFRAME_TRANSLATION_M POSE_GRAPH_KEYFRAME_ROTATION_RAD \
  MBES_LOOP_MIN_POINTS MBES_LOOP_VOXEL_LEAF_M \
  MBES_LOOP_MIN_KEYFRAME_SEPARATION MBES_LOOP_MAX_DISTANCE_M
do
  if [[ -n "${!optional_name+x}" ]]; then
    RECORD_ENV_ARGS+=("$optional_name=${!optional_name}")
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

mkdir -p "$OUT_DIR"

run_cmd ros2 run aqua_localization check_mbes_benchmark_ready.py \
  --bag "$MBES_SRC" \
  --out "$READINESS_OUT" \
  --min-duration-s "$MIN_DURATION_S"

run_env_cmd "${RECORD_ENV_ARGS[@]}" -- "$RECORD_SCRIPT"

run_cmd ros2 run aqua_localization export_mbes_loop_status.py \
  --bag "$MBES_OUT" \
  --out "$STATUS_CSV" \
  --summary-out "$SUMMARY_OUT" \
  --descriptor-sweep-out "$DESCRIPTOR_SWEEP_OUT"

run_cmd ros2 run aqua_localization mbes_loop_benchmark_row.py \
  --csv "$STATUS_CSV" \
  --dataset "$DATASET" \
  --sequence "$SEQUENCE" \
  --duration "$MBES_DURATION" \
  --note "$NOTE" \
  --header \
  --out "$ROW_OUT"

run_cmd ros2 run aqua_localization audit_mbes_loop_candidates.py \
  --csv "$STATUS_CSV" \
  --out "$AUDIT_OUT"

run_cmd ros2 run aqua_localization plot_mbes_loop_audit.py \
  --bag "$MBES_OUT" \
  --csv "$STATUS_CSV" \
  --out "$AUDIT_PLOT_OUT" \
  --title "$DATASET $SEQUENCE accepted loop audit"

cat <<EOF

MBES loop benchmark artifacts:
  readiness:        $READINESS_OUT
  recorded bag:     $MBES_OUT
  status CSV:       $STATUS_CSV
  summary:          $SUMMARY_OUT
  descriptor sweep: $DESCRIPTOR_SWEEP_OUT
  benchmark row:    $ROW_OUT
  audit report:      $AUDIT_OUT
  audit plot:        $AUDIT_PLOT_OUT
EOF
