#!/usr/bin/env bash
# Sweep MBES loop candidate descriptor-ranking weights on the benchmark replay.
#
# Set DRY_RUN=1 to print the commands without executing them.

set -euo pipefail

WORKSPACE="${WORKSPACE:-$(pwd)}"
OUT_ROOT="${OUT_ROOT:-/tmp/aqua_mbes_candidate_descriptor_weight_sweep}"
WEIGHTS="${WEIGHTS:-0,0.25,0.5,1.0,2.0}"
SUMMARY_OUT="${SUMMARY_OUT:-$OUT_ROOT/mbes_candidate_descriptor_weight_sweep.md}"
CSV_OUT="${CSV_OUT:-$OUT_ROOT/mbes_candidate_descriptor_weight_sweep.csv}"
DATASET="${DATASET:-MBES-SLAM}"
SEQUENCE="${SEQUENCE:-beach_pond}"
MBES_DURATION="${MBES_DURATION:-120}"
SWEEP_ROS_DOMAIN_ID_START="${SWEEP_ROS_DOMAIN_ID_START:-$((30 + RANDOM % 120))}"
DRY_RUN="${DRY_RUN:-0}"

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
BENCHMARK_SCRIPT="$SCRIPT_DIR/run_mbes_loop_benchmark.sh"

validate_ros_domain_id() {
  local name="$1"
  local value="$2"
  if ! [[ "$value" =~ ^[0-9]+$ ]] || (( value > 232 )); then
    echo "$name must be an integer from 0 to 232 for this sweep: $value" >&2
    exit 1
  fi
}

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

label_number() {
  local value="$1"
  value="${value//-/m}"
  value="${value//./p}"
  printf '%s' "$value"
}

mkdir -p "$OUT_ROOT"
validate_ros_domain_id SWEEP_ROS_DOMAIN_ID_START "$SWEEP_ROS_DOMAIN_ID_START"

IFS=', ' read -r -a WEIGHT_VALUES <<< "$WEIGHTS"
SUMMARY_CASE_ARGS=()
case_index=0

for weight in "${WEIGHT_VALUES[@]}"; do
  if [[ -z "$weight" ]]; then
    continue
  fi
  label=$(label_number "$weight")
  case_out="$OUT_ROOT/weight_$label"
  case_bag="$OUT_ROOT/bags/mbes_weight_$label"
  case_ros_domain_id=$((SWEEP_ROS_DOMAIN_ID_START + case_index))
  validate_ros_domain_id "case ROS_DOMAIN_ID" "$case_ros_domain_id"
  SUMMARY_CASE_ARGS+=("--case" "$weight:$case_out")
  run_env_cmd \
    "WORKSPACE=$WORKSPACE" \
    "ROS_DOMAIN_ID=$case_ros_domain_id" \
    "OUT_DIR=$case_out" \
    "MBES_OUT=$case_bag" \
    "MBES_DURATION=$MBES_DURATION" \
    "DATASET=$DATASET" \
    "SEQUENCE=$SEQUENCE" \
    "NOTE=candidate descriptor weight $weight, duration ${MBES_DURATION}s" \
    "MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT=$weight" \
    -- "$BENCHMARK_SCRIPT"
  case_index=$((case_index + 1))
done

SUMMARY_ARGS=(
  "${SUMMARY_CASE_ARGS[@]}"
  "--out" "$SUMMARY_OUT"
  "--csv-out" "$CSV_OUT"
  "--dataset" "$DATASET"
  "--sequence" "$SEQUENCE"
)

if [[ -n "${MBES_LOOP_CANDIDATE_DESCRIPTOR_CENTROID_SCALE_M+x}" ]]; then
  SUMMARY_ARGS+=(
    "--descriptor-centroid-scale-m"
    "$MBES_LOOP_CANDIDATE_DESCRIPTOR_CENTROID_SCALE_M"
  )
fi
if [[ -n "${MBES_LOOP_CANDIDATE_DESCRIPTOR_EXTENT_SCALE+x}" ]]; then
  SUMMARY_ARGS+=(
    "--descriptor-extent-scale"
    "$MBES_LOOP_CANDIDATE_DESCRIPTOR_EXTENT_SCALE"
  )
fi
if [[ -n "${MBES_LOOP_CANDIDATE_DESCRIPTOR_POINT_RATIO_SCALE+x}" ]]; then
  SUMMARY_ARGS+=(
    "--descriptor-point-ratio-scale"
    "$MBES_LOOP_CANDIDATE_DESCRIPTOR_POINT_RATIO_SCALE"
  )
fi

run_cmd ros2 run aqua_localization summarize_mbes_candidate_descriptor_weight_sweep.py \
  "${SUMMARY_ARGS[@]}"

cat <<EOF

MBES candidate descriptor-weight sweep artifacts:
  root:    $OUT_ROOT
  summary: $SUMMARY_OUT
  csv:     $CSV_OUT
  ROS_DOMAIN_ID start: $SWEEP_ROS_DOMAIN_ID_START
EOF
