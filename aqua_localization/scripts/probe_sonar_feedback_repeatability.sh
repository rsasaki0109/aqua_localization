#!/usr/bin/env bash
# Run the same MBES benchmark twice and gate raw inter-run agreement.

set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE="${WORKSPACE:-$(cd -- "$SCRIPT_DIR/../.." && pwd)}"
MBES_SRC="${MBES_SRC:-$WORKSPACE/datasets/public/mbes_slam/beach_pond_ros2}"
PROBE_OUT="${PROBE_OUT:-/tmp/aqua_mbes_repeat_probe}"
MBES_DURATION="${MBES_DURATION:-120}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
WINDOW_AGREEMENT_MEAN_MAX_M="${WINDOW_AGREEMENT_MEAN_MAX_M:-1.0}"

if pgrep -af '[i]mu_loc_node|[s]onar_loc_node|[p]ose_graph_node|[m]bes_loop_closure_node|ros2 bag (play|record)' >/dev/null; then
  echo "stale aqua/rosbag processes detected; stop them before a repeatability probe:" >&2
  pgrep -af '[i]mu_loc_node|[s]onar_loc_node|[p]ose_graph_node|[m]bes_loop_closure_node|ros2 bag (play|record)' >&2 || true
  exit 2
fi

mkdir -p "$PROBE_OUT"

for run_number in 1 2; do
  run_dir="$PROBE_OUT/run${run_number}"
  benchmark_dir="$run_dir/benchmark"
  recorded_bag="$run_dir/recorded"
  mkdir -p "$run_dir"

  env \
    WORKSPACE="$WORKSPACE" \
    MBES_SRC="$MBES_SRC" \
    MBES_OUT="$recorded_bag" \
    OUT_DIR="$benchmark_dir" \
    MBES_DURATION="$MBES_DURATION" \
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    "$SCRIPT_DIR/run_mbes_loop_benchmark.sh"

  ros2 run aqua_localization export_rosbag_odometry_tum.py \
    --bag "$recorded_bag" \
    --topic /aqua_imu_loc/odometry \
    --out "$run_dir/input_odometry.tum"
  cp "$benchmark_dir/mbes_beach_pond_loop_status.csv" \
    "$run_dir/mbes_loop_status.csv"
  ros2 run aqua_localization export_estimator_status.py \
    --bag "$recorded_bag" \
    --out "$run_dir/estimator_status.csv"
done

ros2 run aqua_localization compare_mbes_repeat_probe.py \
  "$PROBE_OUT/run1" \
  "$PROBE_OUT/run2" \
  --out "$PROBE_OUT/mbes_repeat_probe_compare.md" \
  --window-agreement-mean-max-m "$WINDOW_AGREEMENT_MEAN_MAX_M" \
  --require-window-agreement

echo "MBES repeatability probe: $PROBE_OUT/mbes_repeat_probe_compare.md"

