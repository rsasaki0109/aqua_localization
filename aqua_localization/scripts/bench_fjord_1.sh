#!/usr/bin/env bash
# Replay NTNU subset-fjord/fjord_1, record /aqua_imu_loc/odometry to a TUM file,
# and compare against fjord_1_baseline.tum. Append the APE metrics to a benchmark
# log so future UKF/parameter changes can be tracked over time.
#
# Usage:
#   bench_fjord_1.sh [output_dir]
#
# Defaults the output dir to docs/benchmarks/. Requires a converted rosbag2 at
# datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_ros2 and a sourced
# install/setup.bash.

set -euo pipefail

WORKSPACE_ROOT=${WORKSPACE_ROOT:-$(pwd)}
BAG_PATH=${BAG_PATH:-aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_ros2}
BASELINE_TUM=${BASELINE_TUM:-aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_baseline.tum}
BAG_RATE=${BAG_RATE:-10.0}
RECORD_DURATION=${RECORD_DURATION:-40s}
OUT_DIR=${1:-aqua_localization/docs/benchmarks}
RUN_ID=${RUN_ID:-$(date +%Y-%m-%dT%H-%M-%S)}
TUM_FILE="/tmp/bench_fjord_1_${RUN_ID}.tum"

mkdir -p "${OUT_DIR}"
LOG_FILE="${OUT_DIR}/fjord_1.md"

if [[ ! -d "${BAG_PATH}" ]]; then
  echo "bag not found: ${BAG_PATH}" >&2
  echo "(see datasets/ntnu_demo.md for download + ROS 1 -> ROS 2 conversion)" >&2
  exit 2
fi
if [[ ! -f "${BASELINE_TUM}" ]]; then
  echo "baseline TUM not found: ${BASELINE_TUM}" >&2
  exit 2
fi

cleanup() {
  local pid=${LAUNCH_PID:-}
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -9 "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
  pkill -9 -f "imu_loc_node" 2>/dev/null || true
  pkill -9 -f "rosbag2_player" 2>/dev/null || true
}
trap cleanup EXIT

echo "[bench] launching ntnu_fjord_demo at ${BAG_RATE}x"
ros2 launch aqua_localization ntnu_fjord_demo.launch.py \
  enable_rviz:=false bag_rate:="${BAG_RATE}" bag_path:="${BAG_PATH}" >/dev/null 2>&1 &
LAUNCH_PID=$!

sleep 2
echo "[bench] recording odometry to ${TUM_FILE} for ${RECORD_DURATION}"
timeout "${RECORD_DURATION}" ros2 run aqua_localization record_odometry.py \
  --topic /aqua_imu_loc/odometry \
  --out "${TUM_FILE}" \
  --format tum >/dev/null 2>&1 || true

cleanup

if [[ ! -s "${TUM_FILE}" ]]; then
  echo "no odometry samples were recorded; check launch output" >&2
  exit 3
fi

echo "[bench] computing APE against ${BASELINE_TUM}"
COMPARE_OUT=$(ros2 run aqua_localization compare_trajectories.py "${BASELINE_TUM}" "${TUM_FILE}")
echo "${COMPARE_OUT}"

MEAN=$(echo "${COMPARE_OUT}" | awk '/  mean/ {print $3}')
RMSE=$(echo "${COMPARE_OUT}" | awk '/  rmse/ {print $3}')
COUNT=$(echo "${COMPARE_OUT}" | awk '/^  count/ {print $3}')
DURATION=$(echo "${COMPARE_OUT}" | awk '/matched duration/ {print $3}')

if [[ ! -f "${LOG_FILE}" ]]; then
  cat > "${LOG_FILE}" <<EOF
# fjord_1 APE history

Translation APE (rigid SE(3) Umeyama alignment) of \`/aqua_imu_loc/odometry\` against
\`fjord_1_baseline.tum\`. Each row is one run of \`scripts/bench_fjord_1.sh\`.

| run id | bag_rate | matched count | matched s | mean (m) | rmse (m) | note |
|--------|---------:|---------------:|----------:|---------:|---------:|------|
EOF
fi

NOTE=${BENCH_NOTE:-""}
ROW="| ${RUN_ID} | ${BAG_RATE} | ${COUNT} | ${DURATION} | ${MEAN} | ${RMSE} | ${NOTE} |"

# Insert the row immediately after the last existing table line (lines starting with "|")
# so manual prose sections such as "## Honest takeaways" stay below the table.
if grep -q "^|" "${LOG_FILE}"; then
  LAST_TABLE_LINE=$(grep -n "^|" "${LOG_FILE}" | tail -1 | cut -d: -f1)
  awk -v n="${LAST_TABLE_LINE}" -v row="${ROW}" \
    'NR==n{print; print row; next} {print}' "${LOG_FILE}" > "${LOG_FILE}.tmp"
  mv "${LOG_FILE}.tmp" "${LOG_FILE}"
else
  echo "${ROW}" >> "${LOG_FILE}"
fi
echo "[bench] appended row to ${LOG_FILE}"
