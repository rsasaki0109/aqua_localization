#!/usr/bin/env bash
# One-command smoke test for the pose-graph loop-closure path.

set -euo pipefail

STARTUP_TIMEOUT_S="${STARTUP_TIMEOUT_S:-10}"
ECHO_TIMEOUT_S="${ECHO_TIMEOUT_S:-8}"
EXPECTED_KEYFRAMES="${EXPECTED_KEYFRAMES:-5}"
EXPECTED_LOOPS="${EXPECTED_LOOPS:-1}"

LOG_DIR="${LOG_DIR:-/tmp/aqua_pose_graph_loop_smoke}"
mkdir -p "$LOG_DIR"
POSE_GRAPH_LOG="$LOG_DIR/pose_graph.log"
DEMO_LOG="$LOG_DIR/demo.log"

POSE_GRAPH_PID=""

cleanup() {
  if [[ -n "$POSE_GRAPH_PID" ]] && kill -0 "$POSE_GRAPH_PID" 2>/dev/null; then
    kill -INT "$POSE_GRAPH_PID" 2>/dev/null || true
    local deadline=$((SECONDS + 5))
    while kill -0 "$POSE_GRAPH_PID" 2>/dev/null && (( SECONDS < deadline )); do
      sleep 0.2
    done
    if kill -0 "$POSE_GRAPH_PID" 2>/dev/null; then
      kill -TERM "$POSE_GRAPH_PID" 2>/dev/null || true
    fi
    deadline=$((SECONDS + 3))
    while kill -0 "$POSE_GRAPH_PID" 2>/dev/null && (( SECONDS < deadline )); do
      sleep 0.2
    done
    if kill -0 "$POSE_GRAPH_PID" 2>/dev/null; then
      kill -KILL "$POSE_GRAPH_PID" 2>/dev/null || true
    fi
    wait "$POSE_GRAPH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

read_uint32_topic() {
  local topic="$1"
  python3 - "$topic" "$ECHO_TIMEOUT_S" <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt32

topic = sys.argv[1]
timeout_s = float(sys.argv[2])
value = None

rclpy.init()
node = Node("pose_graph_loop_smoke_reader")
qos = QoSProfile(depth=1)
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
qos.reliability = ReliabilityPolicy.RELIABLE

def on_msg(msg):
  global value
  value = int(msg.data)

node.create_subscription(UInt32, topic, on_msg, qos)
deadline = time.monotonic() + timeout_s
while rclpy.ok() and value is None and time.monotonic() < deadline:
  rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
if value is None:
  sys.exit(124)
print(value)
PY
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + STARTUP_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if ros2 topic list | grep -qx "$topic"; then
      return 0
    fi
    sleep 0.2
  done
  echo "Timed out waiting for $topic. Pose graph log: $POSE_GRAPH_LOG" >&2
  return 1
}

ros2 launch aqua_pose_graph pose_graph.launch.py >"$POSE_GRAPH_LOG" 2>&1 &
POSE_GRAPH_PID="$!"

wait_for_topic "/aqua_pose_graph/keyframe_count"

ros2 run aqua_localization pose_graph_loop_demo.py \
  --keyframes "$EXPECTED_KEYFRAMES" \
  --hold-s 1.0 \
  >"$DEMO_LOG" 2>&1

if ! keyframes="$(read_uint32_topic /aqua_pose_graph/keyframe_count)"; then
  keyframes=""
fi
if ! loops="$(read_uint32_topic /aqua_pose_graph/loop_constraint_count)"; then
  loops=""
fi

if [[ -z "$keyframes" || "$keyframes" -lt "$EXPECTED_KEYFRAMES" ]]; then
  echo "Expected at least $EXPECTED_KEYFRAMES keyframes, got '${keyframes:-none}'." >&2
  echo "Pose graph log: $POSE_GRAPH_LOG" >&2
  echo "Demo log: $DEMO_LOG" >&2
  exit 1
fi

if [[ -z "$loops" || "$loops" -lt "$EXPECTED_LOOPS" ]]; then
  echo "Expected at least $EXPECTED_LOOPS loop constraint, got '${loops:-none}'." >&2
  echo "Pose graph log: $POSE_GRAPH_LOG" >&2
  echo "Demo log: $DEMO_LOG" >&2
  exit 1
fi

echo "pose graph loop smoke passed: keyframes=$keyframes loops=$loops"
echo "logs: $LOG_DIR"
