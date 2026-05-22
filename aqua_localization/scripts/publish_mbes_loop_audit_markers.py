#!/usr/bin/env python3
"""Publish prioritized MBES loop audit markers for RViz inspection.

The script reads a results-included replay bag for `/aqua_pose_graph/keyframe`
poses and a loop-status CSV exported from `/mbes_loop_closure/status`. It then
publishes only accepted loop candidates as colored RViz markers on
`/mbes_loop_audit/markers`.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import audit_mbes_loop_candidates as audit


@dataclass(frozen=True)
class KeyframePose:
    keyframe_id: int
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class MarkerSpec:
    rank: int
    priority: str
    candidate_id: int
    current_id: int
    candidate_xyz: tuple[float, float, float]
    current_xyz: tuple[float, float, float]
    label_xyz: tuple[float, float, float]
    label: str
    flags: tuple[str, ...]


PRIORITY_COLORS = {
    "high": (1.0, 0.12, 0.08, 1.0),
    "medium": (1.0, 0.72, 0.05, 0.9),
    "low": (0.1, 0.82, 0.35, 0.8),
}


def default_ros2_typestore():
    try:
        from rosbags.typesys import Stores, get_typestore
    except ImportError as e:
        raise RuntimeError(
            "missing dependency: install rosbags type stores to read rosbag2 files"
        ) from e

    for store_name in ("ROS2_HUMBLE", "ROS2_JAZZY", "ROS2_IRON"):
        store = getattr(Stores, store_name, None)
        if store is not None:
            return get_typestore(store)
    raise RuntimeError("rosbags does not provide a supported ROS 2 typestore")


def open_reader_with_typestore_fallback(any_reader, bag_dir: Path):
    reader = any_reader([bag_dir])
    try:
        reader.open()
        return reader
    except Exception as e:
        if "no type definitions" not in str(e).lower():
            raise

    reader = any_reader([bag_dir], default_typestore=default_ros2_typestore())
    reader.open()
    return reader


def deserialize_keyframe(reader, raw, msgtype: str):
    try:
        return reader.deserialize(raw, msgtype)
    except Exception:
        if msgtype != "aqua_msgs/msg/PoseGraphKeyframe":
            raise

    try:
        from aqua_msgs.msg import PoseGraphKeyframe
        from rclpy.serialization import deserialize_message
    except ImportError as e:
        raise RuntimeError(
            "cannot deserialize aqua_msgs/msg/PoseGraphKeyframe; source the ROS 2 "
            "workspace or record bags with embedded type definitions"
        ) from e
    return deserialize_message(raw, PoseGraphKeyframe)


def read_keyframe_poses(bag: Path, topic: str) -> dict[int, KeyframePose]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        raise RuntimeError("missing dependency: install rosbags to read rosbag2 files") from e

    bag_dir = bag if bag.is_dir() else bag.parent
    if not bag_dir.is_dir():
        raise RuntimeError(f"not a rosbag2 directory: {bag_dir}")

    keyframes: dict[int, KeyframePose] = {}
    reader = open_reader_with_typestore_fallback(AnyReader, bag_dir)
    try:
        wanted = [connection for connection in reader.connections if connection.topic == topic]
        if not wanted:
            available = ", ".join(sorted({connection.topic for connection in reader.connections}))
            raise RuntimeError(f"topic {topic!r} not found. Available topics: {available}")
        for connection, _t_ns, raw in reader.messages(connections=wanted):
            msg = deserialize_keyframe(reader, raw, connection.msgtype)
            p = msg.pose.position
            keyframes[int(msg.id)] = KeyframePose(int(msg.id), float(p.x), float(p.y), float(p.z))
    finally:
        reader.close()
    return keyframes


def midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    z_offset: float,
) -> tuple[float, float, float]:
    return (
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2]) + z_offset,
    )


def build_marker_specs(
    audit_rows: Iterable[audit.AuditRow],
    keyframes: dict[int, KeyframePose],
    *,
    max_markers: int,
    label_z_offset: float,
) -> list[MarkerSpec]:
    specs: list[MarkerSpec] = []
    for rank, item in enumerate(audit_rows, start=1):
        if len(specs) >= max_markers:
            break
        row = item.row
        candidate = keyframes.get(row.candidate_id)
        current = keyframes.get(row.current_id)
        if candidate is None or current is None:
            continue
        candidate_xyz = (candidate.x, candidate.y, candidate.z)
        current_xyz = (current.x, current.y, current.z)
        label = (
            f"#{rank} {item.priority} {row.candidate_id}->{row.current_id} "
            f"fit={audit.format_float(row.fitness_score)} "
            f"dt={audit.format_float(row.correction_translation_m)}"
        )
        specs.append(
            MarkerSpec(
                rank=rank,
                priority=item.priority,
                candidate_id=row.candidate_id,
                current_id=row.current_id,
                candidate_xyz=candidate_xyz,
                current_xyz=current_xyz,
                label_xyz=midpoint(candidate_xyz, current_xyz, label_z_offset),
                label=label,
                flags=item.flags,
            )
        )
    return specs


def marker_scale(priority: str) -> float:
    if priority == "high":
        return 0.28
    if priority == "medium":
        return 0.18
    return 0.12


def apply_color(marker, priority: str) -> None:
    r, g, b, a = PRIORITY_COLORS.get(priority, PRIORITY_COLORS["medium"])
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a


def make_point(xyz: tuple[float, float, float]):
    from geometry_msgs.msg import Point

    point = Point()
    point.x, point.y, point.z = xyz
    return point


def make_marker_array(specs: list[MarkerSpec], frame_id: str):
    from visualization_msgs.msg import Marker, MarkerArray

    markers = MarkerArray()
    for idx, spec in enumerate(specs):
        line = Marker()
        line.header.frame_id = frame_id
        line.ns = f"mbes_loop_audit/{spec.priority}"
        line.id = idx * 2
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = marker_scale(spec.priority)
        apply_color(line, spec.priority)
        line.points.append(make_point(spec.candidate_xyz))
        line.points.append(make_point(spec.current_xyz))
        markers.markers.append(line)

        text = Marker()
        text.header.frame_id = frame_id
        text.ns = f"mbes_loop_audit/{spec.priority}_label"
        text.id = idx * 2 + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = make_point(spec.label_xyz)
        text.pose.orientation.w = 1.0
        text.scale.z = 1.2 if spec.priority == "high" else 0.85
        apply_color(text, spec.priority)
        text.text = spec.label
        markers.markers.append(text)
    return markers


def publish_markers(args: argparse.Namespace, specs: list[MarkerSpec]) -> int:
    try:
        import rclpy
        from visualization_msgs.msg import MarkerArray
    except ImportError as e:
        raise RuntimeError("missing ROS 2 Python dependencies; source the workspace") from e

    rclpy.init()
    node = rclpy.create_node("mbes_loop_audit_marker_publisher")
    publisher = node.create_publisher(MarkerArray, args.topic, 1)
    marker_array = make_marker_array(specs, args.frame)
    for _ in range(max(1, args.publish_count)):
        now = node.get_clock().now().to_msg()
        for marker in marker_array.markers:
            marker.header.stamp = now
        publisher.publish(marker_array)
        rclpy.spin_once(node, timeout_sec=args.period_s)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path,
                        help="Results-included rosbag2 directory")
    parser.add_argument("--csv", required=True, type=Path,
                        help="Loop-status CSV")
    parser.add_argument("--keyframe-topic", default="/aqua_pose_graph/keyframe")
    parser.add_argument("--topic", default="/mbes_loop_audit/markers",
                        help="MarkerArray topic to publish")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--max-markers", type=int, default=100)
    parser.add_argument("--label-z-offset", type=float, default=1.0)
    parser.add_argument("--publish-count", type=int, default=5)
    parser.add_argument("--period-s", type=float, default=0.5)

    parser.add_argument("--max-fitness", type=float, default=2.0)
    parser.add_argument("--max-translation-m", type=float, default=5.0)
    parser.add_argument("--max-rotation-rad", type=float, default=0.5)
    parser.add_argument("--min-keyframe-separation", type=int, default=20)
    parser.add_argument("--descriptor-extent-warn", type=float, default=5.0)
    parser.add_argument("--descriptor-point-ratio-warn", type=float, default=0.5)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        rows = audit.read_loop_status_csv(args.csv)
        keyframes = read_keyframe_poses(args.bag, args.keyframe_topic)
        audit_rows = audit.accepted_audit_rows(rows, args)
        specs = build_marker_specs(
            audit_rows,
            keyframes,
            max_markers=args.max_markers,
            label_z_offset=args.label_z_offset,
        )
        if not specs:
            print("no accepted loop markers could be built", file=sys.stderr)
            return 1
        print(
            f"publishing {len(specs)} accepted loop audit markers "
            f"on {args.topic} from {len(keyframes)} keyframes"
        )
        return publish_markers(args, specs)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
