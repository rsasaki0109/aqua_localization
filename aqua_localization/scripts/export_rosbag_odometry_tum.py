#!/usr/bin/env python3
"""Export a nav_msgs/Odometry topic from a rosbag to TUM trajectory format."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def bag_reader_path(path: Path) -> Path:
    if path.suffix == ".mcap":
        return path.parent
    return path


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def odometry_to_tum_line(msg, fallback_timestamp_ns: int) -> str:
    t = stamp_to_seconds(msg.header.stamp) if hasattr(msg, "header") else fallback_timestamp_ns * 1.0e-9
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (
        f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
        f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
    )


def export_odometry_topic(
    bag: Path,
    topic: str,
    out: Path,
    *,
    allow_empty: bool = False,
) -> int:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError(f"missing dependency: {exc}. Install rosbags.") from exc

    bag_dir = bag_reader_path(bag)
    if not bag_dir.is_dir():
        raise ValueError(f"not a rosbag directory: {bag_dir}")

    out.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with AnyReader([bag_dir]) as reader, out.open("w", encoding="utf-8") as fp:
        connections = [conn for conn in reader.connections if conn.topic == topic]
        if not connections:
            available = ", ".join(sorted({conn.topic for conn in reader.connections})[:12])
            raise ValueError(f"topic not found: {topic}; available topics include: {available}")
        invalid_types = sorted({conn.msgtype for conn in connections if conn.msgtype != "nav_msgs/msg/Odometry"})
        if invalid_types:
            raise ValueError(f"topic {topic} is not nav_msgs/msg/Odometry: {', '.join(invalid_types)}")

        for connection, timestamp_ns, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            fp.write(odometry_to_tum_line(msg, timestamp_ns))
            count += 1

    if count == 0 and not allow_empty:
        raise ValueError(f"topic {topic} had no messages")
    return count


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="rosbag2 directory or MCAP file.")
    parser.add_argument("--topic", required=True, help="nav_msgs/Odometry topic to export.")
    parser.add_argument("--out", required=True, type=Path, help="Output TUM trajectory path.")
    parser.add_argument("--allow-empty", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        count = export_odometry_topic(args.bag, args.topic, args.out, allow_empty=args.allow_empty)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"failed to export odometry TUM: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {count} {args.topic} samples to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
