#!/usr/bin/env python3
"""Export aqua_msgs/EstimatorStatus samples from rosbag2 to CSV.

The output columns intentionally match record_status.py so the resulting file
can be consumed directly by compare_mbes_repeat_probe.py.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys


CSV_FIELDS = [
    "timestamp",
    "frame_id",
    "estimator_name",
    "backend",
    "initialized",
    "update_count",
    "last_prediction_dt",
    "position_covariance_trace",
    "orientation_covariance_trace",
    "status",
    "accel_bias_x",
    "accel_bias_y",
    "accel_bias_z",
    "gyro_bias_x",
    "gyro_bias_y",
    "gyro_bias_z",
    "ahrs_gyro_bias_z_enabled",
    "ahrs_gyro_bias_z_active",
    "ahrs_gyro_bias_z_last_observed",
    "sonar_feedback_received",
    "sonar_feedback_applied",
    "sonar_feedback_skipped_stale",
    "sonar_feedback_skipped_nonfinite",
    "sonar_feedback_pending",
]


def bag_reader_path(path: Path) -> Path:
    return path.parent if path.suffix == ".mcap" else path


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def vector_component(values, index: int) -> float:
    try:
        return float(values[index])
    except (IndexError, TypeError):
        return 0.0


def status_row(msg, fallback_timestamp_ns: int) -> dict[str, object]:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    timestamp = stamp_to_seconds(stamp) if stamp is not None else 0.0
    if timestamp <= 0.0:
        timestamp = fallback_timestamp_ns * 1.0e-9
    accel_bias = getattr(msg, "accel_bias", ())
    gyro_bias = getattr(msg, "gyro_bias", ())
    return {
        "timestamp": f"{timestamp:.9f}",
        "frame_id": str(getattr(header, "frame_id", "")),
        "estimator_name": str(getattr(msg, "estimator_name", "")),
        "backend": str(getattr(msg, "backend", "")),
        "initialized": int(bool(getattr(msg, "initialized", False))),
        "update_count": int(getattr(msg, "update_count", 0)),
        "last_prediction_dt": f"{float(getattr(msg, 'last_prediction_dt', 0.0)):.9f}",
        "position_covariance_trace": (
            f"{float(getattr(msg, 'position_covariance_trace', 0.0)):.9f}"
        ),
        "orientation_covariance_trace": (
            f"{float(getattr(msg, 'orientation_covariance_trace', 0.0)):.9f}"
        ),
        "status": str(getattr(msg, "status", "")),
        "accel_bias_x": f"{vector_component(accel_bias, 0):.9f}",
        "accel_bias_y": f"{vector_component(accel_bias, 1):.9f}",
        "accel_bias_z": f"{vector_component(accel_bias, 2):.9f}",
        "gyro_bias_x": f"{vector_component(gyro_bias, 0):.9f}",
        "gyro_bias_y": f"{vector_component(gyro_bias, 1):.9f}",
        "gyro_bias_z": f"{vector_component(gyro_bias, 2):.9f}",
        "ahrs_gyro_bias_z_enabled": int(
            bool(getattr(msg, "ahrs_gyro_bias_z_enabled", False))
        ),
        "ahrs_gyro_bias_z_active": int(
            bool(getattr(msg, "ahrs_gyro_bias_z_active", False))
        ),
        "ahrs_gyro_bias_z_last_observed": (
            f"{float(getattr(msg, 'ahrs_gyro_bias_z_last_observed', 0.0)):.9f}"
        ),
        "sonar_feedback_received": int(
            getattr(msg, "sonar_feedback_received", 0)
        ),
        "sonar_feedback_applied": int(getattr(msg, "sonar_feedback_applied", 0)),
        "sonar_feedback_skipped_stale": int(
            getattr(msg, "sonar_feedback_skipped_stale", 0)
        ),
        "sonar_feedback_skipped_nonfinite": int(
            getattr(msg, "sonar_feedback_skipped_nonfinite", 0)
        ),
        "sonar_feedback_pending": int(getattr(msg, "sonar_feedback_pending", 0)),
    }


def default_ros2_typestore():
    try:
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError("missing dependency: install rosbags") from exc
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
    except Exception as exc:
        if "no type definitions" not in str(exc).lower():
            raise
    reader = any_reader([bag_dir], default_typestore=default_ros2_typestore())
    reader.open()
    return reader


def deserialize_status_message(reader, raw, msgtype: str):
    try:
        return reader.deserialize(raw, msgtype)
    except Exception:
        if msgtype != "aqua_msgs/msg/EstimatorStatus":
            raise
    try:
        from aqua_msgs.msg import EstimatorStatus
        from rclpy.serialization import deserialize_message
    except ImportError as exc:
        raise RuntimeError(
            "cannot deserialize aqua_msgs/msg/EstimatorStatus; source the ROS 2 "
            "workspace or use a bag with embedded type definitions"
        ) from exc
    return deserialize_message(raw, EstimatorStatus)


def export_status_topic(
    bag: Path,
    topic: str,
    out: Path,
    *,
    allow_empty: bool = False,
) -> int:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError("missing dependency: install rosbags") from exc

    bag_dir = bag_reader_path(bag)
    if not bag_dir.is_dir():
        raise ValueError(f"not a rosbag2 directory: {bag_dir}")

    reader = open_reader_with_typestore_fallback(AnyReader, bag_dir)
    count = 0
    try:
        connections = [
            connection for connection in reader.connections
            if connection.topic == topic
        ]
        if not connections:
            available = ", ".join(
                sorted({connection.topic for connection in reader.connections})[:20]
            )
            raise ValueError(
                f"topic not found: {topic}; available topics include: {available}"
            )
        invalid_types = sorted(
            {
                connection.msgtype
                for connection in connections
                if connection.msgtype != "aqua_msgs/msg/EstimatorStatus"
            }
        )
        if invalid_types:
            raise ValueError(
                f"topic {topic} is not aqua_msgs/msg/EstimatorStatus: "
                f"{', '.join(invalid_types)}"
            )

        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", encoding="utf-8", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
            writer.writeheader()
            for connection, timestamp_ns, raw in reader.messages(
                connections=connections
            ):
                msg = deserialize_status_message(
                    reader, raw, connection.msgtype
                )
                writer.writerow(status_row(msg, timestamp_ns))
                count += 1
    finally:
        reader.close()

    if count == 0 and not allow_empty:
        raise ValueError(f"topic {topic} had no messages")
    return count


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag", required=True, type=Path, help="rosbag2 directory or MCAP file"
    )
    parser.add_argument(
        "--topic",
        default="/aqua_imu_loc/status",
        help="EstimatorStatus topic (default: /aqua_imu_loc/status)",
    )
    parser.add_argument("--out", required=True, type=Path, help="output CSV path")
    parser.add_argument("--allow-empty", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        count = export_status_topic(
            args.bag,
            args.topic,
            args.out,
            allow_empty=args.allow_empty,
        )
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"failed to export estimator status: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {count} {args.topic} samples to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

