#!/usr/bin/env python3
"""Build 3DGS transforms.json by matching extracted frames to odometry."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import sys

import numpy as np


TRANSFORMS_SCHEMA = "aqua_localization.underwater_3dgs_transforms.v1"


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"file not found: {path}")
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON: {path}: {exc}") from exc


def write_json(path: Path, payload):
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def bag_path_from_manifest(manifest):
    bag_path = manifest.get("bag", {}).get("path")
    if not bag_path:
        raise ValueError("manifest has no bag.path")
    path = Path(bag_path).expanduser()
    return path.parent if path.name == "metadata.yaml" else path


def trajectory_role(manifest):
    role = manifest.get("roles", {}).get("trajectory", {})
    if role.get("status") != "found" or not role.get("topic"):
        raise ValueError("manifest has no found trajectory topic")
    msg_type = role.get("type")
    if msg_type != "nav_msgs/msg/Odometry":
        raise ValueError(f"unsupported trajectory topic type: {msg_type}")
    return role


def stamp_to_ns(stamp):
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nanosec)


def message_stamp_ns(msg):
    return stamp_to_ns(getattr(getattr(msg, "header", None), "stamp", None))


def read_rosbag_odometry(bag_path: Path, topic: str, storage_id: str | None = None):
    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python bag dependencies are unavailable. Source your ROS 2 workspace "
            "or run this command inside a ROS 2 environment."
        ) from exc

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id or "")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    while reader.has_next():
        current_topic, data, timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        yield timestamp_ns, deserialize_message(data, Odometry)


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def odometry_to_matrix(msg):
    pose = msg.pose.pose
    p = pose.position
    q = pose.orientation
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = quaternion_to_matrix(float(q.x), float(q.y), float(q.z), float(q.w))
    matrix[:3, 3] = [float(p.x), float(p.y), float(p.z)]
    return matrix


def odometry_sample(timestamp_ns: int, msg):
    msg_stamp = message_stamp_ns(msg)
    match_timestamp = msg_stamp if msg_stamp is not None else int(timestamp_ns)
    return {
        "timestamp_ns": int(timestamp_ns),
        "message_stamp_ns": msg_stamp,
        "match_timestamp_ns": int(match_timestamp),
        "transform_matrix": odometry_to_matrix(msg).tolist(),
    }


def load_frames(pack_dir: Path):
    frames_path = pack_dir / "frames.json"
    if not frames_path.is_file():
        raise FileNotFoundError(f"frames.json not found: {frames_path}")
    payload = load_json(frames_path)
    frames = payload.get("frames", [])
    if not isinstance(frames, list):
        raise ValueError("frames.json has no frames list")
    return payload


def load_pack_index(pack_dir: Path):
    path = pack_dir / "pack_index.json"
    if not path.is_file():
        raise FileNotFoundError(f"pack_index.json not found: {path}")
    return load_json(path)


def frame_match_time(frame):
    value = frame.get("message_stamp_ns")
    if value is None:
        value = frame.get("timestamp_ns")
    if value is None:
        raise ValueError(f"frame has no timestamp: {frame}")
    return int(value)


def nearest_sample(samples, timestamp_ns: int):
    if not samples:
        return None, None
    times = [sample["match_timestamp_ns"] for sample in samples]
    index = bisect_left(times, timestamp_ns)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    best = min(candidates, key=lambda sample: abs(sample["match_timestamp_ns"] - timestamp_ns))
    return best, abs(best["match_timestamp_ns"] - timestamp_ns)


def collect_odometry_samples(reader):
    samples = [odometry_sample(timestamp_ns, msg) for timestamp_ns, msg in reader]
    samples.sort(key=lambda sample: sample["match_timestamp_ns"])
    return samples


def update_pack_index(pack_dir: Path, pack_index, transforms_payload):
    paths = pack_index.setdefault("paths", {})
    paths["transforms"] = "transforms.json"
    paths["frames"] = "frames.json"
    pack_index["status"] = "transforms_estimated"
    pack_index["estimated_transforms"] = {
        "schema": transforms_payload["schema"],
        "count": transforms_payload["frame_count"],
        "trajectory_topic": transforms_payload["trajectory_topic"],
        "trajectory_type": transforms_payload["trajectory_type"],
        "max_time_diff_ns": transforms_payload["max_time_diff_ns"],
        "skipped_count": transforms_payload["skipped_count"],
    }
    write_json(pack_dir / "pack_index.json", pack_index)


def build_transforms(
    manifest_path: Path,
    pack_dir: Path,
    max_time_diff_s: float = 0.05,
    reader=None,
):
    if max_time_diff_s < 0.0:
        raise ValueError("max_time_diff must be >= 0")

    manifest = load_json(manifest_path)
    role = trajectory_role(manifest)
    topic = role["topic"]
    msg_type = role["type"]
    frames_payload = load_frames(pack_dir)
    pack_index = load_pack_index(pack_dir)
    max_time_diff_ns = int(max_time_diff_s * 1_000_000_000)

    if reader is None:
        storage_id = manifest.get("bag", {}).get("storage_identifier")
        reader = read_rosbag_odometry(bag_path_from_manifest(manifest), topic, storage_id)

    odom_samples = collect_odometry_samples(reader)
    if not odom_samples:
        raise ValueError(f"no odometry samples found on {topic}")

    matched_frames = []
    skipped = []
    for frame in frames_payload.get("frames", []):
        frame_time = frame_match_time(frame)
        odom, diff_ns = nearest_sample(odom_samples, frame_time)
        if odom is None or diff_ns is None:
            skipped.append({"frame_index": frame.get("index"), "reason": "no_odometry"})
            continue
        if diff_ns > max_time_diff_ns:
            skipped.append(
                {
                    "frame_index": frame.get("index"),
                    "file_path": frame.get("file_path"),
                    "time_diff_ns": int(diff_ns),
                    "reason": "time_diff_exceeded",
                }
            )
            continue
        matched_frames.append(
            {
                "file_path": frame.get("file_path"),
                "timestamp_ns": frame.get("timestamp_ns"),
                "frame_match_timestamp_ns": frame_time,
                "odom_timestamp_ns": odom["match_timestamp_ns"],
                "odom_bag_timestamp_ns": odom["timestamp_ns"],
                "odom_message_stamp_ns": odom["message_stamp_ns"],
                "time_diff_ns": int(diff_ns),
                "transform_matrix": odom["transform_matrix"],
            }
        )

    payload = {
        "schema": TRANSFORMS_SCHEMA,
        "dataset": manifest.get("dataset"),
        "sequence": manifest.get("sequence"),
        "trajectory_topic": topic,
        "trajectory_type": msg_type,
        "camera_pose_convention": "raw_ros_odometry_pose_as_camera_to_world",
        "max_time_diff_ns": max_time_diff_ns,
        "source_frame_count": len(frames_payload.get("frames", [])),
        "odometry_sample_count": len(odom_samples),
        "frame_count": len(matched_frames),
        "skipped_count": len(skipped),
        "frames": matched_frames,
        "skipped_frames": skipped,
        "notes": [
            "Transforms are nearest-neighbour matches between extracted image timestamps and odometry.",
            "Camera intrinsics and camera-to-base extrinsics are not applied in this exporter.",
        ],
    }
    write_json(pack_dir / "transforms.json", payload)
    update_pack_index(pack_dir, pack_index, payload)
    return payload


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Match extracted 3DGS image frames to nav_msgs/Odometry and write transforms.json."
    )
    parser.add_argument("--manifest", required=True, type=Path, help="Input 3DGS manifest JSON.")
    parser.add_argument("--pack", required=True, type=Path, help="Dataset pack directory with frames.json.")
    parser.add_argument(
        "--max-time-diff",
        type=float,
        default=0.05,
        help="Maximum frame-to-odometry timestamp difference in seconds.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        payload = build_transforms(args.manifest, args.pack, args.max_time_diff)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
