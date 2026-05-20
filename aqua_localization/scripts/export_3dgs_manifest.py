#!/usr/bin/env python3
"""Export a lightweight underwater 3DGS input manifest from rosbag2 metadata."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - ROS environments normally include PyYAML.
    yaml = None

try:
    import inspect_bag_topics
except ImportError:  # pragma: no cover - allows importing from tests with spec loaders.
    inspect_bag_topics = None


SCHEMA = "aqua_localization.underwater_3dgs_manifest.v1"

ROLE_DEFINITIONS = {
    "image": {
        "label": "camera image",
        "required": True,
        "types": {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"},
    },
    "camera_info": {
        "label": "camera intrinsics",
        "required": True,
        "types": {"sensor_msgs/msg/CameraInfo"},
    },
    "trajectory": {
        "label": "estimated trajectory",
        "required": True,
        "types": {"nav_msgs/msg/Odometry", "nav_msgs/msg/Path"},
    },
    "depth_pressure": {
        "label": "depth or pressure prior",
        "required": False,
        "types": {
            "sensor_msgs/msg/FluidPressure",
            "std_msgs/msg/Float32",
            "std_msgs/msg/Float64",
            "nav_msgs/msg/Odometry",
        },
    },
    "mbes": {
        "label": "MBES/submap prior",
        "required": False,
        "types": {"sensor_msgs/msg/PointCloud2"},
    },
}


def metadata_file_for(path: Path) -> Path:
    path = path.expanduser()
    return path if path.name == "metadata.yaml" else path / "metadata.yaml"


def ns_from_value(value):
    if value is None:
        return None
    if isinstance(value, dict):
        for key in ("nanoseconds", "nanoseconds_since_epoch"):
            if key in value:
                return ns_from_value(value[key])
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def seconds_from_ns(value):
    return None if value is None else value / 1_000_000_000.0


def topic_from_entry(entry):
    metadata = entry.get("topic_metadata") or {}
    name = metadata.get("name")
    msg_type = metadata.get("type")
    if not name or not msg_type:
        return None
    topic = {
        "name": str(name),
        "type": str(msg_type),
        "message_count": entry.get("message_count"),
    }
    try:
        topic["message_count"] = int(topic["message_count"])
    except (TypeError, ValueError):
        topic["message_count"] = None
    return topic


def parse_metadata_with_yaml(metadata_path: Path):
    raw = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    info = raw.get("rosbag2_bagfile_information", raw)
    topics = []
    for entry in info.get("topics_with_message_count", []) or []:
        topic = topic_from_entry(entry)
        if topic:
            topics.append(topic)

    duration_ns = ns_from_value(info.get("duration"))
    start_ns = ns_from_value(info.get("starting_time"))
    return {
        "storage_identifier": info.get("storage_identifier"),
        "duration_s": seconds_from_ns(duration_ns),
        "start_time_ns": start_ns,
        "end_time_ns": start_ns + duration_ns if start_ns is not None and duration_ns is not None else None,
        "topics": sorted(topics, key=lambda item: item["name"]),
    }


def parse_metadata_fallback(metadata_path: Path):
    if inspect_bag_topics is None:
        return {
            "storage_identifier": None,
            "duration_s": None,
            "start_time_ns": None,
            "end_time_ns": None,
            "topics": [],
        }
    topics = inspect_bag_topics.parse_metadata(metadata_path)
    return {
        "storage_identifier": None,
        "duration_s": None,
        "start_time_ns": None,
        "end_time_ns": None,
        "topics": [
            {"name": topic["name"], "type": topic["type"], "message_count": None}
            for topic in topics
        ],
    }


def parse_metadata(metadata_path: Path):
    if yaml is not None:
        return parse_metadata_with_yaml(metadata_path)
    return parse_metadata_fallback(metadata_path)


def name_has(topic, tokens):
    name = topic["name"].lower()
    return any(token in name for token in tokens)


def score_role(topic, role: str) -> int:
    name = topic["name"].lower()
    msg_type = topic["type"]
    score = 0

    if role == "image" and msg_type in ROLE_DEFINITIONS[role]["types"]:
        score += 100
        if name_has(topic, ("image", "camera", "left", "right", "rgb", "rect")):
            score += 35
        if "compressed" in msg_type.lower() or "compressed" in name:
            score -= 8
        if name_has(topic, ("debug", "preview", "thumbnail")):
            score -= 30
    elif role == "camera_info" and msg_type == "sensor_msgs/msg/CameraInfo":
        score += 100
        if name_has(topic, ("camera", "left", "right", "rgb")):
            score += 30
    elif role == "trajectory" and msg_type in ROLE_DEFINITIONS[role]["types"]:
        score += 80 if msg_type == "nav_msgs/msg/Odometry" else 55
        if "/aqua_fusion/odometry" in name:
            score += 60
        if "/aqua_imu_loc/odometry" in name:
            score += 50
        if "/aqua_visual_frontend/odometry" in name:
            score += 45
        if name_has(topic, ("odom", "trajectory", "path", "pose")):
            score += 20
        if name_has(topic, ("gt", "ground_truth", "apriltag")):
            score -= 25
    elif role == "depth_pressure" and msg_type in ROLE_DEFINITIONS[role]["types"]:
        if msg_type == "sensor_msgs/msg/FluidPressure":
            score += 100
        elif msg_type in {"std_msgs/msg/Float32", "std_msgs/msg/Float64"}:
            if name_has(topic, ("pressure", "bar30", "barometer", "depth")):
                score += 70
        elif msg_type == "nav_msgs/msg/Odometry":
            if name_has(topic, ("depth", "pressure")):
                score += 45
        if name_has(topic, ("pressure", "bar30", "barometer", "depth")):
            score += 45
        if name_has(topic, ("image", "camera")):
            score -= 60
    elif role == "mbes" and msg_type == "sensor_msgs/msg/PointCloud2":
        score += 80
        if name_has(topic, ("mbes", "multibeam", "norbit", "sonar", "detections", "submap")):
            score += 50
        if name_has(topic, ("points", "cloud")):
            score += 15

    return score


def ranked_candidates(topics, role: str, limit: int = 5):
    scored = []
    for topic in topics:
        score = score_role(topic, role)
        if score > 0:
            scored.append((score, topic))
    scored.sort(key=lambda item: (-item[0], item[1]["name"]))
    return [
        {
            "topic": topic["name"],
            "type": topic["type"],
            "message_count": topic.get("message_count"),
            "score": score,
        }
        for score, topic in scored[:limit]
    ]


def find_topic(topics, topic_name):
    return next((topic for topic in topics if topic["name"] == topic_name), None)


def role_entry(role: str, topics, override: str | None):
    definition = ROLE_DEFINITIONS[role]
    source = "override" if override else "auto"
    selected = find_topic(topics, override) if override else None
    if selected is None and not override:
        candidates = ranked_candidates(topics, role, limit=1)
        selected = find_topic(topics, candidates[0]["topic"]) if candidates else None

    if selected is None:
        return {
            "topic": override,
            "type": None,
            "message_count": None,
            "required": definition["required"],
            "status": "missing",
            "source": source,
            "candidates": ranked_candidates(topics, role),
        }

    expected = selected["type"] in definition["types"]
    return {
        "topic": selected["name"],
        "type": selected["type"],
        "message_count": selected.get("message_count"),
        "required": definition["required"],
        "status": "found" if expected else "type_mismatch",
        "source": source,
        "candidates": ranked_candidates(topics, role),
    }


def build_roles(topics, args):
    overrides = {
        "image": args.image_topic,
        "camera_info": args.camera_info_topic,
        "trajectory": args.trajectory_topic,
        "depth_pressure": args.depth_topic,
        "mbes": args.mbes_topic,
    }
    return {
        role: role_entry(role, topics, overrides[role])
        for role in ROLE_DEFINITIONS
    }


def build_warnings(roles):
    warnings = []
    for role, entry in roles.items():
        label = ROLE_DEFINITIONS[role]["label"]
        if entry["status"] == "missing":
            if entry["required"]:
                warnings.append(f"missing required {label} topic")
            else:
                warnings.append(f"missing optional {label} topic")
        elif entry["status"] == "type_mismatch":
            warnings.append(
                f"{label} topic {entry['topic']} has unexpected type {entry['type']}"
            )
    if roles["image"]["status"] == "found" and roles["image"]["message_count"] == 0:
        warnings.append("camera image topic has zero messages")
    return warnings


def build_manifest(args):
    metadata_path = metadata_file_for(args.bag)
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yaml not found: {metadata_path}")

    metadata = parse_metadata(metadata_path)
    topics = metadata["topics"]
    roles = build_roles(topics, args)
    warnings = build_warnings(roles)

    return {
        "schema": SCHEMA,
        "dataset": args.dataset,
        "sequence": args.sequence,
        "bag": {
            "path": str(args.bag),
            "metadata": str(metadata_path),
            "storage_identifier": metadata["storage_identifier"],
            "duration_s": metadata["duration_s"],
            "start_time_ns": metadata["start_time_ns"],
            "end_time_ns": metadata["end_time_ns"],
        },
        "roles": roles,
        "topics": topics,
        "warnings": warnings,
        "notes": [
            "This manifest describes candidate inputs for an underwater 3DGS export pipeline.",
            "It does not train or evaluate a 3D Gaussian Splatting model.",
        ],
    }


def write_manifest(path: Path, manifest):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Inspect rosbag2 metadata and emit a JSON manifest for underwater 3DGS experiments."
    )
    parser.add_argument("--bag", required=True, type=Path, help="Path to a rosbag2 directory or metadata.yaml.")
    parser.add_argument("--dataset", required=True, help="Dataset name, e.g. Tank Dataset.")
    parser.add_argument("--sequence", required=True, help="Sequence name, e.g. short_test.")
    parser.add_argument("--out", type=Path, help="Optional JSON output path.")
    parser.add_argument("--image-topic", help="Override camera image topic.")
    parser.add_argument("--camera-info-topic", help="Override CameraInfo topic.")
    parser.add_argument("--trajectory-topic", help="Override estimated trajectory topic.")
    parser.add_argument("--depth-topic", help="Override depth or pressure topic.")
    parser.add_argument("--mbes-topic", help="Override optional MBES/submap PointCloud2 topic.")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return a non-zero exit code when required topics are missing.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        manifest = build_manifest(args)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if args.out:
        write_manifest(args.out, manifest)
    else:
        print(json.dumps(manifest, indent=2, sort_keys=True))

    if args.strict and any(
        entry["required"] and entry["status"] != "found"
        for entry in manifest["roles"].values()
    ):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
