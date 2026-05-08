#!/usr/bin/env python3
"""Inspect rosbag2 metadata and suggest aqua_localization replay arguments."""

import argparse
import re
import shlex
import sys
from pathlib import Path


TYPE_PRIORITY = {
    "sensor_msgs/msg/Imu": "imu",
    "sensor_msgs/msg/FluidPressure": "pressure",
    "sensor_msgs/msg/PointCloud2": "sonar_points",
    "geometry_msgs/msg/TwistStamped": "current_velocity",
    "std_msgs/msg/Float64": "scalar",
    "std_msgs/msg/Float32": "scalar",
}


def normalize_scalar(value):
    value = value.strip()
    if value.startswith(("'", '"')) and value.endswith(("'", '"')):
        return value[1:-1]
    return value


def extract_inline_value(line, key):
    match = re.search(rf"\b{re.escape(key)}\s*:\s*([^,}}]+)", line)
    if not match:
        return None
    return normalize_scalar(match.group(1))


def parse_metadata(metadata_path):
    topics = []
    current = {}
    in_topic_metadata = False

    for raw_line in metadata_path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        if "topic_metadata:" in line:
            if current.get("name") and current.get("type"):
                topics.append(current)
            current = {}
            in_topic_metadata = True
            inline_name = extract_inline_value(line, "name")
            inline_type = extract_inline_value(line, "type")
            if inline_name:
                current["name"] = inline_name
            if inline_type:
                current["type"] = inline_type
            continue

        if not in_topic_metadata:
            continue

        if line.startswith("name:"):
            current["name"] = normalize_scalar(line.split(":", 1)[1])
        elif line.startswith("type:"):
            current["type"] = normalize_scalar(line.split(":", 1)[1])
        elif line.startswith("- ") and current.get("name") and current.get("type"):
            topics.append(current)
            current = {}

    if current.get("name") and current.get("type"):
        topics.append(current)

    deduped = {}
    for topic in topics:
        deduped[(topic["name"], topic["type"])] = topic
    return sorted(deduped.values(), key=lambda item: item["name"])


def score_topic(topic, role):
    name = topic["name"].lower()
    msg_type = topic["type"]
    score = 0

    if role == "imu" and msg_type == "sensor_msgs/msg/Imu":
        score += 100
        if "imu" in name:
            score += 30
        if "mavros" in name:
            score += 10
    elif role == "pressure" and msg_type == "sensor_msgs/msg/FluidPressure":
        score += 100
        if "pressure" in name or "bar30" in name:
            score += 30
    elif role == "depth" and msg_type in {"std_msgs/msg/Float64", "std_msgs/msg/Float32"}:
        if "depth" in name:
            score += 100
        if "pressure" in name:
            score -= 50
    elif role == "scalar_pressure" and msg_type in {"std_msgs/msg/Float64", "std_msgs/msg/Float32"}:
        if any(token in name for token in ("barometer", "baro", "pressure")):
            score += 90
        if "depth" in name:
            score -= 40
    elif role == "sonar_points" and msg_type == "sensor_msgs/msg/PointCloud2":
        score += 80
        if any(token in name for token in ("sonar", "fls", "multibeam")):
            score += 35
        if any(token in name for token in ("points", "cloud")):
            score += 20
    elif role == "current_velocity" and msg_type == "geometry_msgs/msg/TwistStamped":
        score += 60
        if any(token in name for token in ("current", "water")):
            score += 40

    return score


def choose_topic(topics, role):
    scored = [(score_topic(topic, role), topic) for topic in topics]
    scored = [item for item in scored if item[0] > 0]
    if not scored:
        return None
    return sorted(scored, key=lambda item: (-item[0], item[1]["name"]))[0][1]


def detect_profile(topics):
    names = " ".join(topic["name"].lower() for topic in topics)
    if any(token in names for token in ("/rexrov", "uuv")):
        return "uuv_simulator"
    if any(token in names for token in ("/mavros", "bar30", "/fls")):
        return "bluerov2"
    return "default"


def profile_args(profile):
    if profile == "bluerov2":
        return [
            "imu_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/bluerov2.yaml",
            "sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/bluerov2.yaml",
            "fusion_params_file:=$(ros2 pkg prefix aqua_fusion)/share/aqua_fusion/config/bluerov2.yaml",
        ]
    if profile == "uuv_simulator":
        return [
            "imu_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/uuv_simulator.yaml",
            "sonar_params_file:=$(ros2 pkg prefix aqua_sonar_loc)/share/aqua_sonar_loc/config/uuv_simulator.yaml",
            "fusion_params_file:=$(ros2 pkg prefix aqua_fusion)/share/aqua_fusion/config/uuv_simulator.yaml",
        ]
    return []


def build_launch_args(bag_path, topics, profile):
    imu = choose_topic(topics, "imu")
    pressure = choose_topic(topics, "pressure")
    scalar_pressure = choose_topic(topics, "scalar_pressure")
    depth = choose_topic(topics, "depth")
    sonar = choose_topic(topics, "sonar_points")
    current = choose_topic(topics, "current_velocity")

    args = [
        "ros2",
        "launch",
        "aqua_localization",
        "replay.launch.py",
        "start_bag:=true",
        f"bag_path:={shlex.quote(str(bag_path))}",
    ]

    args.extend(profile_args(profile))

    if imu:
        args.append(f"bag_imu_topic:={imu['name']}")
    if pressure:
        args.append(f"bag_pressure_topic:={pressure['name']}")
    elif scalar_pressure:
        scalar_config = "scalar_to_pressure_ntnu.yaml"
        if "pressure" in scalar_pressure["name"].lower():
            scalar_config = "scalar_to_pressure.yaml"
        args.extend(
            [
                "enable_scalar_to_pressure:=true",
                f"bag_scalar_pressure_topic:={scalar_pressure['name']}",
                f"scalar_to_pressure_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/{scalar_config}",
            ]
        )
    elif depth:
        args.extend(
            [
                "enable_depth_to_pressure:=true",
                f"bag_depth_topic:={depth['name']}",
                "depth_to_pressure_params_file:=$(ros2 pkg prefix aqua_imu_loc)/share/aqua_imu_loc/config/depth_to_pressure_uuv_simulator.yaml",
            ]
        )
    else:
        args.append("enable_imu_loc:=false")

    if sonar:
        args.append(f"bag_sonar_points_topic:={sonar['name']}")
    else:
        args.extend(["enable_sonar_loc:=false", "enable_fusion:=false"])

    if current:
        args.append(f"current_velocity_topic:={current['name']}")

    return args, {
        "imu": imu,
        "pressure": pressure,
        "scalar_pressure": scalar_pressure,
        "depth": depth,
        "sonar_points": sonar,
        "current_velocity": current,
    }


def print_topic_table(topics):
    print("Detected topics:")
    for topic in topics:
        print(f"  {topic['name']} [{topic['type']}]")


def print_selection(selection, profile):
    print("\nSuggested role mapping:")
    print(f"  profile: {profile}")
    for role in ("imu", "pressure", "scalar_pressure", "depth", "sonar_points", "current_velocity"):
        topic = selection[role]
        if topic:
            print(f"  {role}: {topic['name']} [{topic['type']}]")
        else:
            print(f"  {role}: <not found>")


def main():
    parser = argparse.ArgumentParser(
        description="Inspect rosbag2 metadata.yaml and suggest aqua_localization replay.launch.py arguments."
    )
    parser.add_argument("bag_path", help="Path to a rosbag2 directory or metadata.yaml file.")
    parser.add_argument(
        "--profile",
        choices=["auto", "default", "bluerov2", "uuv_simulator"],
        default="auto",
        help="Vehicle profile arguments to include in the generated command.",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag_path).expanduser()
    metadata_path = bag_path if bag_path.name == "metadata.yaml" else bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        print(f"metadata.yaml not found: {metadata_path}", file=sys.stderr)
        return 2

    topics = parse_metadata(metadata_path)
    if not topics:
        print(f"No topics found in {metadata_path}", file=sys.stderr)
        return 1

    profile = detect_profile(topics) if args.profile == "auto" else args.profile
    launch_args, selection = build_launch_args(bag_path, topics, profile)

    print_topic_table(topics)
    print_selection(selection, profile)
    print("\nSuggested command:")
    print("  " + " \\\n  ".join(launch_args))
    return 0


if __name__ == "__main__":
    sys.exit(main())
