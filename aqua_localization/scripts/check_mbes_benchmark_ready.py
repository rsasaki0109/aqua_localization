#!/usr/bin/env python3
"""Check whether an MBES-SLAM bag is ready for loop-status benchmarking."""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path
import sys


DEFAULT_MIN_DURATION_S = 1.0

REQUIRED_ROLES = [
    {
        "role": "MBES points",
        "topics": ("/norbit/detections",),
        "types": ("sensor_msgs/msg/PointCloud2",),
    },
    {
        "role": "reference odometry",
        "topics": ("/nav/processed/odometry",),
        "types": ("nav_msgs/msg/Odometry",),
    },
    {
        "role": "IMU",
        "topics": (
            "/nav/processed/microstrain/imu/madgwick",
            "/nav/sensors/microstrain/imu/raw",
        ),
        "types": ("sensor_msgs/msg/Imu",),
        "allow_type_fallback": True,
    },
]


@dataclass(frozen=True)
class TopicInfo:
    name: str
    msg_type: str
    count: int


@dataclass(frozen=True)
class BagMetadata:
    path: Path
    duration_s: float | None
    message_count: int | None
    topics: tuple[TopicInfo, ...]


@dataclass(frozen=True)
class RoleCheck:
    role: str
    required: tuple[str, ...]
    found: TopicInfo | None
    passed: bool
    reason: str


def normalize_scalar(value: str) -> str:
    value = value.strip().rstrip(",")
    if value.startswith(("'", '"')) and value.endswith(("'", '"')):
        return value[1:-1]
    return value


def extract_inline_value(line: str, key: str) -> str | None:
    match = re.search(rf"\b{re.escape(key)}\s*:\s*([^,}}]+)", line)
    if not match:
        return None
    return normalize_scalar(match.group(1))


def parse_int_value(line: str, key: str) -> int | None:
    value = extract_inline_value(line, key)
    if value is None:
        return None
    try:
        return int(value)
    except ValueError:
        return None


def parse_metadata(metadata_path: Path) -> BagMetadata:
    topics: list[TopicInfo] = []
    current: dict[str, str | int] = {}
    in_topics = False
    in_topic_metadata = False
    duration_ns: int | None = None
    total_messages: int | None = None

    def flush_current() -> None:
        if current.get("name") and current.get("type"):
            topics.append(
                TopicInfo(
                    name=str(current["name"]),
                    msg_type=str(current["type"]),
                    count=int(current.get("count", 0)),
                )
            )
        current.clear()

    for raw_line in metadata_path.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        if not stripped or stripped.startswith("#"):
            continue

        if stripped == "topics_with_message_count:":
            in_topics = True
            in_topic_metadata = False
            continue

        if not in_topics:
            if stripped.startswith("nanoseconds:") and duration_ns is None:
                duration_ns = parse_int_value(stripped, "nanoseconds")
            elif indent == 2 and stripped.startswith("message_count:"):
                total_messages = parse_int_value(stripped, "message_count")
            continue

        if stripped.startswith("- message_count:"):
            flush_current()
            current["count"] = parse_int_value(stripped, "message_count") or 0
            in_topic_metadata = False
            continue

        if "topic_metadata:" in stripped:
            in_topic_metadata = True
            inline_name = extract_inline_value(stripped, "name")
            inline_type = extract_inline_value(stripped, "type")
            if inline_name:
                current["name"] = inline_name
            if inline_type:
                current["type"] = inline_type
            continue

        if not in_topic_metadata:
            continue

        if stripped.startswith("name:"):
            current["name"] = normalize_scalar(stripped.split(":", 1)[1])
        elif stripped.startswith("type:"):
            current["type"] = normalize_scalar(stripped.split(":", 1)[1])

    flush_current()
    deduped = {
        (topic.name, topic.msg_type): topic
        for topic in topics
    }
    return BagMetadata(
        path=metadata_path,
        duration_s=(duration_ns / 1.0e9) if duration_ns is not None else None,
        message_count=total_messages,
        topics=tuple(sorted(deduped.values(), key=lambda topic: topic.name)),
    )


def find_topic(metadata: BagMetadata, role_spec: dict) -> TopicInfo | None:
    required_names = set(role_spec["topics"])
    allowed_types = set(role_spec["types"])
    by_exact_name = [
        topic for topic in metadata.topics
        if topic.name in required_names and topic.msg_type in allowed_types
    ]
    if by_exact_name:
        return sorted(by_exact_name, key=lambda topic: (-topic.count, topic.name))[0]

    if role_spec.get("allow_type_fallback"):
        fallback = [
            topic for topic in metadata.topics
            if topic.msg_type in allowed_types and "imu" in topic.name.lower()
        ]
        if fallback:
            return sorted(fallback, key=lambda topic: (-topic.count, topic.name))[0]
    return None


def check_roles(metadata: BagMetadata) -> list[RoleCheck]:
    checks = []
    for role_spec in REQUIRED_ROLES:
        found = find_topic(metadata, role_spec)
        if found is None:
            checks.append(
                RoleCheck(
                    role=role_spec["role"],
                    required=tuple(role_spec["topics"]),
                    found=None,
                    passed=False,
                    reason="missing",
                )
            )
            continue
        passed = found.count > 0
        checks.append(
            RoleCheck(
                role=role_spec["role"],
                required=tuple(role_spec["topics"]),
                found=found,
                passed=passed,
                reason="ok" if passed else "zero messages",
            )
        )
    return checks


def is_ready(metadata: BagMetadata, checks: list[RoleCheck], min_duration_s: float) -> bool:
    duration_ok = metadata.duration_s is not None and metadata.duration_s >= min_duration_s
    return duration_ok and all(check.passed for check in checks)


def format_seconds(value: float | None) -> str:
    if value is None:
        return "TBD"
    return f"{value:.2f}"


def format_count(value: int | None) -> str:
    return "TBD" if value is None else str(value)


def format_report(metadata: BagMetadata, checks: list[RoleCheck], min_duration_s: float) -> str:
    verdict = "PASS" if is_ready(metadata, checks, min_duration_s) else "FAIL"
    lines = [
        "# MBES Benchmark Readiness Report",
        "",
        f"- Metadata: `{metadata.path}`",
        f"- Verdict: **{verdict}**",
        f"- Duration s: {format_seconds(metadata.duration_s)}",
        f"- Minimum duration s: {format_seconds(min_duration_s)}",
        f"- Total messages: {format_count(metadata.message_count)}",
        "",
        "## Required Topics",
        "",
        "| Role | Required topic(s) | Found topic | Type | Messages | Status |",
        "|------|-------------------|-------------|------|---------:|--------|",
    ]
    for check in checks:
        found_name = check.found.name if check.found else "TBD"
        found_type = check.found.msg_type if check.found else "TBD"
        found_count = str(check.found.count) if check.found else "0"
        status = "PASS" if check.passed else f"FAIL: {check.reason}"
        lines.append(
            "| "
            + " | ".join(
                [
                    check.role,
                    ", ".join(f"`{topic}`" for topic in check.required),
                    f"`{found_name}`" if found_name != "TBD" else "TBD",
                    f"`{found_type}`" if found_type != "TBD" else "TBD",
                    found_count,
                    status,
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Next Command",
            "",
            "If the verdict is PASS, record the loop-status benchmark bag:",
            "",
            "```bash",
            "MBES_SRC=/path/to/beach_pond_ros2 \\",
            "MBES_OUT=/tmp/aqua_mbes_beach_pond_with_loop_status \\",
            "MBES_DURATION=120 \\",
            "./aqua_localization/scripts/record_mbes_demo.sh",
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Check MBES-SLAM beach_pond rosbag2 metadata before running loop-status benchmarks."
    )
    parser.add_argument("--bag", required=True, type=Path, help="Bag directory or metadata.yaml path.")
    parser.add_argument("--out", type=Path, help="Optional Markdown report output path.")
    parser.add_argument(
        "--min-duration-s",
        type=float,
        default=DEFAULT_MIN_DURATION_S,
        help="Minimum required bag duration in seconds.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    metadata_path = args.bag if args.bag.name == "metadata.yaml" else args.bag / "metadata.yaml"
    if not metadata_path.is_file():
        print(f"metadata.yaml not found: {metadata_path}", file=sys.stderr)
        return 2

    metadata = parse_metadata(metadata_path)
    checks = check_roles(metadata)
    report = format_report(metadata, checks, args.min_duration_s)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
    else:
        print(report)
    return 0 if is_ready(metadata, checks, args.min_duration_s) else 1


if __name__ == "__main__":
    sys.exit(main())
