"""Tests for check_mbes_benchmark_ready.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "check_mbes_benchmark_ready.py"


def load_module():
    spec = importlib.util.spec_from_file_location("check_mbes_benchmark_ready", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_metadata(path: Path, topics, duration_ns=120_000_000_000, total=1000):
    lines = [
        "rosbag2_bagfile_information:",
        "  duration:",
        f"    nanoseconds: {duration_ns}",
        f"  message_count: {total}",
        "  topics_with_message_count:",
    ]
    for name, msg_type, count in topics:
        lines.extend(
            [
                f"  - message_count: {count}",
                "    topic_metadata:",
                f"      name: {name}",
                f"      type: {msg_type}",
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def good_topics():
    return [
        ("/norbit/detections", "sensor_msgs/msg/PointCloud2", 120),
        ("/nav/processed/odometry", "nav_msgs/msg/Odometry", 60),
        ("/nav/processed/microstrain/imu/madgwick", "sensor_msgs/msg/Imu", 500),
    ]


def test_parse_metadata_reads_duration_counts_and_topics(tmp_path):
    module = load_module()
    metadata_path = write_metadata(tmp_path / "metadata.yaml", good_topics())

    metadata = module.parse_metadata(metadata_path)

    assert metadata.duration_s == 120.0
    assert metadata.message_count == 1000
    assert len(metadata.topics) == 3
    assert metadata.topics[0].name == "/nav/processed/microstrain/imu/madgwick"


def test_ready_bag_passes_required_roles(tmp_path):
    module = load_module()
    metadata_path = write_metadata(tmp_path / "metadata.yaml", good_topics())

    metadata = module.parse_metadata(metadata_path)
    checks = module.check_roles(metadata)

    assert module.is_ready(metadata, checks, min_duration_s=60.0)
    assert [check.role for check in checks] == ["MBES points", "reference odometry", "IMU"]
    assert all(check.passed for check in checks)


def test_imu_type_fallback_accepts_raw_imu_topic(tmp_path):
    module = load_module()
    topics = [
        ("/norbit/detections", "sensor_msgs/msg/PointCloud2", 120),
        ("/nav/processed/odometry", "nav_msgs/msg/Odometry", 60),
        ("/some/other/imu", "sensor_msgs/msg/Imu", 500),
    ]
    metadata = module.parse_metadata(write_metadata(tmp_path / "metadata.yaml", topics))

    imu_check = module.check_roles(metadata)[2]

    assert imu_check.passed
    assert imu_check.found.name == "/some/other/imu"


def test_missing_topic_fails_report(tmp_path):
    module = load_module()
    topics = [
        ("/norbit/detections", "sensor_msgs/msg/PointCloud2", 120),
        ("/nav/processed/microstrain/imu/madgwick", "sensor_msgs/msg/Imu", 500),
    ]
    metadata = module.parse_metadata(write_metadata(tmp_path / "metadata.yaml", topics))
    checks = module.check_roles(metadata)
    report = module.format_report(metadata, checks, min_duration_s=1.0)

    assert not module.is_ready(metadata, checks, min_duration_s=1.0)
    assert "reference odometry" in report
    assert "FAIL: missing" in report


def test_cli_writes_report_and_returns_success_for_ready_bag(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    out = tmp_path / "ready.md"
    write_metadata(bag / "metadata.yaml", good_topics())

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--bag",
            str(bag),
            "--out",
            str(out),
            "--min-duration-s",
            "60",
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 0
    assert proc.stderr == ""
    assert "Verdict: **PASS**" in out.read_text(encoding="utf-8")


def test_cli_returns_failure_for_short_bag(tmp_path):
    metadata = write_metadata(
        tmp_path / "metadata.yaml",
        good_topics(),
        duration_ns=500_000_000,
    )

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--bag",
            str(metadata),
            "--min-duration-s",
            "1",
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 1
    assert "Verdict: **FAIL**" in proc.stdout
