"""Tests for check_3dgs_bag_ready.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "check_3dgs_bag_ready.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("check_3dgs_bag_ready", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_metadata(tmp_path, topics):
    lines = [
        "rosbag2_bagfile_information:",
        "  storage_identifier: mcap",
        "  topics_with_message_count:",
    ]
    for name, msg_type, count in topics:
        lines.extend(
            [
                "    - topic_metadata:",
                f"        name: {name}",
                f"        type: {msg_type}",
                f"      message_count: {count}",
            ]
        )
    metadata = tmp_path / "metadata.yaml"
    metadata.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return metadata


def test_ready_report_for_camera_bag(tmp_path):
    module = load_module()
    write_metadata(
        tmp_path,
        [
            ("/camera/left/image_raw", "sensor_msgs/msg/Image", 20),
            ("/camera/left/camera_info", "sensor_msgs/msg/CameraInfo", 20),
            ("/aqua_visual_frontend/odometry", "nav_msgs/msg/Odometry", 20),
            ("/pressure", "sensor_msgs/msg/FluidPressure", 20),
        ],
    )

    args = module.parse_args([
        "--bag",
        str(tmp_path),
        "--dataset",
        "Tank Dataset",
        "--sequence",
        "short_test",
    ])
    report = module.build_report(args)

    assert report["ready"] is True
    assert report["required"]["image"]["ready"] is True
    assert report["required"]["camera_info"]["ready"] is True
    assert report["required"]["trajectory"]["topic"] == "/aqua_visual_frontend/odometry"
    assert report["optional"]["depth_pressure"]["ready"] is True
    assert report["missing_required_roles"] == []


def test_missing_camera_info_fails(tmp_path):
    module = load_module()
    write_metadata(
        tmp_path,
        [
            ("/camera/left/image_raw", "sensor_msgs/msg/Image", 20),
            ("/aqua_visual_frontend/odometry", "nav_msgs/msg/Odometry", 20),
        ],
    )

    args = module.parse_args(["--bag", str(tmp_path)])
    report = module.build_report(args)

    assert report["ready"] is False
    assert "camera_info" in report["missing_required_roles"]


def test_cli_returns_nonzero_when_not_ready(tmp_path):
    write_metadata(
        tmp_path,
        [
            ("/imu/data", "sensor_msgs/msg/Imu", 10),
            ("/aqua_visual_frontend/odometry", "nav_msgs/msg/Odometry", 20),
        ],
    )

    proc = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--bag", str(tmp_path), "--json"],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 1
    assert '"ready": false' in proc.stdout
