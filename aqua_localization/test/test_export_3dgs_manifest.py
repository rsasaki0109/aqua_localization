"""Tests for export_3dgs_manifest.py."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "export_3dgs_manifest.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("export_3dgs_manifest", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_metadata(tmp_path, topics, duration_ns=15_000_000_000, start_ns=42_000_000_000):
    lines = [
        "rosbag2_bagfile_information:",
        "  storage_identifier: mcap",
        "  duration:",
        f"    nanoseconds: {duration_ns}",
        "  starting_time:",
        f"    nanoseconds_since_epoch: {start_ns}",
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


def test_build_manifest_selects_3dgs_inputs(tmp_path):
    module = load_module()
    write_metadata(
        tmp_path,
        [
            ("/camera/left/image_raw", "sensor_msgs/msg/Image", 300),
            ("/camera/left/camera_info", "sensor_msgs/msg/CameraInfo", 300),
            ("/aqua_imu_loc/odometry", "nav_msgs/msg/Odometry", 5400),
            ("/pressure", "sensor_msgs/msg/FluidPressure", 450),
            ("/norbit/detections", "sensor_msgs/msg/PointCloud2", 780),
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
    manifest = module.build_manifest(args)

    assert manifest["schema"] == module.SCHEMA
    assert manifest["bag"]["storage_identifier"] == "mcap"
    assert manifest["bag"]["duration_s"] == 15.0
    assert manifest["bag"]["end_time_ns"] == 57_000_000_000
    assert manifest["roles"]["image"]["topic"] == "/camera/left/image_raw"
    assert manifest["roles"]["image"]["message_count"] == 300
    assert manifest["roles"]["camera_info"]["topic"] == "/camera/left/camera_info"
    assert manifest["roles"]["trajectory"]["topic"] == "/aqua_imu_loc/odometry"
    assert manifest["roles"]["depth_pressure"]["topic"] == "/pressure"
    assert manifest["roles"]["mbes"]["topic"] == "/norbit/detections"
    assert manifest["warnings"] == []


def test_manifest_reports_missing_required_topics(tmp_path):
    module = load_module()
    write_metadata(
        tmp_path,
        [
            ("/pressure", "sensor_msgs/msg/FluidPressure", 450),
            ("/sonar/points", "sensor_msgs/msg/PointCloud2", 100),
        ],
    )

    args = module.parse_args([
        "--bag",
        str(tmp_path / "metadata.yaml"),
        "--dataset",
        "MBES-SLAM",
        "--sequence",
        "beach_pond",
    ])
    manifest = module.build_manifest(args)

    assert manifest["roles"]["image"]["status"] == "missing"
    assert manifest["roles"]["camera_info"]["status"] == "missing"
    assert manifest["roles"]["trajectory"]["status"] == "missing"
    assert "missing required camera image topic" in manifest["warnings"]
    assert "missing required camera intrinsics topic" in manifest["warnings"]
    assert "missing required estimated trajectory topic" in manifest["warnings"]


def test_override_topic_is_used(tmp_path):
    module = load_module()
    write_metadata(
        tmp_path,
        [
            ("/front/image_raw", "sensor_msgs/msg/Image", 100),
            ("/left/image_raw", "sensor_msgs/msg/Image", 300),
            ("/left/camera_info", "sensor_msgs/msg/CameraInfo", 300),
            ("/aqua_fusion/odometry", "nav_msgs/msg/Odometry", 1000),
        ],
    )

    args = module.parse_args([
        "--bag",
        str(tmp_path),
        "--dataset",
        "AQUALOC",
        "--sequence",
        "harbor_07",
        "--image-topic",
        "/front/image_raw",
    ])
    manifest = module.build_manifest(args)

    assert manifest["roles"]["image"]["topic"] == "/front/image_raw"
    assert manifest["roles"]["image"]["source"] == "override"
    assert manifest["roles"]["image"]["message_count"] == 100


def test_cli_writes_manifest_json(tmp_path):
    write_metadata(
        tmp_path,
        [
            ("/camera/left/image_raw", "sensor_msgs/msg/Image", 300),
            ("/camera/left/camera_info", "sensor_msgs/msg/CameraInfo", 300),
            ("/aqua_visual_frontend/odometry", "nav_msgs/msg/Odometry", 271),
        ],
    )
    out = tmp_path / "manifest.json"

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--bag",
            str(tmp_path),
            "--dataset",
            "Tank Dataset",
            "--sequence",
            "short_test",
            "--out",
            str(out),
        ],
        check=True,
    )

    manifest = json.loads(out.read_text(encoding="utf-8"))
    assert manifest["dataset"] == "Tank Dataset"
    assert manifest["sequence"] == "short_test"
    assert manifest["roles"]["trajectory"]["topic"] == "/aqua_visual_frontend/odometry"
    assert "missing optional depth or pressure prior topic" in manifest["warnings"]
