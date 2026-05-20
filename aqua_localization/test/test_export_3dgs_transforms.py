"""Tests for export_3dgs_transforms.py."""

import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "export_3dgs_transforms.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("export_3dgs_transforms", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def make_manifest(path: Path, trajectory_type="nav_msgs/msg/Odometry", status="found"):
    manifest = {
        "schema": "aqua_localization.underwater_3dgs_manifest.v1",
        "dataset": "Tank Dataset",
        "sequence": "short_test",
        "bag": {
            "path": "datasets/public/tank_dataset/short_test_ros2",
            "storage_identifier": "mcap",
        },
        "roles": {
            "trajectory": {
                "topic": "/aqua_visual_frontend/odometry",
                "type": trajectory_type,
                "message_count": 3,
                "status": status,
                "required": True,
            }
        },
    }
    write_json(path, manifest)
    return manifest


def make_pack(path: Path):
    path.mkdir(parents=True)
    write_json(
        path / "frames.json",
        {
            "schema": "aqua_localization.underwater_3dgs_frames.v1",
            "frames": [
                {
                    "index": 0,
                    "timestamp_ns": 1_000,
                    "message_stamp_ns": 1_020,
                    "file_path": "images/frame_000000.png",
                },
                {
                    "index": 1,
                    "timestamp_ns": 2_000,
                    "message_stamp_ns": 2_010,
                    "file_path": "images/frame_000001.png",
                },
                {
                    "index": 2,
                    "timestamp_ns": 9_000,
                    "message_stamp_ns": 9_000,
                    "file_path": "images/frame_000002.png",
                },
            ],
        },
    )
    write_json(
        path / "pack_index.json",
        {
            "schema": "aqua_localization.underwater_3dgs_dataset_pack.v1",
            "paths": {"frames": "frames.json"},
            "status": "frames_extracted",
        },
    )


def stamp_from_ns(value):
    return SimpleNamespace(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def odom_msg(message_stamp_ns, xyz=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0)):
    position = SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2])
    orientation = SimpleNamespace(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    pose = SimpleNamespace(position=position, orientation=orientation)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp_from_ns(message_stamp_ns)),
        pose=SimpleNamespace(pose=pose),
    )


def test_build_transforms_matches_nearest_odometry(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path)
    make_pack(pack)

    yaw90 = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    payload = module.build_transforms(
        manifest_path,
        pack,
        max_time_diff_s=0.000001,
        reader=[
            (900, odom_msg(990, xyz=(1.0, 2.0, 3.0))),
            (1_900, odom_msg(2_000, xyz=(4.0, 5.0, 6.0), quat=yaw90)),
            (7_000, odom_msg(7_000, xyz=(7.0, 8.0, 9.0))),
        ],
    )

    assert payload["schema"] == module.TRANSFORMS_SCHEMA
    assert payload["frame_count"] == 2
    assert payload["skipped_count"] == 1
    assert payload["frames"][0]["file_path"] == "images/frame_000000.png"
    assert payload["frames"][0]["time_diff_ns"] == 30
    assert payload["frames"][0]["transform_matrix"][0][3] == 1.0
    assert payload["frames"][0]["transform_matrix"][1][3] == 2.0
    assert payload["frames"][0]["transform_matrix"][2][3] == 3.0
    assert abs(payload["frames"][1]["transform_matrix"][0][0]) < 1e-12
    assert abs(payload["frames"][1]["transform_matrix"][1][0] - 1.0) < 1e-12
    assert payload["skipped_frames"][0]["reason"] == "time_diff_exceeded"

    saved = json.loads((pack / "transforms.json").read_text(encoding="utf-8"))
    assert saved["frame_count"] == 2
    index = json.loads((pack / "pack_index.json").read_text(encoding="utf-8"))
    assert index["status"] == "transforms_estimated"
    assert index["paths"]["transforms"] == "transforms.json"
    assert index["estimated_transforms"]["count"] == 2


def test_zero_norm_quaternion_raises(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path)
    make_pack(pack)

    try:
        module.build_transforms(
            manifest_path,
            pack,
            reader=[(1_000, odom_msg(1_020, quat=(0.0, 0.0, 0.0, 0.0)))],
        )
    except ValueError as exc:
        assert "zero-norm quaternion" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_missing_frames_json_raises(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path)
    pack.mkdir()
    write_json(pack / "pack_index.json", {"paths": {}})

    try:
        module.build_transforms(manifest_path, pack, reader=[])
    except FileNotFoundError as exc:
        assert "frames.json not found" in str(exc)
    else:
        raise AssertionError("expected FileNotFoundError")


def test_unsupported_trajectory_type_raises(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path, trajectory_type="nav_msgs/msg/Path")
    make_pack(pack)

    try:
        module.build_transforms(manifest_path, pack, reader=[])
    except ValueError as exc:
        assert "unsupported trajectory topic type" in str(exc)
    else:
        raise AssertionError("expected ValueError")
