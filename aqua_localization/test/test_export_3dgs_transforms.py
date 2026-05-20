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


def make_manifest(
    path: Path,
    trajectory_type="nav_msgs/msg/Odometry",
    status="found",
    include_camera_info=True,
):
    roles = {
        "trajectory": {
            "topic": "/aqua_visual_frontend/odometry",
            "type": trajectory_type,
            "message_count": 3,
            "status": status,
            "required": True,
        }
    }
    if include_camera_info:
        roles["camera_info"] = {
            "topic": "/camera/left/camera_info",
            "type": "sensor_msgs/msg/CameraInfo",
            "message_count": 1,
            "status": "found",
            "required": True,
        }
    manifest = {
        "schema": "aqua_localization.underwater_3dgs_manifest.v1",
        "dataset": "Tank Dataset",
        "sequence": "short_test",
        "bag": {
            "path": "datasets/public/tank_dataset/short_test_ros2",
            "storage_identifier": "mcap",
        },
        "roles": roles,
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


def camera_info_msg(message_stamp_ns=500, width=640, height=480):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp_from_ns(message_stamp_ns)),
        width=width,
        height=height,
        k=[520.0, 0.0, 321.0, 0.0, 521.0, 239.0, 0.0, 0.0, 1.0],
        d=[0.1, -0.02, 0.001, 0.002, 0.0],
        distortion_model="plumb_bob",
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
        camera_info_reader=[(480, camera_info_msg())],
    )

    assert payload["schema"] == module.TRANSFORMS_SCHEMA
    assert payload["camera_info_topic"] == "/camera/left/camera_info"
    assert payload["intrinsics_source"] == "camera_info"
    assert payload["w"] == 640
    assert payload["h"] == 480
    assert payload["fl_x"] == 520.0
    assert payload["fl_y"] == 521.0
    assert payload["cx"] == 321.0
    assert payload["cy"] == 239.0
    assert payload["camera_model"] == "plumb_bob"
    assert payload["distortion_params"] == [0.1, -0.02, 0.001, 0.002, 0.0]
    assert payload["frame_count"] == 2
    assert payload["skipped_count"] == 1
    assert payload["frames"][0]["file_path"] == "images/frame_000000.png"
    assert payload["frames"][0]["time_diff_ns"] == 30
    assert payload["frames"][0]["transform_matrix"][0][3] == 1.0
    assert payload["frames"][0]["transform_matrix"][1][3] == 2.0
    assert payload["frames"][0]["transform_matrix"][2][3] == 3.0
    assert payload["base_from_camera"]["translation_xyz_m"] == [0.0, 0.0, 0.0]
    assert payload["base_from_camera"]["quaternion_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert abs(payload["frames"][1]["transform_matrix"][0][0]) < 1e-12
    assert abs(payload["frames"][1]["transform_matrix"][1][0] - 1.0) < 1e-12
    assert payload["skipped_frames"][0]["reason"] == "time_diff_exceeded"

    saved = json.loads((pack / "transforms.json").read_text(encoding="utf-8"))
    assert saved["frame_count"] == 2
    index = json.loads((pack / "pack_index.json").read_text(encoding="utf-8"))
    assert index["status"] == "transforms_estimated"
    assert index["paths"]["transforms"] == "transforms.json"
    assert index["estimated_transforms"]["count"] == 2
    assert index["estimated_transforms"]["intrinsics_source"] == "camera_info"


def test_build_transforms_allows_missing_camera_info(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path, include_camera_info=False)
    make_pack(pack)

    payload = module.build_transforms(
        manifest_path,
        pack,
        max_time_diff_s=0.000001,
        reader=[(900, odom_msg(990, xyz=(1.0, 2.0, 3.0)))],
    )

    assert payload["camera_info_topic"] is None
    assert payload["intrinsics_source"] is None
    assert "fl_x" not in payload


def test_build_transforms_can_write_nerfstudio_format(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path)
    make_pack(pack)

    payload = module.build_transforms(
        manifest_path,
        pack,
        max_time_diff_s=0.000001,
        output_format="nerfstudio",
        reader=[
            (900, odom_msg(990, xyz=(1.0, 2.0, 3.0))),
            (1_900, odom_msg(2_000, xyz=(4.0, 5.0, 6.0))),
        ],
        camera_info_reader=[(480, camera_info_msg())],
    )

    assert payload["schema"] == module.NERFSTUDIO_SCHEMA
    assert payload["format"] == "nerfstudio"
    assert payload["fl_x"] == 520.0
    assert payload["frames"][0]["file_path"] == "images/frame_000000.png"
    assert payload["frames"][0]["transform_matrix"][0][3] == 1.0
    assert payload["frames"][0]["metadata"]["time_diff_ns"] == 30
    assert "odom_timestamp_ns" not in payload["frames"][0]
    assert payload["metadata"]["trajectory_topic"] == "/aqua_visual_frontend/odometry"
    assert payload["metadata"]["matched_frame_count"] == 2
    assert payload["metadata"]["base_from_camera"]["translation_xyz_m"] == [0.0, 0.0, 0.0]

    saved = json.loads((pack / "transforms.json").read_text(encoding="utf-8"))
    assert saved["format"] == "nerfstudio"
    index = json.loads((pack / "pack_index.json").read_text(encoding="utf-8"))
    assert index["estimated_transforms"]["transforms_format"] == "nerfstudio"
    assert index["estimated_transforms"]["count"] == 2


def test_nerfstudio_format_requires_intrinsics(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path, include_camera_info=False)
    make_pack(pack)

    try:
        module.build_transforms(
            manifest_path,
            pack,
            output_format="nerfstudio",
            reader=[(900, odom_msg(990, xyz=(1.0, 2.0, 3.0)))],
        )
    except ValueError as exc:
        assert "requires CameraInfo intrinsics" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_base_from_camera_translation_is_applied(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path)
    make_pack(pack)

    payload = module.build_transforms(
        manifest_path,
        pack,
        max_time_diff_s=0.000001,
        base_from_camera_values=[0.25, -0.45, 0.1, 0.0, 0.0, 0.0, 1.0],
        reader=[(900, odom_msg(990, xyz=(1.0, 2.0, 3.0)))],
        camera_info_reader=[(480, camera_info_msg())],
    )

    matrix = payload["frames"][0]["transform_matrix"]
    assert matrix[0][3] == 1.25
    assert matrix[1][3] == 1.55
    assert matrix[2][3] == 3.1
    assert payload["base_from_camera"]["translation_xyz_m"] == [0.25, -0.45, 0.1]


def test_base_from_camera_rotation_is_composed(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path)
    make_pack(pack)
    yaw90 = [0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)]

    payload = module.build_transforms(
        manifest_path,
        pack,
        max_time_diff_s=0.000001,
        base_from_camera_values=[0.0, 0.0, 0.0, *yaw90],
        reader=[(900, odom_msg(990, xyz=(0.0, 0.0, 0.0)))],
        camera_info_reader=[(480, camera_info_msg())],
    )

    matrix = payload["frames"][0]["transform_matrix"]
    assert abs(matrix[0][0]) < 1e-12
    assert abs(matrix[0][1] + 1.0) < 1e-12
    assert abs(matrix[1][0] - 1.0) < 1e-12
    assert abs(matrix[1][1]) < 1e-12


def test_camera_info_requires_nine_k_values():
    module = load_module()
    msg = camera_info_msg()
    msg.k = [1.0, 2.0]

    try:
        module.camera_info_payload(1, msg)
    except ValueError as exc:
        assert "CameraInfo.k" in str(exc)
    else:
        raise AssertionError("expected ValueError")


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
            camera_info_reader=[],
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
