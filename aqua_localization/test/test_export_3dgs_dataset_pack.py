"""Tests for export_3dgs_dataset_pack.py."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "export_3dgs_dataset_pack.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("export_3dgs_dataset_pack", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_manifest(path: Path):
    manifest = {
        "schema": "aqua_localization.underwater_3dgs_manifest.v1",
        "dataset": "Tank Dataset",
        "sequence": "short_test",
        "bag": {
            "path": "datasets/public/tank_dataset/short_test_ros2",
            "metadata": "datasets/public/tank_dataset/short_test_ros2/metadata.yaml",
            "duration_s": 15.0,
        },
        "roles": {
            "image": {
                "topic": "/camera/left/image_raw",
                "type": "sensor_msgs/msg/Image",
                "message_count": 300,
                "status": "found",
                "required": True,
            },
            "camera_info": {
                "topic": "/camera/left/camera_info",
                "type": "sensor_msgs/msg/CameraInfo",
                "message_count": 300,
                "status": "found",
                "required": True,
            },
            "trajectory": {
                "topic": "/aqua_visual_frontend/odometry",
                "type": "nav_msgs/msg/Odometry",
                "message_count": 271,
                "status": "found",
                "required": True,
            },
            "depth_pressure": {
                "topic": "/pressure",
                "type": "sensor_msgs/msg/FluidPressure",
                "message_count": 450,
                "status": "found",
                "required": False,
            },
            "mbes": {
                "topic": None,
                "type": None,
                "message_count": None,
                "status": "missing",
                "required": False,
            },
        },
        "warnings": ["missing optional MBES/submap prior topic"],
    }
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return manifest


def test_create_pack_writes_expected_scaffold(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    write_manifest(manifest_path)
    out = tmp_path / "pack"

    pack_index = module.create_pack(manifest_path, out)

    assert pack_index["schema"] == module.PACK_SCHEMA
    assert pack_index["dataset"] == "Tank Dataset"
    assert pack_index["sequence"] == "short_test"
    assert (out / "manifest.json").is_file()
    assert (out / "pack_index.json").is_file()
    assert (out / "transforms_stub.json").is_file()
    assert (out / "README.md").is_file()
    assert (out / "images" / ".gitkeep").is_file()
    assert (out / "depth" / ".gitkeep").is_file()
    assert (out / "mbes" / ".gitkeep").is_file()

    transforms = json.loads((out / "transforms_stub.json").read_text(encoding="utf-8"))
    assert transforms["schema"] == module.TRANSFORMS_SCHEMA
    assert transforms["source_topics"]["image"] == "/camera/left/image_raw"
    assert transforms["source_topics"]["trajectory"] == "/aqua_visual_frontend/odometry"
    assert transforms["expected_image_count"] == 300
    assert transforms["frames"] == []

    readme = (out / "README.md").read_text(encoding="utf-8")
    assert "Underwater 3DGS Dataset Pack" in readme
    assert "/camera/left/image_raw" in readme
    assert "missing optional MBES/submap prior topic" in readme


def test_create_pack_refuses_non_empty_output_without_force(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    write_manifest(manifest_path)
    out = tmp_path / "pack"
    out.mkdir()
    (out / "old.txt").write_text("old", encoding="utf-8")

    try:
        module.create_pack(manifest_path, out)
    except FileExistsError as exc:
        assert "not empty" in str(exc)
    else:
        raise AssertionError("expected FileExistsError")


def test_create_pack_force_replaces_output(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    write_manifest(manifest_path)
    out = tmp_path / "pack"
    out.mkdir()
    (out / "old.txt").write_text("old", encoding="utf-8")

    module.create_pack(manifest_path, out, force=True)

    assert not (out / "old.txt").exists()
    assert (out / "pack_index.json").is_file()


def test_cli_writes_pack_and_prints_index(tmp_path):
    manifest_path = tmp_path / "manifest.json"
    write_manifest(manifest_path)
    out = tmp_path / "pack"

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--manifest",
            str(manifest_path),
            "--out",
            str(out),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    printed = json.loads(proc.stdout)
    assert printed["status"] == "scaffold_only"
    assert printed["paths"]["images"] == "images/"
    assert (out / "README.md").is_file()
