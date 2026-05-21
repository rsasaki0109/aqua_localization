"""Tests for check_3dgs_training_ready.py."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "check_3dgs_training_ready.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("check_3dgs_training_ready", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_pack(tmp_path, transforms=None, image_names=None):
    pack = tmp_path / "pack"
    images = pack / "images"
    images.mkdir(parents=True)
    for name in image_names or ["frame_000000.png"]:
        (images / name).write_bytes(b"fake image")
    payload = transforms or {
        "schema": "aqua_localization.nerfstudio_transforms.v1",
        "format": "nerfstudio",
        "camera_model": "pinhole",
        "w": 612,
        "h": 512,
        "fl_x": 655.0,
        "fl_y": 655.0,
        "cx": 306.0,
        "cy": 256.0,
        "metadata": {"intrinsics_source": "manual"},
        "frames": [
            {
                "file_path": "images/frame_000000.png",
                "transform_matrix": [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            }
        ],
    }
    (pack / "transforms.json").write_text(json.dumps(payload), encoding="utf-8")
    return pack


def test_training_ready_report_accepts_valid_pack(tmp_path):
    module = load_module()
    pack = write_pack(tmp_path)

    report = module.build_report(pack)

    assert report["ready"] is True
    assert report["format"] == "nerfstudio"
    assert report["frame_count"] == 1
    assert report["image_count"] == 1
    assert report["intrinsics"]["fl_x"] == 655.0
    assert report["intrinsics_source"] == "manual"
    assert report["failures"] == []


def test_training_ready_report_rejects_missing_image_and_intrinsics(tmp_path):
    module = load_module()
    transforms = {
        "format": "aqua",
        "w": 612,
        "h": 512,
        "frames": [
            {
                "file_path": "images/missing.png",
                "transform_matrix": [[1.0, 0.0], [0.0, 1.0]],
            }
        ],
    }
    pack = write_pack(tmp_path, transforms=transforms, image_names=[])

    report = module.build_report(pack)

    assert report["ready"] is False
    assert any("format must be nerfstudio" in failure for failure in report["failures"])
    assert any("missing camera intrinsics" in failure for failure in report["failures"])
    assert any("missing frame images" in failure for failure in report["failures"])
    assert any("invalid 4x4" in failure for failure in report["failures"])


def test_cli_returns_nonzero_when_not_ready(tmp_path):
    transforms = {
        "format": "nerfstudio",
        "camera_model": "pinhole",
        "w": 612,
        "h": 512,
        "fl_x": 655.0,
        "fl_y": 655.0,
        "cx": 306.0,
        "cy": 256.0,
        "frames": [{"file_path": "images/missing.png", "transform_matrix": []}],
    }
    pack = write_pack(tmp_path, transforms=transforms, image_names=[])

    proc = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--pack", str(pack), "--json"],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 1
    assert '"ready": false' in proc.stdout


def test_cli_help_includes_pack_option():
    proc = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--help"],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "--pack" in proc.stdout
