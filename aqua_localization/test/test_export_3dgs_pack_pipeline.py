"""Tests for export_3dgs_pack_pipeline.py."""

import importlib.util
import json
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "export_3dgs_pack_pipeline.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("export_3dgs_pack_pipeline", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_metadata(tmp_path):
    metadata = tmp_path / "metadata.yaml"
    metadata.write_text(
        "\n".join(
            [
                "rosbag2_bagfile_information:",
                "  storage_identifier: mcap",
                "  duration:",
                "    nanoseconds: 15000000000",
                "  topics_with_message_count:",
                "    - topic_metadata:",
                "        name: /camera/left/image_raw",
                "        type: sensor_msgs/msg/Image",
                "      message_count: 300",
                "    - topic_metadata:",
                "        name: /camera/left/camera_info",
                "        type: sensor_msgs/msg/CameraInfo",
                "      message_count: 300",
                "    - topic_metadata:",
                "        name: /aqua_visual_frontend/odometry",
                "        type: nav_msgs/msg/Odometry",
                "      message_count: 271",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return metadata


def test_pipeline_runs_all_steps_with_summary(tmp_path, monkeypatch):
    module = load_module()
    write_metadata(tmp_path)
    out = tmp_path / "pack"

    def fake_export_frames(manifest_path, pack_dir, max_frames, stride, image_format, jpeg_quality):
        assert manifest_path == out / "manifest.json"
        assert pack_dir == out
        assert max_frames == 12
        assert stride == 3
        assert image_format == "jpg"
        assert jpeg_quality == 88
        payload = {
            "schema": "aqua_localization.underwater_3dgs_frames.v1",
            "frame_count": 4,
            "frames": [{"file_path": "images/frame_000000.jpg"}],
        }
        module.export_3dgs_dataset_pack.write_json(pack_dir / "frames.json", payload)
        return payload

    def fake_build_transforms(
        manifest_path,
        pack_dir,
        max_time_diff_s,
        output_format,
        base_from_camera_values,
        camera_intrinsics_values,
        camera_model,
        distortion_params,
    ):
        assert manifest_path == out / "manifest.json"
        assert pack_dir == out
        assert max_time_diff_s == 0.07
        assert output_format == "nerfstudio"
        assert base_from_camera_values == [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
        assert camera_intrinsics_values == [612.0, 512.0, 655.0, 655.0, 306.0, 256.0]
        assert camera_model == "pinhole"
        assert distortion_params == [0.0, 0.0]
        payload = {
            "schema": "aqua_localization.nerfstudio_transforms.v1",
            "format": "nerfstudio",
            "frames": [{"file_path": "images/frame_000000.jpg"}],
            "metadata": {"skipped_count": 1},
        }
        module.export_3dgs_dataset_pack.write_json(pack_dir / "transforms.json", payload)
        return payload

    monkeypatch.setattr(module.export_3dgs_frames, "export_frames", fake_export_frames)
    monkeypatch.setattr(module.export_3dgs_transforms, "build_transforms", fake_build_transforms)

    args = module.parse_args(
        [
            "--bag",
            str(tmp_path),
            "--dataset",
            "Tank Dataset",
            "--sequence",
            "short_test",
            "--out",
            str(out),
            "--max-frames",
            "12",
            "--stride",
            "3",
            "--image-format",
            "jpg",
            "--jpeg-quality",
            "88",
            "--max-time-diff",
            "0.07",
            "--format",
            "nerfstudio",
            "--base-from-camera",
            "0.1",
            "0.2",
            "0.3",
            "0.0",
            "0.0",
            "0.0",
            "1.0",
            "--camera-intrinsics",
            "612",
            "512",
            "655",
            "655",
            "306",
            "256",
            "--camera-model",
            "pinhole",
            "--distortion-params",
            "0.0",
            "0.0",
        ]
    )
    summary = module.run_pipeline(args)

    assert summary["schema"] == module.PIPELINE_SCHEMA
    assert summary["status"] == "complete"
    assert summary["counts"]["frames"] == 4
    assert summary["counts"]["transforms"] == 1
    assert summary["counts"]["skipped_transforms"] == 1
    assert summary["formats"]["transforms_format"] == "nerfstudio"
    assert summary["options"]["camera_intrinsics"] == [612.0, 512.0, 655.0, 655.0, 306.0, 256.0]
    assert summary["inputs"]["image_topic"] == "/camera/left/image_raw"
    assert (out / "manifest.json").is_file()
    assert (out / "pack_index.json").is_file()
    assert json.loads((out / "summary.json").read_text(encoding="utf-8")) == summary


def test_cli_help_includes_core_options():
    proc = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--help"],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "--bag" in proc.stdout
    assert "--max-frames" in proc.stdout
    assert "--base-from-camera" in proc.stdout
