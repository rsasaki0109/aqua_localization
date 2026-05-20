"""Tests for export_3dgs_frames.py."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "export_3dgs_frames.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("export_3dgs_frames", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def make_manifest(path: Path, msg_type="sensor_msgs/msg/Image", status="found"):
    manifest = {
        "schema": "aqua_localization.underwater_3dgs_manifest.v1",
        "dataset": "Tank Dataset",
        "sequence": "short_test",
        "bag": {
            "path": "datasets/public/tank_dataset/short_test_ros2",
            "storage_identifier": "mcap",
        },
        "roles": {
            "image": {
                "topic": "/camera/left/image_raw",
                "type": msg_type,
                "message_count": 4,
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
        path / "pack_index.json",
        {
            "schema": "aqua_localization.underwater_3dgs_dataset_pack.v1",
            "paths": {"images": "images/"},
            "status": "scaffold_only",
        },
    )


def stamp(sec, nanosec):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def raw_rgb_msg(image_rgb, sec=1, nanosec=2):
    height, width, channels = image_rgb.shape
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(sec, nanosec)),
        height=height,
        width=width,
        encoding="rgb8",
        is_bigendian=False,
        step=width * channels,
        data=image_rgb.tobytes(),
    )


def compressed_msg(image_bgr, image_format=".jpg", sec=1, nanosec=2):
    ok, encoded = cv2.imencode(image_format, image_bgr)
    assert ok
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(sec, nanosec)),
        format=image_format.lstrip("."),
        data=encoded.tobytes(),
    )


def test_export_raw_image_frames_updates_pack(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path)
    make_pack(pack)

    red_rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    red_rgb[:, :, 0] = 255
    blue_rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    blue_rgb[:, :, 2] = 255

    payload = module.export_frames(
        manifest_path,
        pack,
        reader=[
            (100, raw_rgb_msg(red_rgb, sec=10, nanosec=20)),
            (200, raw_rgb_msg(blue_rgb, sec=11, nanosec=30)),
        ],
    )

    assert payload["frame_count"] == 2
    assert payload["frames"][0]["file_path"] == "images/frame_000000.png"
    assert payload["frames"][0]["timestamp_ns"] == 100
    assert payload["frames"][0]["message_stamp_ns"] == 10_000_000_020
    assert payload["frames"][0]["encoding"] == "rgb8"

    saved = cv2.imread(str(pack / "images" / "frame_000000.png"), cv2.IMREAD_UNCHANGED)
    assert saved.shape == (2, 3, 3)
    assert saved[0, 0].tolist() == [0, 0, 255]

    frames = json.loads((pack / "frames.json").read_text(encoding="utf-8"))
    assert frames["schema"] == module.FRAMES_SCHEMA
    index = json.loads((pack / "pack_index.json").read_text(encoding="utf-8"))
    assert index["status"] == "frames_extracted"
    assert index["paths"]["frames"] == "frames.json"
    assert index["extracted_frames"]["count"] == 2


def test_export_respects_stride_and_max_frames(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path)
    make_pack(pack)

    image = np.zeros((1, 1, 3), dtype=np.uint8)
    reader = [(i, raw_rgb_msg(image, sec=i, nanosec=0)) for i in range(6)]

    payload = module.export_frames(
        manifest_path,
        pack,
        max_frames=2,
        stride=2,
        reader=reader,
    )

    assert payload["frame_count"] == 2
    assert [frame["source_index"] for frame in payload["frames"]] == [0, 2]
    assert payload["source_message_count"] == 3
    assert sorted(path.name for path in (pack / "images").glob("*.png")) == [
        "frame_000000.png",
        "frame_000001.png",
    ]


def test_export_compressed_image_to_jpeg(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path, msg_type="sensor_msgs/msg/CompressedImage")
    make_pack(pack)

    image_bgr = np.zeros((2, 2, 3), dtype=np.uint8)
    image_bgr[:, :, 1] = 180
    payload = module.export_frames(
        manifest_path,
        pack,
        image_format="jpg",
        reader=[(123, compressed_msg(image_bgr))],
    )

    assert payload["format"] == "jpg"
    assert payload["frames"][0]["file_path"] == "images/frame_000000.jpg"
    assert payload["frames"][0]["encoding"] == "jpg"
    assert (pack / "images" / "frame_000000.jpg").is_file()


def test_missing_image_topic_raises(tmp_path):
    module = load_module()
    manifest_path = tmp_path / "manifest.json"
    pack = tmp_path / "pack"
    make_manifest(manifest_path, status="missing")
    make_pack(pack)

    try:
        module.export_frames(manifest_path, pack, reader=[])
    except ValueError as exc:
        assert "no found image topic" in str(exc)
    else:
        raise AssertionError("expected ValueError")
