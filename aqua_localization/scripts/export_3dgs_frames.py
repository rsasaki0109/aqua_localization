#!/usr/bin/env python3
"""Extract camera frames from a ROS 2 bag into a 3DGS dataset pack."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np


FRAMES_SCHEMA = "aqua_localization.underwater_3dgs_frames.v1"
SUPPORTED_FORMATS = {"png", "jpg", "jpeg"}


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"file not found: {path}")
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON: {path}: {exc}") from exc


def write_json(path: Path, payload):
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def image_role(manifest):
    role = manifest.get("roles", {}).get("image", {})
    if role.get("status") != "found" or not role.get("topic"):
        raise ValueError("manifest has no found image topic")
    msg_type = role.get("type")
    if msg_type not in {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}:
        raise ValueError(f"unsupported image topic type: {msg_type}")
    return role


def bag_path_from_manifest(manifest):
    bag_path = manifest.get("bag", {}).get("path")
    if not bag_path:
        raise ValueError("manifest has no bag.path")
    path = Path(bag_path).expanduser()
    return path.parent if path.name == "metadata.yaml" else path


def stamp_to_ns(stamp):
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if sec is None or nanosec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nanosec)


def message_stamp_ns(msg):
    return stamp_to_ns(getattr(getattr(msg, "header", None), "stamp", None))


def read_rosbag_images(bag_path: Path, topic: str, msg_type: str, storage_id: str | None = None):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python bag dependencies are unavailable. Source your ROS 2 workspace "
            "or run this command inside a ROS 2 environment."
        ) from exc

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id or "")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    message_cls = get_message(msg_type)

    while reader.has_next():
        current_topic, data, timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        yield timestamp_ns, deserialize_message(data, message_cls)


def row_cropped_array(msg, dtype, channels: int):
    itemsize = np.dtype(dtype).itemsize
    row_values = int(msg.step) // itemsize
    expected = int(msg.width) * channels
    array = np.frombuffer(msg.data, dtype=dtype)
    rows = array.reshape((int(msg.height), row_values))
    cropped = rows[:, :expected]
    if channels == 1:
        image = cropped.reshape((int(msg.height), int(msg.width)))
    else:
        image = cropped.reshape((int(msg.height), int(msg.width), channels))
    if bool(getattr(msg, "is_bigendian", False)) != (sys.byteorder == "big"):
        image = image.byteswap().newbyteorder()
    return image


def decode_raw_image(msg):
    encoding = str(getattr(msg, "encoding", "")).lower()
    if encoding in {"mono8", "8uc1"}:
        return row_cropped_array(msg, np.uint8, 1)
    if encoding in {"bgr8", "8uc3"}:
        return row_cropped_array(msg, np.uint8, 3)
    if encoding == "rgb8":
        return cv2.cvtColor(row_cropped_array(msg, np.uint8, 3), cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return row_cropped_array(msg, np.uint8, 4)
    if encoding == "rgba8":
        return cv2.cvtColor(row_cropped_array(msg, np.uint8, 4), cv2.COLOR_RGBA2BGRA)
    if encoding in {"mono16", "16uc1"}:
        return row_cropped_array(msg, np.uint16, 1)
    raise ValueError(f"unsupported sensor_msgs/Image encoding: {msg.encoding}")


def decode_compressed_image(msg):
    encoded = np.frombuffer(msg.data, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("failed to decode sensor_msgs/CompressedImage")
    return image


def decode_image_message(msg, msg_type: str):
    if msg_type == "sensor_msgs/msg/CompressedImage":
        return decode_compressed_image(msg)
    return decode_raw_image(msg)


def image_metadata(msg, image, msg_type: str):
    encoding = getattr(msg, "encoding", None)
    if msg_type == "sensor_msgs/msg/CompressedImage":
        encoding = getattr(msg, "format", "compressed")
    return {
        "encoding": encoding,
        "height": int(image.shape[0]),
        "width": int(image.shape[1]),
        "channels": 1 if len(image.shape) == 2 else int(image.shape[2]),
    }


def normalize_format(value: str):
    image_format = value.lower().lstrip(".")
    if image_format == "jpeg":
        return "jpg"
    if image_format not in SUPPORTED_FORMATS:
        raise ValueError(f"unsupported image format: {value}")
    return image_format


def imwrite_params(image_format: str, quality: int):
    if image_format == "jpg":
        return [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    if image_format == "png":
        return [int(cv2.IMWRITE_PNG_COMPRESSION), 3]
    return []


def save_image(path: Path, image, image_format: str, quality: int):
    path.parent.mkdir(parents=True, exist_ok=True)
    ok = cv2.imwrite(str(path), image, imwrite_params(image_format, quality))
    if not ok:
        raise RuntimeError(f"failed to write image: {path}")


def load_pack_index(pack_dir: Path):
    path = pack_dir / "pack_index.json"
    if not path.is_file():
        raise FileNotFoundError(f"pack_index.json not found: {path}")
    return load_json(path)


def update_pack_index(pack_dir: Path, pack_index, frames_payload):
    paths = pack_index.setdefault("paths", {})
    paths["frames"] = "frames.json"
    paths["images"] = "images/"
    pack_index["status"] = "frames_extracted"
    pack_index["extracted_frames"] = {
        "schema": frames_payload["schema"],
        "count": frames_payload["frame_count"],
        "image_topic": frames_payload["image_topic"],
        "image_type": frames_payload["image_type"],
        "stride": frames_payload["stride"],
        "max_frames": frames_payload["max_frames"],
        "format": frames_payload["format"],
    }
    write_json(pack_dir / "pack_index.json", pack_index)


def export_frames(
    manifest_path: Path,
    pack_dir: Path,
    max_frames: int | None = None,
    stride: int = 1,
    image_format: str = "png",
    jpeg_quality: int = 92,
    reader=None,
):
    if stride < 1:
        raise ValueError("stride must be >= 1")
    if max_frames is not None and max_frames < 1:
        raise ValueError("max_frames must be >= 1")

    image_format = normalize_format(image_format)
    manifest = load_json(manifest_path)
    role = image_role(manifest)
    topic = role["topic"]
    msg_type = role["type"]
    pack_index = load_pack_index(pack_dir)
    images_dir = pack_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    if reader is None:
        storage_id = manifest.get("bag", {}).get("storage_identifier")
        reader = read_rosbag_images(bag_path_from_manifest(manifest), topic, msg_type, storage_id)

    frames = []
    seen = 0
    exported = 0
    for timestamp_ns, msg in reader:
        if seen % stride != 0:
            seen += 1
            continue
        image = decode_image_message(msg, msg_type)
        suffix = "jpg" if image_format == "jpg" else image_format
        filename = f"frame_{exported:06d}.{suffix}"
        output_path = images_dir / filename
        save_image(output_path, image, image_format, jpeg_quality)
        meta = image_metadata(msg, image, msg_type)
        frames.append(
            {
                "index": exported,
                "source_index": seen,
                "timestamp_ns": int(timestamp_ns),
                "message_stamp_ns": message_stamp_ns(msg),
                "file_path": f"images/{filename}",
                "topic": topic,
                **meta,
            }
        )
        exported += 1
        seen += 1
        if max_frames is not None and exported >= max_frames:
            break

    payload = {
        "schema": FRAMES_SCHEMA,
        "dataset": manifest.get("dataset"),
        "sequence": manifest.get("sequence"),
        "image_topic": topic,
        "image_type": msg_type,
        "format": image_format,
        "stride": stride,
        "max_frames": max_frames,
        "source_message_count": seen,
        "frame_count": len(frames),
        "frames": frames,
    }
    write_json(pack_dir / "frames.json", payload)
    update_pack_index(pack_dir, pack_index, payload)
    return payload


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Extract image frames from a ROS 2 bag into a 3DGS dataset pack."
    )
    parser.add_argument("--manifest", required=True, type=Path, help="Input 3DGS manifest JSON.")
    parser.add_argument("--pack", required=True, type=Path, help="Dataset pack directory.")
    parser.add_argument("--max-frames", type=int, default=None, help="Maximum frames to export.")
    parser.add_argument("--stride", type=int, default=1, help="Export every Nth image message.")
    parser.add_argument("--format", default="png", choices=["png", "jpg", "jpeg"], help="Output image format.")
    parser.add_argument("--jpeg-quality", type=int, default=92, help="JPEG quality when --format jpg/jpeg.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        payload = export_frames(
            args.manifest,
            args.pack,
            max_frames=args.max_frames,
            stride=args.stride,
            image_format=args.format,
            jpeg_quality=args.jpeg_quality,
        )
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
