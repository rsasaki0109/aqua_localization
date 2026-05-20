#!/usr/bin/env python3
"""Run the underwater 3DGS pack export pipeline in one command."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

import export_3dgs_dataset_pack
import export_3dgs_frames
import export_3dgs_manifest
import export_3dgs_transforms


PIPELINE_SCHEMA = "aqua_localization.underwater_3dgs_pack_pipeline.v1"


def write_json(path: Path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def manifest_args_from_pipeline(args):
    return SimpleNamespace(
        bag=args.bag,
        dataset=args.dataset,
        sequence=args.sequence,
        image_topic=args.image_topic,
        camera_info_topic=args.camera_info_topic,
        trajectory_topic=args.trajectory_topic,
        depth_topic=args.depth_topic,
        mbes_topic=args.mbes_topic,
    )


def build_summary(args, manifest, pack_index, frames_payload, transforms_payload):
    warnings = manifest.get("warnings", [])
    if args.camera_intrinsics is not None:
        warnings = [
            warning
            for warning in warnings
            if warning != "missing required camera intrinsics topic"
        ]

    return {
        "schema": PIPELINE_SCHEMA,
        "dataset": manifest.get("dataset"),
        "sequence": manifest.get("sequence"),
        "output_dir": str(args.out),
        "inputs": {
            "bag": str(args.bag),
            "image_topic": manifest.get("roles", {}).get("image", {}).get("topic"),
            "camera_info_topic": manifest.get("roles", {}).get("camera_info", {}).get("topic"),
            "trajectory_topic": manifest.get("roles", {}).get("trajectory", {}).get("topic"),
            "depth_topic": manifest.get("roles", {}).get("depth_pressure", {}).get("topic"),
            "mbes_topic": manifest.get("roles", {}).get("mbes", {}).get("topic"),
        },
        "options": {
            "max_frames": args.max_frames,
            "stride": args.stride,
            "image_format": args.image_format,
            "jpeg_quality": args.jpeg_quality,
            "max_time_diff": args.max_time_diff,
            "transforms_format": args.format,
            "base_from_camera": args.base_from_camera,
            "camera_intrinsics": args.camera_intrinsics,
            "camera_model": args.camera_model,
            "distortion_params": args.distortion_params,
        },
        "paths": {
            "manifest": "manifest.json",
            "pack_index": "pack_index.json",
            "frames": "frames.json",
            "transforms": "transforms.json",
            "summary": "summary.json",
        },
        "counts": {
            "manifest_topics": len(manifest.get("topics", [])),
            "frames": frames_payload.get("frame_count"),
            "transforms": len(transforms_payload.get("frames", [])),
            "skipped_transforms": (
                transforms_payload.get("skipped_count")
                if "skipped_count" in transforms_payload
                else transforms_payload.get("metadata", {}).get("skipped_count")
            ),
        },
        "formats": {
            "pack_schema": pack_index.get("schema"),
            "frames_schema": frames_payload.get("schema"),
            "transforms_schema": transforms_payload.get("schema"),
            "transforms_format": transforms_payload.get("format", "aqua"),
        },
        "warnings": warnings,
        "status": "complete",
    }


def run_pipeline(args):
    manifest = export_3dgs_manifest.build_manifest(manifest_args_from_pipeline(args))

    with tempfile.TemporaryDirectory(prefix="aqua_3dgs_manifest_") as tmp_dir:
        temp_manifest = Path(tmp_dir) / "manifest.json"
        export_3dgs_manifest.write_manifest(temp_manifest, manifest)

        pack_index = export_3dgs_dataset_pack.create_pack(temp_manifest, args.out, args.force)
        manifest_path = args.out / "manifest.json"

    frames_payload = export_3dgs_frames.export_frames(
        manifest_path,
        args.out,
        max_frames=args.max_frames,
        stride=args.stride,
        image_format=args.image_format,
        jpeg_quality=args.jpeg_quality,
    )
    transforms_payload = export_3dgs_transforms.build_transforms(
        manifest_path,
        args.out,
        max_time_diff_s=args.max_time_diff,
        output_format=args.format,
        base_from_camera_values=args.base_from_camera,
        camera_intrinsics_values=args.camera_intrinsics,
        camera_model=args.camera_model,
        distortion_params=args.distortion_params,
    )
    summary = build_summary(args, manifest, pack_index, frames_payload, transforms_payload)
    write_json(args.out / "summary.json", summary)
    return summary


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Create a 3DGS dataset pack from rosbag2 metadata, images, odometry, and CameraInfo."
    )
    parser.add_argument("--bag", required=True, type=Path, help="Path to a rosbag2 directory or metadata.yaml.")
    parser.add_argument("--dataset", required=True, help="Dataset name, e.g. Tank Dataset.")
    parser.add_argument("--sequence", required=True, help="Sequence name, e.g. short_test.")
    parser.add_argument("--out", required=True, type=Path, help="Output 3DGS dataset pack directory.")
    parser.add_argument("--force", action="store_true", help="Replace an existing non-empty output directory.")
    parser.add_argument("--max-frames", type=int, default=None, help="Maximum image frames to export.")
    parser.add_argument("--stride", type=int, default=1, help="Export every Nth image message.")
    parser.add_argument("--image-format", default="png", choices=["png", "jpg", "jpeg"], help="Output image format.")
    parser.add_argument("--jpeg-quality", type=int, default=92, help="JPEG quality when --image-format jpg/jpeg.")
    parser.add_argument("--max-time-diff", type=float, default=0.05, help="Maximum frame-to-odometry time difference in seconds.")
    parser.add_argument("--format", choices=sorted(export_3dgs_transforms.TRANSFORM_FORMATS), default="aqua", help="Output transforms.json format.")
    parser.add_argument(
        "--base-from-camera",
        nargs=7,
        type=float,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        default=None,
        help="Camera pose in the odometry base frame as x y z qx qy qz qw.",
    )
    parser.add_argument(
        "--camera-intrinsics",
        nargs=6,
        type=float,
        metavar=("W", "H", "FL_X", "FL_Y", "CX", "CY"),
        default=None,
        help="Manual camera intrinsics as w h fl_x fl_y cx cy when no CameraInfo topic exists.",
    )
    parser.add_argument("--camera-model", default="pinhole", help="Camera model label used with --camera-intrinsics.")
    parser.add_argument(
        "--distortion-params",
        nargs="*",
        type=float,
        default=None,
        help="Optional distortion parameters used with --camera-intrinsics.",
    )
    parser.add_argument("--image-topic", help="Override camera image topic.")
    parser.add_argument("--camera-info-topic", help="Override CameraInfo topic.")
    parser.add_argument("--trajectory-topic", help="Override estimated trajectory topic.")
    parser.add_argument("--depth-topic", help="Override depth or pressure topic.")
    parser.add_argument("--mbes-topic", help="Override optional MBES/submap PointCloud2 topic.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        summary = run_pipeline(args)
    except (FileNotFoundError, FileExistsError, RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
