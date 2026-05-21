#!/usr/bin/env python3
"""Check whether a 3DGS sample pack is ready for training tooling."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


REQUIRED_INTRINSICS = ("w", "h", "fl_x", "fl_y", "cx", "cy")


def load_json(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"file not found: {path}")
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON: {path}: {exc}") from exc


def is_4x4_matrix(value):
    if not isinstance(value, list) or len(value) != 4:
        return False
    return all(isinstance(row, list) and len(row) == 4 for row in value)


def image_paths(pack_dir: Path):
    image_dir = pack_dir / "images"
    if not image_dir.is_dir():
        return []
    return sorted(
        path
        for path in image_dir.iterdir()
        if path.is_file() and path.suffix.lower() in {".png", ".jpg", ".jpeg"}
    )


def build_report(pack_dir: Path):
    transforms_path = pack_dir / "transforms.json"
    transforms = load_json(transforms_path)
    frames = transforms.get("frames", [])
    failures = []
    warnings = []

    transform_format = transforms.get("format")
    if transform_format != "nerfstudio":
        failures.append("transforms.json format must be nerfstudio")

    if not isinstance(frames, list) or not frames:
        failures.append("transforms.json must contain at least one frame")
        frames = []

    missing_intrinsics = [
        field
        for field in REQUIRED_INTRINSICS
        if transforms.get(field) is None
    ]
    if missing_intrinsics:
        failures.append(f"missing camera intrinsics: {', '.join(missing_intrinsics)}")

    missing_images = []
    invalid_matrices = []
    duplicate_paths = []
    seen_paths = set()
    for index, frame in enumerate(frames):
        file_path = frame.get("file_path")
        if not file_path:
            missing_images.append(f"<frame {index} has no file_path>")
        else:
            if file_path in seen_paths:
                duplicate_paths.append(file_path)
            seen_paths.add(file_path)
            if not (pack_dir / file_path).is_file():
                missing_images.append(file_path)

        if not is_4x4_matrix(frame.get("transform_matrix")):
            invalid_matrices.append(index)

    if missing_images:
        shown = ", ".join(missing_images[:5])
        suffix = "" if len(missing_images) <= 5 else f", ... +{len(missing_images) - 5}"
        failures.append(f"missing frame images: {shown}{suffix}")
    if invalid_matrices:
        shown = ", ".join(str(index) for index in invalid_matrices[:8])
        suffix = "" if len(invalid_matrices) <= 8 else f", ... +{len(invalid_matrices) - 8}"
        failures.append(f"invalid 4x4 transform_matrix at frame indices: {shown}{suffix}")
    if duplicate_paths:
        shown = ", ".join(duplicate_paths[:5])
        suffix = "" if len(duplicate_paths) <= 5 else f", ... +{len(duplicate_paths) - 5}"
        failures.append(f"duplicate frame file_path entries: {shown}{suffix}")

    images = image_paths(pack_dir)
    frame_count = len(frames)
    if images and len(images) != frame_count:
        warnings.append(
            f"image file count ({len(images)}) differs from transform frame count ({frame_count})"
        )

    return {
        "pack": str(pack_dir),
        "ready": not failures,
        "format": transform_format,
        "frame_count": frame_count,
        "image_count": len(images),
        "intrinsics": {
            field: transforms.get(field)
            for field in REQUIRED_INTRINSICS
        },
        "camera_model": transforms.get("camera_model"),
        "intrinsics_source": transforms.get("metadata", {}).get("intrinsics_source"),
        "failures": failures,
        "warnings": warnings,
    }


def print_text_report(report):
    print(f"3DGS training ready: {str(report['ready']).lower()}")
    print(f"pack: {report['pack']}")
    print(f"format: {report['format']}")
    print(f"frames: {report['frame_count']}")
    print(f"images: {report['image_count']}")
    print(
        "intrinsics: "
        f"{report['intrinsics'].get('w')}x{report['intrinsics'].get('h')} "
        f"fx={report['intrinsics'].get('fl_x')} fy={report['intrinsics'].get('fl_y')} "
        f"cx={report['intrinsics'].get('cx')} cy={report['intrinsics'].get('cy')}"
    )
    print(f"intrinsics source: {report['intrinsics_source']}")
    if report["failures"]:
        print("failures:")
        for failure in report["failures"]:
            print(f"  - {failure}")
    if report["warnings"]:
        print("warnings:")
        for warning in report["warnings"]:
            print(f"  - {warning}")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Check a nerfstudio-style 3DGS sample pack before training."
    )
    parser.add_argument("--pack", required=True, type=Path, help="Path to the 3DGS sample pack directory.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        report = build_report(args.pack)
    except (FileNotFoundError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print_text_report(report)
    return 0 if report["ready"] else 1


if __name__ == "__main__":
    sys.exit(main())
