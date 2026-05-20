#!/usr/bin/env python3
"""Create a scaffolded underwater 3DGS dataset pack from a manifest JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys


PACK_SCHEMA = "aqua_localization.underwater_3dgs_dataset_pack.v1"
TRANSFORMS_SCHEMA = "aqua_localization.transforms_stub.v1"


def load_manifest(path: Path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"manifest not found: {path}")
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid manifest JSON: {path}: {exc}") from exc


def role_topic(manifest, role: str):
    entry = manifest.get("roles", {}).get(role, {})
    return entry.get("topic") if entry.get("status") == "found" else None


def role_count(manifest, role: str):
    entry = manifest.get("roles", {}).get(role, {})
    count = entry.get("message_count")
    return count if isinstance(count, int) else None


def selected_inputs(manifest):
    roles = manifest.get("roles", {})
    selected = {}
    for role, entry in roles.items():
        selected[role] = {
            "topic": entry.get("topic"),
            "type": entry.get("type"),
            "message_count": entry.get("message_count"),
            "status": entry.get("status"),
            "required": entry.get("required", False),
        }
    return selected


def make_transforms_stub(manifest):
    image_count = role_count(manifest, "image")
    return {
        "schema": TRANSFORMS_SCHEMA,
        "dataset": manifest.get("dataset"),
        "sequence": manifest.get("sequence"),
        "camera_model": "from_ros_camera_info",
        "source_topics": {
            "image": role_topic(manifest, "image"),
            "camera_info": role_topic(manifest, "camera_info"),
            "trajectory": role_topic(manifest, "trajectory"),
            "depth_pressure": role_topic(manifest, "depth_pressure"),
            "mbes": role_topic(manifest, "mbes"),
        },
        "frames": [],
        "expected_image_count": image_count,
        "notes": [
            "This is a placeholder transforms file for future image and pose extraction.",
            "Frame entries should use exported images/ paths and camera-to-world transforms.",
        ],
    }


def make_pack_index(manifest, output_dir: Path):
    return {
        "schema": PACK_SCHEMA,
        "dataset": manifest.get("dataset"),
        "sequence": manifest.get("sequence"),
        "source_manifest": "manifest.json",
        "source_bag": manifest.get("bag", {}),
        "paths": {
            "images": "images/",
            "depth": "depth/",
            "mbes": "mbes/",
            "transforms": "transforms_stub.json",
            "readme": "README.md",
        },
        "selected_inputs": selected_inputs(manifest),
        "warnings": manifest.get("warnings", []),
        "output_dir": str(output_dir),
        "status": "scaffold_only",
    }


def format_topic_line(label, entry):
    topic = entry.get("topic") or "<missing>"
    msg_type = entry.get("type") or "<unknown>"
    count = entry.get("message_count")
    status = entry.get("status") or "unknown"
    count_text = "unknown count" if count is None else f"{count} messages"
    return f"- {label}: `{topic}` (`{msg_type}`, {count_text}, {status})"


def make_readme(manifest, pack_index):
    roles = manifest.get("roles", {})
    lines = [
        f"# Underwater 3DGS Dataset Pack: {manifest.get('dataset')} / {manifest.get('sequence')}",
        "",
        "This directory is a scaffold for a future underwater 3D Gaussian Splatting export.",
        "It does not contain extracted image frames yet.",
        "",
        "## Source",
        "",
        f"- Bag: `{manifest.get('bag', {}).get('path')}`",
        f"- Metadata: `{manifest.get('bag', {}).get('metadata')}`",
        f"- Duration: `{manifest.get('bag', {}).get('duration_s')}` seconds",
        "",
        "## Selected Inputs",
        "",
        format_topic_line("Image", roles.get("image", {})),
        format_topic_line("Camera info", roles.get("camera_info", {})),
        format_topic_line("Trajectory", roles.get("trajectory", {})),
        format_topic_line("Depth / pressure", roles.get("depth_pressure", {})),
        format_topic_line("MBES / submap", roles.get("mbes", {})),
        "",
        "## Files",
        "",
        "- `manifest.json`: copied source manifest",
        "- `pack_index.json`: dataset pack index and selected input summary",
        "- `transforms_stub.json`: placeholder camera transform schema",
        "- `images/`: future exported camera frames",
        "- `depth/`: future depth or pressure-derived priors",
        "- `mbes/`: future MBES or submap priors",
    ]
    warnings = pack_index.get("warnings") or []
    if warnings:
        lines.extend(["", "## Warnings", ""])
        lines.extend(f"- {warning}" for warning in warnings)
    lines.extend(
        [
            "",
            "## Next Step",
            "",
            "Use this pack as the stable target for the future ROS bag frame extractor.",
            "Once images and poses are exported, replace `transforms_stub.json` with a concrete training-format file.",
            "",
        ]
    )
    return "\n".join(lines)


def ensure_output_dir(path: Path, force: bool):
    if path.exists():
        if not path.is_dir():
            raise FileExistsError(f"output path exists and is not a directory: {path}")
        if any(path.iterdir()) and not force:
            raise FileExistsError(f"output directory is not empty: {path}")
        if force:
            for child in path.iterdir():
                if child.is_dir():
                    shutil.rmtree(child)
                else:
                    child.unlink()
    path.mkdir(parents=True, exist_ok=True)


def write_json(path: Path, payload):
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def create_pack(manifest_path: Path, output_dir: Path, force: bool = False):
    manifest = load_manifest(manifest_path)
    ensure_output_dir(output_dir, force)

    for dirname in ("images", "depth", "mbes"):
        directory = output_dir / dirname
        directory.mkdir(parents=True, exist_ok=True)
        (directory / ".gitkeep").write_text("", encoding="utf-8")

    pack_index = make_pack_index(manifest, output_dir)
    transforms = make_transforms_stub(manifest)

    write_json(output_dir / "manifest.json", manifest)
    write_json(output_dir / "pack_index.json", pack_index)
    write_json(output_dir / "transforms_stub.json", transforms)
    (output_dir / "README.md").write_text(make_readme(manifest, pack_index), encoding="utf-8")
    return pack_index


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Create a scaffolded 3DGS dataset pack from export_3dgs_manifest.py JSON."
    )
    parser.add_argument("--manifest", required=True, type=Path, help="Input 3DGS manifest JSON.")
    parser.add_argument("--out", required=True, type=Path, help="Output dataset pack directory.")
    parser.add_argument("--force", action="store_true", help="Replace an existing non-empty output directory.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        pack_index = create_pack(args.manifest, args.out, args.force)
    except (FileNotFoundError, FileExistsError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(pack_index, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
