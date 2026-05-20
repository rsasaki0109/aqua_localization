#!/usr/bin/env python3
"""Check whether a rosbag2 directory has the minimum topics for 3DGS export."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import export_3dgs_manifest


REQUIRED_ROLES = ("image", "camera_info", "trajectory")


def manifest_args(args):
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


def role_ready(entry):
    return entry.get("status") == "found" and (entry.get("message_count") or 0) > 0


def build_report(args):
    manifest = export_3dgs_manifest.build_manifest(manifest_args(args))
    roles = manifest.get("roles", {})
    required = {}
    missing = []
    for role in REQUIRED_ROLES:
        entry = roles.get(role, {})
        ok = role_ready(entry)
        required[role] = {
            "ready": ok,
            "topic": entry.get("topic"),
            "type": entry.get("type"),
            "message_count": entry.get("message_count"),
            "status": entry.get("status"),
        }
        if not ok:
            missing.append(role)

    optional = {}
    for role in ("depth_pressure", "mbes"):
        entry = roles.get(role, {})
        optional[role] = {
            "ready": role_ready(entry),
            "topic": entry.get("topic"),
            "type": entry.get("type"),
            "message_count": entry.get("message_count"),
            "status": entry.get("status"),
        }

    return {
        "dataset": args.dataset,
        "sequence": args.sequence,
        "bag": str(args.bag),
        "ready": not missing,
        "required": required,
        "optional": optional,
        "missing_required_roles": missing,
        "warnings": manifest.get("warnings", []),
    }


def print_text_report(report):
    print(f"3DGS bag ready: {str(report['ready']).lower()}")
    print(f"bag: {report['bag']}")
    print("required:")
    for role, entry in report["required"].items():
        status = "ok" if entry["ready"] else "missing"
        print(
            f"  {role}: {status} topic={entry['topic']} "
            f"type={entry['type']} messages={entry['message_count']}"
        )
    print("optional:")
    for role, entry in report["optional"].items():
        status = "ok" if entry["ready"] else "missing"
        print(
            f"  {role}: {status} topic={entry['topic']} "
            f"type={entry['type']} messages={entry['message_count']}"
        )
    if report["warnings"]:
        print("warnings:")
        for warning in report["warnings"]:
            print(f"  - {warning}")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Check rosbag2 metadata for minimum underwater 3DGS export topics."
    )
    parser.add_argument("--bag", required=True, type=Path, help="Path to a rosbag2 directory or metadata.yaml.")
    parser.add_argument("--dataset", default="unknown", help="Dataset label for the report.")
    parser.add_argument("--sequence", default="unknown", help="Sequence label for the report.")
    parser.add_argument("--image-topic", help="Override camera image topic.")
    parser.add_argument("--camera-info-topic", help="Override CameraInfo topic.")
    parser.add_argument("--trajectory-topic", help="Override estimated trajectory topic.")
    parser.add_argument("--depth-topic", help="Override depth or pressure topic.")
    parser.add_argument("--mbes-topic", help="Override optional MBES/submap PointCloud2 topic.")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        report = build_report(args)
    except FileNotFoundError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print_text_report(report)
    return 0 if report["ready"] else 1


if __name__ == "__main__":
    sys.exit(main())
