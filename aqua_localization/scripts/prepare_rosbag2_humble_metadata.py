#!/usr/bin/env python3
"""Create a ROS 2 Humble-compatible rosbag2 metadata view.

Some bags produced by the Python ``rosbags`` toolchain include newer metadata
fields or structured QoS profiles that older ROS 2 Humble rosbag2 readers cannot
parse. This script creates a lightweight output directory containing symlinks to
the original bag files and a rewritten ``metadata.yaml`` that Humble can read.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import sys

import yaml


def bag_dir_for(path: Path) -> Path:
    return path.parent if path.name == "metadata.yaml" else path


def normalize_topic_metadata(metadata: dict) -> int:
    info = metadata.setdefault("rosbag2_bagfile_information", {})
    rewrites = 0
    version = info.get("version")
    if version is None or int(version) > 5:
        info["version"] = 5
        rewrites += 1

    topics = info.get("topics_with_message_count") or []
    for entry in topics:
        topic_metadata = entry.get("topic_metadata") or {}
        if topic_metadata.pop("type_description_hash", None) is not None:
            rewrites += 1
        qos = topic_metadata.get("offered_qos_profiles")
        if not isinstance(qos, str):
            topic_metadata["offered_qos_profiles"] = ""
            rewrites += 1
    return rewrites


def linked_file_target(src_file: Path, dst_file: Path) -> Path:
    try:
        return src_file.relative_to(dst_file.parent)
    except ValueError:
        return src_file.resolve()


def link_or_copy_bag_files(src_dir: Path, dst_dir: Path) -> int:
    linked = 0
    metadata_path = src_dir / "metadata.yaml"
    for src_file in sorted(src_dir.iterdir()):
        if src_file == metadata_path or not src_file.is_file():
            continue
        dst_file = dst_dir / src_file.name
        if dst_file.exists() or dst_file.is_symlink():
            dst_file.unlink()
        try:
            dst_file.symlink_to(linked_file_target(src_file, dst_file))
        except OSError:
            shutil.copy2(src_file, dst_file)
        linked += 1
    return linked


def load_and_normalize_metadata(src_dir: Path) -> tuple[dict, int]:
    metadata_path = src_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yaml not found: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    rewrites = normalize_topic_metadata(metadata)
    return metadata, rewrites


def write_metadata(metadata: dict, path: Path) -> None:
    path.write_text(
        yaml.safe_dump(metadata, sort_keys=False),
        encoding="utf-8",
    )


def rewrite_metadata_in_place(src: Path) -> int:
    src_dir = bag_dir_for(src)
    metadata, rewrites = load_and_normalize_metadata(src_dir)
    write_metadata(metadata, src_dir / "metadata.yaml")
    return rewrites


def prepare_metadata_view(src: Path, dst: Path) -> tuple[int, int]:
    src_dir = bag_dir_for(src)
    if dst.exists():
        raise FileExistsError(f"destination already exists: {dst}")

    dst.mkdir(parents=True)
    try:
        bag_files = link_or_copy_bag_files(src_dir, dst)
        metadata, rewrites = load_and_normalize_metadata(src_dir)
        write_metadata(metadata, dst / "metadata.yaml")
    except Exception:
        shutil.rmtree(dst, ignore_errors=True)
        raise
    return bag_files, rewrites


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--src", required=True, type=Path, help="Source rosbag2 directory")
    parser.add_argument("--dst", type=Path, help="Output metadata view")
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="Rewrite the source metadata.yaml instead of creating a linked view",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        if args.in_place:
            if args.dst is not None:
                raise ValueError("--dst cannot be used with --in-place")
            rewrites = rewrite_metadata_in_place(args.src)
            print(
                f"rewrote Humble metadata in place at {bag_dir_for(args.src)} "
                f"({rewrites} metadata rewrites)"
            )
            return 0
        if args.dst is None:
            raise ValueError("--dst is required unless --in-place is set")
        bag_files, rewrites = prepare_metadata_view(args.src, args.dst)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"failed to prepare Humble metadata view: {exc}", file=sys.stderr)
        return 1
    print(
        f"prepared Humble metadata view at {args.dst} "
        f"({bag_files} bag file links, {rewrites} metadata rewrites)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
