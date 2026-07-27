#!/usr/bin/env python3
"""Download, verify, extract, and optionally convert MBES-SLAM beach_pond."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys


DEFAULT_URL = "https://seaward.science/files/pos-datasets/bag/beach_pond.tar.gz"
EXPECTED_ARCHIVE_BYTES = 2_584_672_367
INCLUDED_TOPICS = [
    "/norbit/detections",
    "/nav/sensors/microstrain/imu/raw",
    "/nav/processed/microstrain/imu/madgwick",
    "/nav/sensors/microstrain/mag/raw",
    "/nav/processed/odometry",
    "/nav/sensors/navsat/ubx_pos/fix",
    "/tf",
    "/tf_static",
]


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(command: list[str], *, dry_run: bool) -> None:
    print("+", " ".join(command))
    if not dry_run:
        subprocess.run(command, check=True)


def locate_ros1_bag(destination: Path) -> Path | None:
    candidates = sorted(destination.glob("**/beach_pond.bag"))
    return candidates[0] if candidates else None


def acquire(
    destination: Path,
    *,
    url: str = DEFAULT_URL,
    convert: bool = True,
    dry_run: bool = False,
) -> tuple[Path, Path | None]:
    archive = destination / "beach_pond.tar.gz"
    ros2_bag = destination / "beach_pond_ros2"
    destination.mkdir(parents=True, exist_ok=True)

    if not archive.exists() or archive.stat().st_size != EXPECTED_ARCHIVE_BYTES:
        curl = shutil.which("curl")
        if curl is None:
            raise RuntimeError("curl is required")
        run(
            [
                curl,
                "--location",
                "--fail",
                "--insecure",
                "--continue-at",
                "-",
                "--retry",
                "5",
                "--retry-all-errors",
                "--connect-timeout",
                "20",
                "--output",
                str(archive),
                url,
            ],
            dry_run=dry_run,
        )
    elif not dry_run:
        print(f"using complete archive: {archive}")

    if not dry_run:
        size = archive.stat().st_size
        if size != EXPECTED_ARCHIVE_BYTES:
            raise RuntimeError(
                f"archive size mismatch: expected {EXPECTED_ARCHIVE_BYTES}, got {size}"
            )
        digest = sha256_of(archive)
        (destination / "beach_pond.tar.gz.sha256").write_text(
            f"{digest}  {archive.name}\n",
            encoding="utf-8",
        )

    ros1_bag = locate_ros1_bag(destination)
    if ros1_bag is None:
        run(
            ["tar", "xzf", str(archive), "-C", str(destination)],
            dry_run=dry_run,
        )
        if not dry_run:
            ros1_bag = locate_ros1_bag(destination)
            if ros1_bag is None:
                raise RuntimeError("extracted archive does not contain beach_pond.bag")

    if not convert:
        return archive, ros1_bag

    if (ros2_bag / "metadata.yaml").is_file():
        print(f"using converted rosbag2: {ros2_bag}")
        return archive, ros2_bag

    converter = shutil.which("rosbags-convert")
    if converter is None:
        raise RuntimeError("rosbags-convert is required (install the rosbags package)")
    if ros1_bag is None:
        # The dry-run path has not extracted the archive yet.
        ros1_bag = destination / "beach_pond" / "beach_pond.bag"
    command = [
        converter,
        "--src",
        str(ros1_bag),
        "--dst",
        str(ros2_bag),
        "--dst-storage",
        "mcap",
        "--src-typestore",
        "ros1_noetic",
        "--dst-typestore",
        "ros2_jazzy",
    ]
    for topic in INCLUDED_TOPICS:
        command.extend(["--include-topic", topic])
    run(command, dry_run=dry_run)
    return archive, ros2_bag


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dest",
        required=True,
        type=Path,
        help="destination directory, preferably on an external SSD",
    )
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument(
        "--no-convert",
        action="store_true",
        help="download and extract only; do not create a ROS 2 MCAP bag",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        archive, result = acquire(
            args.dest,
            url=args.url,
            convert=not args.no_convert,
            dry_run=args.dry_run,
        )
    except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"failed to acquire beach_pond: {exc}", file=sys.stderr)
        return 1
    print(f"archive: {archive}")
    if result is not None:
        print(f"result: {result}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
