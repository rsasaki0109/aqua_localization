#!/usr/bin/env python3
"""Ingest a Tank Dataset AQUA-SLAM run as a benchmark baseline row."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import shutil
import sys

import ingest_external_tum_result
import ros1_odometry_csv_to_tum


DEFAULT_RUNTIME = "AQUA-SLAM Docker image orb_dvl2_ros_noetic"
DEFAULT_SOURCE = "/AQUA_SLAM/orb_odom"


@dataclass(frozen=True)
class BaselinePaths:
    estimate_tum: Path
    benchmark_row: Path
    manifest: Path


def sanitize_name(value: str) -> str:
    return "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in value).strip("_") or "sequence"


def default_paths(out_dir: Path, sequence: str) -> BaselinePaths:
    stem = sanitize_name(sequence)
    return BaselinePaths(
        estimate_tum=out_dir / f"{stem}_aqua_slam.tum",
        benchmark_row=out_dir / f"{stem}_aqua_slam_benchmark_row.md",
        manifest=out_dir / f"{stem}_aqua_slam_baseline.md",
    )


def output_paths(args) -> BaselinePaths:
    defaults = default_paths(args.out_dir, args.sequence)
    return BaselinePaths(
        estimate_tum=args.tum_out or defaults.estimate_tum,
        benchmark_row=args.row_out or defaults.benchmark_row,
        manifest=args.manifest_out or defaults.manifest,
    )


def export_note(args, used_csv: bool) -> str:
    if args.export:
        return args.export
    if used_csv:
        return f"rostopic echo -p {args.source} + ros1_odometry_csv_to_tum.py"
    return "external AQUA-SLAM TUM trajectory"


def prepare_estimate(args, paths: BaselinePaths) -> tuple[Path, bool]:
    if args.csv is not None:
        rows = ros1_odometry_csv_to_tum.convert_rows(args.csv, time_unit=args.time_unit)
        if not rows:
            raise ValueError(f"{args.csv}: no AQUA-SLAM odometry rows found")
        ros1_odometry_csv_to_tum.write_tum(rows, paths.estimate_tum)
        return paths.estimate_tum, True

    if args.tum is None:
        raise ValueError("expected --csv or --tum")
    if args.copy_tum:
        paths.estimate_tum.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(args.tum, paths.estimate_tum)
        return paths.estimate_tum, False
    return args.tum, False


def make_ingest_args(args, estimate: Path, paths: BaselinePaths, used_csv: bool, *, manifest: bool):
    argv = [
        "--reference",
        str(args.reference),
        "--estimate",
        str(estimate),
        "--dataset",
        args.dataset,
        "--sequence",
        args.sequence,
        "--system",
        args.system,
        "--runtime",
        args.runtime,
        "--source",
        args.source,
        "--export",
        export_note(args, used_csv),
        "--out",
        str(paths.manifest if manifest else paths.benchmark_row),
    ]
    if args.config:
        argv.extend(["--config", args.config])
    if args.commit:
        argv.extend(["--commit", args.commit])
    if args.note:
        argv.extend(["--note", args.note])
    if args.scale:
        argv.append("--scale")
    if args.no_align:
        argv.append("--no-align")
    if args.header and not manifest:
        argv.append("--header")
    if manifest:
        argv.extend(["--manifest", "--header"])
    return ingest_external_tum_result.parse_args(argv)


def write_row_and_manifest(args, estimate: Path, paths: BaselinePaths, used_csv: bool) -> tuple[str, str]:
    row_args = make_ingest_args(args, estimate, paths, used_csv, manifest=False)
    row_text = ingest_external_tum_result.make_row(row_args)
    ingest_external_tum_result.write_output(paths.benchmark_row, row_text, args.append_row)

    manifest_args = make_ingest_args(args, estimate, paths, used_csv, manifest=True)
    manifest_text = ingest_external_tum_result.format_manifest(manifest_args)
    ingest_external_tum_result.write_output(paths.manifest, manifest_text, False)
    return row_text, manifest_text


def append_row(path: Path, row_text: str) -> None:
    ingest_external_tum_result.write_output(path, row_text, True)


def format_summary(args, paths: BaselinePaths, estimate: Path, row_text: str) -> str:
    return "\n".join([
        "# Tank AQUA-SLAM Baseline Ingestion",
        "",
        f"- Sequence: `{args.sequence}`",
        f"- Reference: `{args.reference}`",
        f"- Estimate TUM: `{estimate}`",
        f"- Benchmark row: `{paths.benchmark_row}`",
        f"- Manifest: `{paths.manifest}`",
        "",
        "## Benchmark Row",
        "",
        row_text,
        "",
    ])


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Ingest a Tank Dataset AQUA-SLAM CSV/TUM run as a benchmark row."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--csv", type=Path, help="rostopic echo -p CSV from /AQUA_SLAM/orb_odom.")
    source.add_argument("--tum", type=Path, help="Existing AQUA-SLAM TUM trajectory.")
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--system", default="AQUA-SLAM")
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_aqua_slam_baseline"))
    parser.add_argument("--tum-out", type=Path)
    parser.add_argument("--row-out", type=Path)
    parser.add_argument("--manifest-out", type=Path)
    parser.add_argument("--append-row", action="store_true")
    parser.add_argument("--append-to", type=Path, help="Optional Markdown file to append the row to.")
    parser.add_argument("--copy-tum", action="store_true", help="Copy --tum into --tum-out before ingesting.")
    parser.add_argument("--time-unit", choices=("auto", "seconds", "nanoseconds"), default="auto")
    parser.add_argument("--config", default="")
    parser.add_argument("--commit", default="")
    parser.add_argument("--runtime", default=DEFAULT_RUNTIME)
    parser.add_argument("--source", default=DEFAULT_SOURCE)
    parser.add_argument("--export", default="")
    parser.add_argument("--note", default="")
    parser.add_argument("--scale", action="store_true")
    parser.add_argument("--no-align", action="store_true")
    parser.add_argument(
        "--header",
        dest="header",
        action="store_true",
        default=True,
        help="Include a Markdown table header in the standalone benchmark row.",
    )
    parser.add_argument(
        "--no-header",
        dest="header",
        action="store_false",
        help="Write only the data row in the standalone benchmark row.",
    )
    return parser.parse_args(argv)


def validate_args(args) -> None:
    if not args.reference.exists():
        raise ValueError(f"reference not found: {args.reference}")
    if args.csv is not None and not args.csv.exists():
        raise ValueError(f"CSV not found: {args.csv}")
    if args.tum is not None and not args.tum.exists():
        raise ValueError(f"TUM not found: {args.tum}")
    if args.append_to is not None and not args.append_to.exists():
        raise ValueError(f"append target not found: {args.append_to}")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    paths = output_paths(args)
    try:
        validate_args(args)
        args.out_dir.mkdir(parents=True, exist_ok=True)
        estimate, used_csv = prepare_estimate(args, paths)
        row_text, _manifest_text = write_row_and_manifest(args, estimate, paths, used_csv)
        if args.append_to is not None:
            append_row(args.append_to, row_text)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(format_summary(args, paths, estimate, row_text))
    return 0


if __name__ == "__main__":
    sys.exit(main())
