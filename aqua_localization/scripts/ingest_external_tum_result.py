#!/usr/bin/env python3
"""Ingest an external SLAM TUM trajectory into the benchmark table format.

This wrapper is meant for AQUA-SLAM and other systems that are run outside this
ROS 2 workspace. Once their odometry has been exported to TUM, this script uses
the same APE implementation as the rest of the repository and records enough
provenance in the Markdown note to make the row reproducible.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from types import SimpleNamespace

import trajectory_benchmark_row


def note_parts(args) -> list[str]:
    parts = []
    if args.note:
        parts.append(args.note)
    if args.config:
        parts.append(f"config={args.config}")
    if args.commit:
        parts.append(f"commit={args.commit}")
    if args.runtime:
        parts.append(f"runtime={args.runtime}")
    if args.source:
        parts.append(f"source={args.source}")
    if args.export:
        parts.append(f"export={args.export}")
    return parts


def build_note(args) -> str:
    parts = note_parts(args)
    return "; ".join(parts) if parts else "external TUM result"


def make_row(args) -> str:
    compare_module = trajectory_benchmark_row.load_compare_module()
    stats, _ = compare_module.compare(args.reference, args.estimate, args.scale, args.no_align)
    row_args = SimpleNamespace(
        dataset=args.dataset,
        sequence=args.sequence,
        system=args.system,
        note=build_note(args),
    )
    parts = []
    if args.header:
        parts.append(trajectory_benchmark_row.table_header())
    parts.append(trajectory_benchmark_row.format_row(row_args, stats))
    return "\n".join(parts)


def format_manifest(args) -> str:
    alignment = "raw" if args.no_align else "Sim(3)" if args.scale else "SE(3)"
    lines = [
        "# External TUM Result",
        "",
        f"- Dataset: `{args.dataset}`",
        f"- Sequence: `{args.sequence}`",
        f"- System: `{args.system}`",
        f"- Reference TUM: `{args.reference}`",
        f"- Estimate TUM: `{args.estimate}`",
        f"- Alignment: `{alignment}`",
    ]
    if args.config:
        lines.append(f"- Config: `{args.config}`")
    if args.commit:
        lines.append(f"- Commit: `{args.commit}`")
    if args.runtime:
        lines.append(f"- Runtime: {args.runtime}")
    if args.source:
        lines.append(f"- Source: {args.source}")
    if args.export:
        lines.append(f"- Export: {args.export}")
    if args.note:
        lines.append(f"- Note: {args.note}")
    lines.extend(["", "## Benchmark Row", "", make_row(args)])
    return "\n".join(lines) + "\n"


def write_output(path: Path, text: str, append: bool):
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = "a" if append else "w"
    with path.open(mode, encoding="utf-8") as fp:
        if append and path.exists() and path.stat().st_size > 0:
            fp.write("\n")
        fp.write(text)
        if not text.endswith("\n"):
            fp.write("\n")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Compare an external SLAM TUM trajectory and emit a reproducible Markdown row."
    )
    parser.add_argument("--reference", required=True, type=Path, help="Reference TUM trajectory.")
    parser.add_argument("--estimate", required=True, type=Path, help="External system TUM trajectory.")
    parser.add_argument("--dataset", required=True, help="Dataset name for the Markdown row.")
    parser.add_argument("--sequence", required=True, help="Sequence name for the Markdown row.")
    parser.add_argument("--system", required=True, help="External system name, e.g. AQUA-SLAM.")
    parser.add_argument("--config", default="", help="External system config path or identifier.")
    parser.add_argument("--commit", default="", help="External system git commit or release tag.")
    parser.add_argument("--runtime", default="", help="Runtime note, e.g. Docker image or replay rate.")
    parser.add_argument("--source", default="", help="Source repository, artifact, or run log URL.")
    parser.add_argument("--export", default="", help="How the estimate TUM was exported.")
    parser.add_argument("--note", default="", help="Additional row note.")
    parser.add_argument("--scale", action="store_true", help="Use Sim(3) alignment.")
    parser.add_argument("--no-align", action="store_true", help="Compare raw positions without alignment.")
    parser.add_argument("--header", action="store_true", help="Include the benchmark table header.")
    parser.add_argument("--manifest", action="store_true", help="Emit a provenance manifest plus row.")
    parser.add_argument("--out", type=Path, default=None, help="Optional Markdown output path.")
    parser.add_argument("--append", action="store_true", help="Append to --out instead of replacing.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    text = format_manifest(args) if args.manifest else make_row(args)
    if args.out is not None:
        write_output(args.out, text, args.append)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
