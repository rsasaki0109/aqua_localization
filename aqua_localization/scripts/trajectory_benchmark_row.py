#!/usr/bin/env python3
"""Generate a Markdown benchmark row from two TUM trajectories.

This is a thin reporting wrapper around compare_trajectories.py. It keeps
baseline comparisons consistent across aqua_localization, AQUA-SLAM, and other
external systems once their estimated trajectory has been exported to TUM.
"""

import argparse
import importlib.util
import sys
from pathlib import Path


def load_compare_module():
    script_path = Path(__file__).resolve().parent / "compare_trajectories.py"
    spec = importlib.util.spec_from_file_location("compare_trajectories", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def escape_markdown_cell(value) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def alignment_label(stats: dict) -> str:
    alignment = stats["alignment"]
    if not alignment["applied"]:
        return "raw"
    return "Sim(3)" if alignment["with_scale"] else "SE(3)"


def format_row(args, stats: dict) -> str:
    cells = [
        args.dataset,
        args.sequence,
        args.system,
        alignment_label(stats),
        stats["count"],
        f"{stats['matched_seconds']:.2f}",
        f"{stats['mean']:.4f}",
        f"{stats['median']:.4f}",
        f"{stats['rmse']:.4f}",
        f"{stats['max']:.4f}",
        args.note,
    ]
    return "| " + " | ".join(escape_markdown_cell(c) for c in cells) + " |"


def table_header() -> str:
    return "\n".join(
        [
            "| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |",
            "|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|",
        ]
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Compare two TUM trajectories and emit one Markdown benchmark row."
    )
    parser.add_argument("--reference", required=True, type=Path, help="Reference TUM trajectory.")
    parser.add_argument("--estimate", required=True, type=Path, help="Estimated TUM trajectory.")
    parser.add_argument("--dataset", required=True, help="Dataset name for the Markdown row.")
    parser.add_argument("--sequence", required=True, help="Sequence name for the Markdown row.")
    parser.add_argument("--system", required=True, help="Estimator/system name for the Markdown row.")
    parser.add_argument("--note", default="", help="Short note for the Markdown row.")
    parser.add_argument("--scale", action="store_true", help="Use Sim(3) alignment instead of rigid SE(3).")
    parser.add_argument("--no-align", action="store_true", help="Compare raw positions without alignment.")
    parser.add_argument("--header", action="store_true", help="Print the Markdown table header before the row.")
    parser.add_argument("--out", type=Path, default=None, help="Optional file to write or append the row to.")
    parser.add_argument("--append", action="store_true", help="Append to --out instead of replacing it.")
    return parser.parse_args(argv)


def write_output(path: Path, text: str, append: bool):
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = "a" if append else "w"
    with path.open(mode, encoding="utf-8") as fp:
        if append and path.exists() and path.stat().st_size > 0:
            fp.write("\n")
        fp.write(text)
        fp.write("\n")


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    compare_module = load_compare_module()
    stats, _ = compare_module.compare(args.reference, args.estimate, args.scale, args.no_align)

    parts = []
    if args.header:
        parts.append(table_header())
    parts.append(format_row(args, stats))
    text = "\n".join(parts)

    if args.out is not None:
        write_output(args.out, text, args.append)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
