#!/usr/bin/env python3
"""Generate a Markdown benchmark row from MBES loop-status CSV output."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Iterable


NO_CANDIDATE_ID = 2**32 - 1


@dataclass(frozen=True)
class LoopStatusRow:
    candidate_id: int
    accepted: bool
    converged: bool
    fitness_score: float
    correction_translation_m: float
    status: str


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def parse_int(value: str, default: int = 0) -> int:
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return default


def parse_float(value: str) -> float:
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return math.nan


def read_loop_status_csv(path: Path) -> list[LoopStatusRow]:
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        rows = []
        for row in reader:
            rows.append(
                LoopStatusRow(
                    candidate_id=parse_int(row.get("candidate_id", "")),
                    accepted=parse_bool(row.get("accepted", "")),
                    converged=parse_bool(row.get("converged", "")),
                    fitness_score=parse_float(row.get("fitness_score", "")),
                    correction_translation_m=parse_float(
                        row.get("correction_translation_m", "")
                    ),
                    status=str(row.get("status", "")),
                )
            )
    return rows


def is_no_candidate(row: LoopStatusRow) -> bool:
    status = row.status.lower()
    return "no candidate" in status or (
        row.candidate_id == NO_CANDIDATE_ID and not status
    )


def finite_values(values: Iterable[float]) -> list[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    if q <= 0.0:
        return min(values)
    if q >= 1.0:
        return max(values)
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    alpha = pos - lo
    return ordered[lo] * (1.0 - alpha) + ordered[hi] * alpha


def summarize_rows(rows: list[LoopStatusRow]) -> dict[str, float | int]:
    no_candidate = [row for row in rows if is_no_candidate(row)]
    accepted = [row for row in rows if row.accepted]
    rejected = [
        row for row in rows
        if not row.accepted and not is_no_candidate(row)
    ]
    fitness_values = finite_values(row.fitness_score for row in rows)
    correction_values = finite_values(row.correction_translation_m for row in rows)
    return {
        "samples": len(rows),
        "accepted": len(accepted),
        "rejected": len(rejected),
        "no_candidate": len(no_candidate),
        "converged": sum(1 for row in rows if row.converged),
        "median_fitness": percentile(fitness_values, 0.5),
        "p95_correction_m": percentile(correction_values, 0.95),
    }


def escape_cell(value: str) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def format_number(value: float | int | str, precision: int = 4) -> str:
    if isinstance(value, int):
        return str(value)
    if isinstance(value, str):
        return value
    if not math.isfinite(float(value)):
        return "TBD"
    if float(value).is_integer():
        return str(int(value))
    return f"{float(value):.{precision}f}"


def table_header() -> str:
    return "\n".join(
        [
            "| Dataset | Sequence | Duration s | Status samples | Accepted | Rejected | No candidate | Converged | Median fitness | P95 correction m | Notes |",
            "|---------|----------|-----------:|---------------:|---------:|---------:|-------------:|----------:|---------------:|-----------------:|-------|",
        ]
    )


def format_row(args, summary: dict[str, float | int]) -> str:
    duration = "TBD" if args.duration is None else format_number(args.duration, precision=2)
    cells = [
        args.dataset,
        f"`{args.sequence}`",
        duration,
        format_number(summary["samples"]),
        format_number(summary["accepted"]),
        format_number(summary["rejected"]),
        format_number(summary["no_candidate"]),
        format_number(summary["converged"]),
        format_number(summary["median_fitness"]),
        format_number(summary["p95_correction_m"]),
        args.note,
    ]
    return "| " + " | ".join(escape_cell(cell) for cell in cells) + " |"


def write_output(path: Path, text: str, append: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if append and path.exists() and path.stat().st_size > 0:
        with path.open("a", encoding="utf-8") as fp:
            fp.write("\n")
            fp.write(text)
            fp.write("\n")
        return
    path.write_text(text + "\n", encoding="utf-8")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Generate a Markdown benchmark row from MBES loop-status CSV."
    )
    parser.add_argument("--csv", required=True, type=Path, help="Loop-status CSV.")
    parser.add_argument("--dataset", required=True, help="Dataset name.")
    parser.add_argument("--sequence", required=True, help="Sequence name.")
    parser.add_argument("--duration", type=float, help="Replay duration in seconds.")
    parser.add_argument("--note", default="", help="Short note for the row.")
    parser.add_argument("--header", action="store_true", help="Print the table header.")
    parser.add_argument("--out", type=Path, help="Optional output Markdown file.")
    parser.add_argument("--append", action="store_true", help="Append to --out.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        rows = read_loop_status_csv(args.csv)
    except OSError as exc:
        print(f"failed to read MBES loop-status CSV: {exc}", file=sys.stderr)
        return 2

    parts = []
    if args.header:
        parts.append(table_header())
    parts.append(format_row(args, summarize_rows(rows)))
    text = "\n".join(parts)

    if args.out:
        write_output(args.out, text, args.append)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
