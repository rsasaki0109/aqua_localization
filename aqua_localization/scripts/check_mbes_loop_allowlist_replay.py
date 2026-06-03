#!/usr/bin/env python3
"""Audit whether a selected-loop replay accepted only allowlisted loop IDs."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
import sys


@dataclass(frozen=True)
class LoopStatusRow:
    timestamp: str
    current_id: int
    candidate_id: int
    accepted: bool
    status: str
    fitness_score: str
    correction_translation_m: str
    correction_rotation_rad: str


def truthy(value: str | None) -> bool:
    return value in {"1", "true", "True", "TRUE", "yes", "Yes"}


def read_allowlist(path: Path) -> set[tuple[int, int]]:
    pairs: set[tuple[int, int]] = set()
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"current_id", "candidate_id"} - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                current_id = int(row["current_id"])
                candidate_id = int(row["candidate_id"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_number}: non-numeric loop IDs") from exc
            pairs.add((candidate_id, current_id))
    return pairs


def read_status_rows(path: Path) -> list[LoopStatusRow]:
    rows: list[LoopStatusRow] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"current_id", "candidate_id", "accepted", "status"} - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                rows.append(
                    LoopStatusRow(
                        timestamp=row.get("timestamp", ""),
                        current_id=int(row["current_id"]),
                        candidate_id=int(row["candidate_id"]),
                        accepted=truthy(row.get("accepted")) or row.get("status") == "accepted",
                        status=row.get("status", ""),
                        fitness_score=row.get("fitness_score", ""),
                        correction_translation_m=row.get("correction_translation_m", ""),
                        correction_rotation_rad=row.get("correction_rotation_rad", ""),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_number}: non-numeric loop IDs") from exc
    return rows


def accepted_rows(rows: list[LoopStatusRow]) -> list[LoopStatusRow]:
    return [row for row in rows if row.accepted]


def off_allowlist_rows(
    rows: list[LoopStatusRow],
    allowlist: set[tuple[int, int]],
) -> list[LoopStatusRow]:
    return [
        row for row in accepted_rows(rows)
        if (row.candidate_id, row.current_id) not in allowlist
    ]


def format_report(
    allowlist_path: Path,
    status_path: Path,
    allowlist: set[tuple[int, int]],
    rows: list[LoopStatusRow],
    max_rows: int,
) -> str:
    accepted = accepted_rows(rows)
    off_rows = off_allowlist_rows(rows, allowlist)
    allowed_accepted = len(accepted) - len(off_rows)
    lines = [
        "# MBES Selected-Loop Allowlist Replay Audit",
        "",
        f"- Allowlist CSV: `{allowlist_path}`",
        f"- Selected replay status CSV: `{status_path}`",
        f"- Allowlisted pairs: {len(allowlist)}",
        f"- Accepted replay loops: {len(accepted)}",
        f"- Accepted allowlisted loops: {allowed_accepted}",
        f"- Accepted outside allowlist: {len(off_rows)}",
        "",
    ]
    if not off_rows:
        lines.extend([
            "## Result",
            "",
            "PASS: every accepted selected-replay loop matched the allowlist exactly.",
            "",
        ])
        return "\n".join(lines)

    lines.extend([
        "## Result",
        "",
        "FAIL: selected replay accepted loop pairs that were not present in the allowlist.",
        "",
        "## Off-Allowlist Accepted Loops",
        "",
        "| Rank | Time s | Candidate -> Current | Status | Fitness | Correction m | Rotation rad |",
        "|-----:|--------|----------------------|--------|--------:|-------------:|-------------:|",
    ])
    for rank, row in enumerate(off_rows[:max_rows], start=1):
        lines.append(
            f"| {rank} | {row.timestamp} | {row.candidate_id} -> {row.current_id} | "
            f"{row.status} | {row.fitness_score} | {row.correction_translation_m} | "
            f"{row.correction_rotation_rad} |"
        )
    remaining = len(off_rows) - max_rows
    if remaining > 0:
        lines.extend(["", f"Only the first {max_rows} rows are shown; {remaining} more omitted."])
    lines.append("")
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allowlist", required=True, type=Path,
                        help="CSV used for loop.selection.allowlist_csv")
    parser.add_argument("--status", required=True, type=Path,
                        help="Selected replay mbes_beach_pond_loop_status.csv")
    parser.add_argument("--out", type=Path, help="Optional Markdown output path")
    parser.add_argument("--max-rows", type=int, default=50,
                        help="Maximum off-allowlist rows to show")
    parser.add_argument("--strict", action="store_true",
                        help="Exit non-zero if any accepted loop is outside the allowlist")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        allowlist = read_allowlist(args.allowlist)
        rows = read_status_rows(args.status)
        report = format_report(args.allowlist, args.status, allowlist, rows, args.max_rows)
        off_rows = off_allowlist_rows(rows, allowlist)
    except (OSError, ValueError) as exc:
        print(f"failed to audit MBES loop allowlist replay: {exc}", file=sys.stderr)
        return 1

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
        print(f"wrote MBES selected-loop allowlist audit to {args.out}")
    else:
        print(report)

    if args.strict and off_rows:
        print(
            f"selected replay accepted {len(off_rows)} loop(s) outside the allowlist",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
