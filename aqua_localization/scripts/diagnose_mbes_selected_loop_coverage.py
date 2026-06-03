#!/usr/bin/env python3
"""Diagnose whether selected MBES loop IDs were reachable in a replay."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
import sys


NO_CANDIDATE_ID = 2**32 - 1


@dataclass(frozen=True)
class SelectedLoop:
    timestamp: float
    current_id: int
    candidate_id: int
    status: str


@dataclass(frozen=True)
class StatusRow:
    timestamp: float
    current_id: int
    candidate_id: int
    accepted: bool
    status: str
    fitness_score: str
    correction_translation_m: str
    correction_rotation_rad: str

    @property
    def pair(self) -> tuple[int, int]:
        return (self.candidate_id, self.current_id)

    @property
    def has_candidate(self) -> bool:
        return self.candidate_id != NO_CANDIDATE_ID

    @property
    def accepted_looking(self) -> bool:
        return self.accepted or self.status == "loop selection rejected"


@dataclass(frozen=True)
class CoverageRow:
    selected: SelectedLoop
    exact_rows: list[StatusRow]
    nearby_rows: list[StatusRow]
    nearest_row: StatusRow | None


def truthy(value: str | None) -> bool:
    return value in {"1", "true", "True", "TRUE", "yes", "Yes"}


def read_selected(path: Path) -> list[SelectedLoop]:
    loops: list[SelectedLoop] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"timestamp", "current_id", "candidate_id"} - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                loops.append(
                    SelectedLoop(
                        timestamp=float(row["timestamp"]),
                        current_id=int(row["current_id"]),
                        candidate_id=int(row["candidate_id"]),
                        status=row.get("status", ""),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_number}: invalid selected loop row") from exc
    return loops


def read_status(path: Path) -> list[StatusRow]:
    rows: list[StatusRow] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"timestamp", "current_id", "candidate_id", "accepted", "status"} - set(
            reader.fieldnames
        )
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                status = row.get("status", "")
                accepted = truthy(row.get("accepted")) or status == "accepted"
                rows.append(
                    StatusRow(
                        timestamp=float(row["timestamp"]),
                        current_id=int(row["current_id"]),
                        candidate_id=int(row["candidate_id"]),
                        accepted=accepted,
                        status=status,
                        fitness_score=row.get("fitness_score", ""),
                        correction_translation_m=row.get("correction_translation_m", ""),
                        correction_rotation_rad=row.get("correction_rotation_rad", ""),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_number}: invalid status row") from exc
    return rows


def coverage_rows(
    selected: list[SelectedLoop],
    status_rows: list[StatusRow],
    timestamp_window_s: float,
) -> list[CoverageRow]:
    if timestamp_window_s < 0.0:
        raise ValueError("timestamp_window_s must be non-negative")
    exact: dict[tuple[int, int], list[StatusRow]] = {}
    for row in status_rows:
        exact.setdefault(row.pair, []).append(row)

    rows: list[CoverageRow] = []
    for loop in selected:
        nearby = [
            row
            for row in status_rows
            if abs(row.timestamp - loop.timestamp) <= timestamp_window_s
        ]
        nearest = min(
            status_rows,
            key=lambda row: abs(row.timestamp - loop.timestamp),
            default=None,
        )
        rows.append(
            CoverageRow(
                selected=loop,
                exact_rows=exact.get((loop.candidate_id, loop.current_id), []),
                nearby_rows=nearby,
                nearest_row=nearest,
            )
        )
    return rows


def format_float(value: float, digits: int = 3) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def pair_text(candidate_id: int, current_id: int) -> str:
    if candidate_id == NO_CANDIDATE_ID:
        return f"none -> {current_id}"
    return f"{candidate_id} -> {current_id}"


def exact_status_text(rows: list[StatusRow]) -> str:
    if not rows:
        return "missing"
    counts = Counter(row.status for row in rows)
    parts = [f"{status or 'blank'}:{count}" for status, count in counts.most_common()]
    return ", ".join(parts)


def format_report(
    selected_path: Path,
    status_path: Path,
    selected: list[SelectedLoop],
    status_rows: list[StatusRow],
    rows: list[CoverageRow],
    timestamp_window_s: float,
    max_rows: int,
) -> str:
    exact_observed = sum(1 for row in rows if row.exact_rows)
    exact_accepted = sum(1 for row in rows if any(status.accepted for status in row.exact_rows))
    exact_rejected = sum(
        1 for row in rows if row.exact_rows and not any(status.accepted for status in row.exact_rows)
    )
    exact_missing = len(rows) - exact_observed
    nearby_with_candidates = sum(
        1 for row in rows if any(status.has_candidate for status in row.nearby_rows)
    )
    nearby_accepted_looking = sum(
        1 for row in rows if any(status.accepted_looking for status in row.nearby_rows)
    )
    replay_accepted = [row for row in status_rows if row.accepted]
    replay_selection_rejected = [row for row in status_rows if row.status == "loop selection rejected"]

    lines = [
        "# MBES Selected-Loop Replay Coverage",
        "",
        f"- Selected loop CSV: `{selected_path}`",
        f"- Selected replay status CSV: `{status_path}`",
        f"- Timestamp window: {format_float(timestamp_window_s, 3)} s",
        f"- Allowlisted loops: {len(selected)}",
        f"- Exact pairs observed: {exact_observed}",
        f"- Exact pairs accepted: {exact_accepted}",
        f"- Exact pairs rejected: {exact_rejected}",
        f"- Exact pairs missing: {exact_missing}",
        f"- Allowlist rows with any replay candidate in time window: {nearby_with_candidates}",
        f"- Allowlist rows with accepted-looking candidate in time window: {nearby_accepted_looking}",
        f"- Replay accepted loops: {len(replay_accepted)}",
        f"- Replay `loop selection rejected` rows: {len(replay_selection_rejected)}",
        "",
        "## Diagnosis",
        "",
    ]

    if selected and exact_accepted == len(selected):
        lines.append("PASS: every selected loop was observed and accepted by exact ID.")
    elif exact_missing == len(selected) and nearby_with_candidates == 0:
        lines.append(
            "MISS: selected loop source times did not overlap replay candidate generation. "
            "Investigate replay coverage, keyframe/submap timing, and startup determinism before "
            "tuning loop gates."
        )
    elif nearby_accepted_looking > exact_accepted:
        lines.append(
            "DRIFT: replay produced accepted-looking candidates near selected source times, "
            "but exact selected IDs did not match. Stable loop identity or descriptor-level "
            "selection is needed before using this as trajectory evidence."
        )
    elif exact_observed > exact_accepted:
        lines.append(
            "GATED: selected IDs were replayed but failed downstream gates. Inspect the exact "
            "status values before changing the selector."
        )
    else:
        lines.append(
            "INCONCLUSIVE: selected IDs were not accepted. Inspect the per-loop coverage table "
            "and replay status counts."
        )
    lines.append("")

    lines.extend(
        [
            "## Per-Loop Coverage",
            "",
            "| Rank | Source time s | Selected pair | Exact status | Nearest dt s | Nearest pair | Nearest status | Candidates in window | Accepted-looking in window |",
            "|-----:|--------------:|---------------|--------------|-------------:|--------------|----------------|---------------------:|---------------------------:|",
        ]
    )
    for rank, row in enumerate(rows[:max_rows], start=1):
        nearest = row.nearest_row
        nearest_dt = (
            nearest.timestamp - row.selected.timestamp if nearest is not None else math.nan
        )
        candidate_count = sum(1 for status in row.nearby_rows if status.has_candidate)
        accepted_looking_count = sum(
            1 for status in row.nearby_rows if status.accepted_looking
        )
        lines.append(
            f"| {rank} | {format_float(row.selected.timestamp, 3)} | "
            f"{pair_text(row.selected.candidate_id, row.selected.current_id)} | "
            f"{exact_status_text(row.exact_rows)} | {format_float(nearest_dt, 3)} | "
            f"{pair_text(nearest.candidate_id, nearest.current_id) if nearest else 'n/a'} | "
            f"{nearest.status if nearest else 'n/a'} | {candidate_count} | "
            f"{accepted_looking_count} |"
        )
    if len(rows) > max_rows:
        lines.extend(["", f"Only the first {max_rows} selected rows are shown."])
    lines.append("")

    if replay_selection_rejected:
        lines.extend(
            [
                "## Accepted-Looking Rows Rejected By Selection",
                "",
                "| Rank | Time s | Candidate -> Current | Fitness | Correction m | Rotation rad |",
                "|-----:|-------:|----------------------|--------:|-------------:|-------------:|",
            ]
        )
        for rank, row in enumerate(replay_selection_rejected[:max_rows], start=1):
            lines.append(
                f"| {rank} | {format_float(row.timestamp, 3)} | "
                f"{pair_text(row.candidate_id, row.current_id)} | "
                f"{row.fitness_score} | {row.correction_translation_m} | "
                f"{row.correction_rotation_rad} |"
            )
        if len(replay_selection_rejected) > max_rows:
            lines.extend(
                [
                    "",
                    f"Only the first {max_rows} selection-rejected rows are shown.",
                ]
            )
        lines.append("")

    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--selected",
        required=True,
        type=Path,
        help="Batch-selected loop CSV used as loop.selection.allowlist_csv",
    )
    parser.add_argument(
        "--status",
        required=True,
        type=Path,
        help="Selected replay mbes_beach_pond_loop_status.csv",
    )
    parser.add_argument("--out", type=Path, help="Optional Markdown output path")
    parser.add_argument(
        "--timestamp-window-s",
        type=float,
        default=1.0,
        help="Window around each selected source timestamp for replay coverage checks",
    )
    parser.add_argument("--max-rows", type=int, default=50)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        selected = read_selected(args.selected)
        status_rows = read_status(args.status)
        rows = coverage_rows(selected, status_rows, args.timestamp_window_s)
        report = format_report(
            args.selected,
            args.status,
            selected,
            status_rows,
            rows,
            args.timestamp_window_s,
            args.max_rows,
        )
    except (OSError, ValueError) as exc:
        print(f"failed to diagnose MBES selected-loop coverage: {exc}", file=sys.stderr)
        return 1

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
        print(f"wrote MBES selected-loop coverage report to {args.out}")
    else:
        print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
