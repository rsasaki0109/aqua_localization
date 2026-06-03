#!/usr/bin/env python3
"""Remap selected MBES loop IDs onto accepted-looking rows from a target replay."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys


NO_CANDIDATE_ID = 2**32 - 1
ACCEPTED_LOOKING_STATUS = {"accepted", "loop selection rejected"}


@dataclass(frozen=True)
class SelectedLoop:
    timestamp: float
    current_id: int
    candidate_id: int
    fitness_score: float
    correction_translation_m: float
    correction_rotation_rad: float
    status: str


@dataclass(frozen=True)
class StatusRow:
    timestamp: float
    current_id: int
    candidate_id: int
    accepted: bool
    status: str
    fitness_score: float
    correction_translation_m: float
    correction_rotation_rad: float

    @property
    def pair(self) -> tuple[int, int]:
        return (self.candidate_id, self.current_id)

    @property
    def has_candidate(self) -> bool:
        return self.candidate_id != NO_CANDIDATE_ID

    @property
    def accepted_looking(self) -> bool:
        return self.accepted or self.status in ACCEPTED_LOOKING_STATUS


@dataclass(frozen=True)
class RemapMatch:
    source: SelectedLoop
    target: StatusRow
    score: float
    time_delta_s: float


def truthy(value: str | None) -> bool:
    return value in {"1", "true", "True", "TRUE", "yes", "Yes"}


def parse_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def finite_delta(a: float, b: float) -> float:
    if math.isfinite(a) and math.isfinite(b):
        return abs(a - b)
    return 0.0


def format_float(value: float, digits: int = 9) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.{digits}f}"


def read_selected(path: Path) -> list[SelectedLoop]:
    rows: list[SelectedLoop] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"timestamp", "current_id", "candidate_id"} - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                rows.append(
                    SelectedLoop(
                        timestamp=float(row["timestamp"]),
                        current_id=int(row["current_id"]),
                        candidate_id=int(row["candidate_id"]),
                        fitness_score=parse_float(row.get("fitness_score")),
                        correction_translation_m=parse_float(
                            row.get("correction_translation_m")
                        ),
                        correction_rotation_rad=parse_float(
                            row.get("correction_rotation_rad")
                        ),
                        status=row.get("status", ""),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_number}: invalid selected loop row") from exc
    return rows


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
                        fitness_score=parse_float(row.get("fitness_score")),
                        correction_translation_m=parse_float(
                            row.get("correction_translation_m")
                        ),
                        correction_rotation_rad=parse_float(
                            row.get("correction_rotation_rad")
                        ),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_number}: invalid status row") from exc
    return rows


def score_candidate(source: SelectedLoop, target: StatusRow, timestamp_window_s: float) -> float:
    window = max(timestamp_window_s, 1.0e-9)
    return (
        abs(target.timestamp - source.timestamp) / window
        + finite_delta(source.fitness_score, target.fitness_score)
        + finite_delta(source.correction_translation_m, target.correction_translation_m)
        + 5.0 * finite_delta(source.correction_rotation_rad, target.correction_rotation_rad)
    )


def match_selected_loops(
    selected: list[SelectedLoop],
    status_rows: list[StatusRow],
    timestamp_window_s: float,
) -> tuple[list[RemapMatch], list[SelectedLoop]]:
    if timestamp_window_s < 0.0:
        raise ValueError("timestamp_window_s must be non-negative")
    reusable_targets = [
        row for row in status_rows if row.has_candidate and row.accepted_looking
    ]
    used_pairs: set[tuple[int, int]] = set()
    matches: list[RemapMatch] = []
    unmatched: list[SelectedLoop] = []

    for source in selected:
        candidates = [
            target
            for target in reusable_targets
            if target.pair not in used_pairs
            and abs(target.timestamp - source.timestamp) <= timestamp_window_s
        ]
        if not candidates:
            unmatched.append(source)
            continue
        target = min(
            candidates,
            key=lambda row: (
                score_candidate(source, row, timestamp_window_s),
                abs(row.timestamp - source.timestamp),
                row.current_id,
                row.candidate_id,
            ),
        )
        used_pairs.add(target.pair)
        matches.append(
            RemapMatch(
                source=source,
                target=target,
                score=score_candidate(source, target, timestamp_window_s),
                time_delta_s=target.timestamp - source.timestamp,
            )
        )

    return matches, unmatched


def write_remapped_csv(path: Path, matches: list[RemapMatch]) -> None:
    fields = [
        "timestamp",
        "current_id",
        "candidate_id",
        "source_timestamp",
        "source_current_id",
        "source_candidate_id",
        "time_delta_s",
        "score",
        "fitness_score",
        "correction_translation_m",
        "correction_rotation_rad",
        "target_status",
        "source_fitness_score",
        "source_correction_translation_m",
        "source_correction_rotation_rad",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        for match in matches:
            writer.writerow(
                {
                    "timestamp": format_float(match.target.timestamp),
                    "current_id": match.target.current_id,
                    "candidate_id": match.target.candidate_id,
                    "source_timestamp": format_float(match.source.timestamp),
                    "source_current_id": match.source.current_id,
                    "source_candidate_id": match.source.candidate_id,
                    "time_delta_s": format_float(match.time_delta_s),
                    "score": format_float(match.score),
                    "fitness_score": format_float(match.target.fitness_score),
                    "correction_translation_m": format_float(
                        match.target.correction_translation_m
                    ),
                    "correction_rotation_rad": format_float(
                        match.target.correction_rotation_rad
                    ),
                    "target_status": match.target.status,
                    "source_fitness_score": format_float(match.source.fitness_score),
                    "source_correction_translation_m": format_float(
                        match.source.correction_translation_m
                    ),
                    "source_correction_rotation_rad": format_float(
                        match.source.correction_rotation_rad
                    ),
                }
            )


def pair_text(candidate_id: int, current_id: int) -> str:
    return f"{candidate_id} -> {current_id}"


def format_report(
    selected_path: Path,
    status_path: Path,
    out_path: Path,
    selected: list[SelectedLoop],
    status_rows: list[StatusRow],
    matches: list[RemapMatch],
    unmatched: list[SelectedLoop],
    timestamp_window_s: float,
    max_rows: int,
) -> str:
    accepted_looking = [
        row for row in status_rows if row.has_candidate and row.accepted_looking
    ]
    lines = [
        "# MBES Selected-Loop Remap",
        "",
        f"- Source selected CSV: `{selected_path}`",
        f"- Target replay status CSV: `{status_path}`",
        f"- Remapped allowlist CSV: `{out_path}`",
        f"- Timestamp window: {timestamp_window_s:.3f} s",
        f"- Source selected loops: {len(selected)}",
        f"- Target accepted-looking rows: {len(accepted_looking)}",
        f"- Remapped loops: {len(matches)}",
        f"- Unmatched source loops: {len(unmatched)}",
        "",
        "## Diagnosis",
        "",
    ]
    if not selected:
        lines.append("EMPTY: the source selected CSV has no data rows.")
    elif not matches:
        lines.append(
            "MISS: no selected loops could be remapped onto accepted-looking target rows "
            "within the timestamp window."
        )
    elif unmatched:
        lines.append(
            "PARTIAL: some selected loops remapped to target IDs, but the set is incomplete. "
            "Use this CSV only as a diagnostic second-pass allowlist."
        )
    else:
        lines.append(
            "PASS: every selected loop was remapped to an accepted-looking target row. "
            "This is still a replay diagnostic, not a trajectory claim."
        )
    lines.append("")

    lines.extend(
        [
            "## Remapped Loops",
            "",
            "| Rank | Source pair | Target pair | Source time s | Target time s | dt s | Target status | Score |",
            "|-----:|-------------|-------------|--------------:|--------------:|-----:|---------------|------:|",
        ]
    )
    for rank, match in enumerate(matches[:max_rows], start=1):
        lines.append(
            f"| {rank} | {pair_text(match.source.candidate_id, match.source.current_id)} | "
            f"{pair_text(match.target.candidate_id, match.target.current_id)} | "
            f"{match.source.timestamp:.3f} | {match.target.timestamp:.3f} | "
            f"{match.time_delta_s:.3f} | {match.target.status} | {match.score:.6f} |"
        )
    if len(matches) > max_rows:
        lines.extend(["", f"Only the first {max_rows} remapped rows are shown."])
    lines.append("")

    if unmatched:
        lines.extend(
            [
                "## Unmatched Source Loops",
                "",
                "| Rank | Source pair | Source time s | Source status |",
                "|-----:|-------------|--------------:|---------------|",
            ]
        )
        for rank, source in enumerate(unmatched[:max_rows], start=1):
            lines.append(
                f"| {rank} | {pair_text(source.candidate_id, source.current_id)} | "
                f"{source.timestamp:.3f} | {source.status} |"
            )
        if len(unmatched) > max_rows:
            lines.extend(["", f"Only the first {max_rows} unmatched rows are shown."])
        lines.append("")
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--selected",
        required=True,
        type=Path,
        help="Source selected loop CSV from the normal replay",
    )
    parser.add_argument(
        "--status",
        required=True,
        type=Path,
        help="Target replay loop-status CSV",
    )
    parser.add_argument(
        "--out",
        required=True,
        type=Path,
        help="Remapped allowlist CSV to write",
    )
    parser.add_argument("--report-out", type=Path, help="Optional Markdown report")
    parser.add_argument("--timestamp-window-s", type=float, default=1.0)
    parser.add_argument("--max-rows", type=int, default=50)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        selected = read_selected(args.selected)
        status_rows = read_status(args.status)
        matches, unmatched = match_selected_loops(
            selected,
            status_rows,
            args.timestamp_window_s,
        )
        write_remapped_csv(args.out, matches)
        report = format_report(
            args.selected,
            args.status,
            args.out,
            selected,
            status_rows,
            matches,
            unmatched,
            args.timestamp_window_s,
            args.max_rows,
        )
    except (OSError, ValueError) as exc:
        print(f"failed to remap MBES selected loops: {exc}", file=sys.stderr)
        return 1

    if args.report_out is not None:
        args.report_out.parent.mkdir(parents=True, exist_ok=True)
        args.report_out.write_text(report, encoding="utf-8")
        print(f"wrote MBES selected-loop remap report to {args.report_out}")
    else:
        print(report)
    print(f"wrote MBES selected-loop remapped allowlist to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
