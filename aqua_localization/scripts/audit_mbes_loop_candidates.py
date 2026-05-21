#!/usr/bin/env python3
"""Create a visual-audit checklist from MBES loop-status CSV output."""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Iterable


NO_CANDIDATE_ID = 2**32 - 1


@dataclass(frozen=True)
class LoopStatusRow:
    current_id: int
    candidate_id: int
    accepted: bool
    converged: bool
    fitness_score: float
    correction_translation_m: float
    correction_rotation_rad: float
    descriptor_centroid_distance_m: float
    descriptor_extent_ratio: float
    descriptor_point_count_ratio: float
    status: str


@dataclass(frozen=True)
class AuditRow:
    row: LoopStatusRow
    keyframe_gap: int
    risk_score: float
    priority: str
    flags: tuple[str, ...]


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


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    if value == 0.0:
        return "0"
    return f"{value:.{precision}f}".rstrip("0").rstrip(".")


def read_loop_status_csv(path: Path) -> list[LoopStatusRow]:
    with path.open(newline="", encoding="utf-8") as fp:
        rows = []
        for row in csv.DictReader(fp):
            rows.append(
                LoopStatusRow(
                    current_id=parse_int(row.get("current_id", "")),
                    candidate_id=parse_int(row.get("candidate_id", "")),
                    accepted=parse_bool(row.get("accepted", "")),
                    converged=parse_bool(row.get("converged", "")),
                    fitness_score=parse_float(row.get("fitness_score", "")),
                    correction_translation_m=parse_float(
                        row.get("correction_translation_m", "")
                    ),
                    correction_rotation_rad=parse_float(
                        row.get("correction_rotation_rad", "")
                    ),
                    descriptor_centroid_distance_m=parse_float(
                        row.get("descriptor_centroid_distance_m", "")
                    ),
                    descriptor_extent_ratio=parse_float(
                        row.get("descriptor_extent_ratio", "")
                    ),
                    descriptor_point_count_ratio=parse_float(
                        row.get("descriptor_point_count_ratio", "")
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


def finite_ratio(value: float, denominator: float) -> float:
    if denominator <= 0.0 or not math.isfinite(value):
        return 0.0
    return max(0.0, value / denominator)


def classify_accepted_loop(
    row: LoopStatusRow,
    *,
    max_fitness: float,
    max_translation_m: float,
    max_rotation_rad: float,
    min_keyframe_separation: int,
    descriptor_extent_warn: float,
    descriptor_point_ratio_warn: float,
) -> AuditRow:
    gap = row.current_id - row.candidate_id
    fitness_ratio = finite_ratio(row.fitness_score, max_fitness)
    translation_ratio = finite_ratio(row.correction_translation_m, max_translation_m)
    rotation_ratio = finite_ratio(row.correction_rotation_rad, max_rotation_rad)
    risk_score = fitness_ratio + translation_ratio + rotation_ratio

    flags = []
    if fitness_ratio >= 0.75:
        flags.append("fitness near gate")
    if translation_ratio >= 0.75:
        flags.append("translation near gate")
    if rotation_ratio >= 0.75:
        flags.append("rotation near gate")
    if gap <= 2 * min_keyframe_separation:
        flags.append("short keyframe gap")
    if (
        math.isfinite(row.descriptor_extent_ratio) and
        row.descriptor_extent_ratio >= descriptor_extent_warn
    ):
        flags.append("large extent ratio")
    if (
        math.isfinite(row.descriptor_point_count_ratio) and
        row.descriptor_point_count_ratio <= descriptor_point_ratio_warn
    ):
        flags.append("low point-count ratio")

    priority = "high" if risk_score >= 1.35 or len(flags) >= 2 else "medium"
    if priority == "medium" and risk_score < 0.65 and not flags:
        priority = "low"
    return AuditRow(row, gap, risk_score, priority, tuple(flags))


def accepted_audit_rows(rows: Iterable[LoopStatusRow], args) -> list[AuditRow]:
    accepted = [row for row in rows if row.accepted]
    audit_rows = [
        classify_accepted_loop(
            row,
            max_fitness=args.max_fitness,
            max_translation_m=args.max_translation_m,
            max_rotation_rad=args.max_rotation_rad,
            min_keyframe_separation=args.min_keyframe_separation,
            descriptor_extent_warn=args.descriptor_extent_warn,
            descriptor_point_ratio_warn=args.descriptor_point_ratio_warn,
        )
        for row in accepted
    ]
    return sorted(
        audit_rows,
        key=lambda item: (
            {"high": 0, "medium": 1, "low": 2}[item.priority],
            -item.risk_score,
            item.keyframe_gap,
        ),
    )


def status_counts(rows: Iterable[LoopStatusRow]) -> Counter[str]:
    return Counter(row.status for row in rows)


def format_summary(rows: list[LoopStatusRow]) -> list[str]:
    accepted = [row for row in rows if row.accepted]
    no_candidate = [row for row in rows if is_no_candidate(row)]
    rejected = [
        row for row in rows
        if not row.accepted and not is_no_candidate(row)
    ]
    return [
        f"- Samples: {len(rows)}",
        f"- Accepted loops: {len(accepted)}",
        f"- Rejected candidates: {len(rejected)}",
        f"- No-candidate statuses: {len(no_candidate)}",
        f"- Converged registrations: {sum(1 for row in rows if row.converged)}",
    ]


def format_audit_table(audit_rows: list[AuditRow], max_rows: int) -> list[str]:
    lines = [
        "| Rank | Priority | Candidate -> Current | Gap | Fitness | Correction m | Rotation rad | Descriptor c/e/r | Flags | Audit note |",
        "|-----:|----------|----------------------|----:|--------:|-------------:|-------------:|------------------|-------|------------|",
    ]
    for rank, item in enumerate(audit_rows[:max_rows], start=1):
        row = item.row
        descriptor = (
            f"{format_float(row.descriptor_centroid_distance_m)}/"
            f"{format_float(row.descriptor_extent_ratio)}/"
            f"{format_float(row.descriptor_point_count_ratio)}"
        )
        flags = ", ".join(item.flags) if item.flags else "none"
        lines.append(
            "| "
            f"{rank} | {item.priority} | {row.candidate_id} -> {row.current_id} | "
            f"{item.keyframe_gap} | {format_float(row.fitness_score)} | "
            f"{format_float(row.correction_translation_m)} | "
            f"{format_float(row.correction_rotation_rad)} | {descriptor} | "
            f"{flags} | TODO: inspect accepted marker geometry |"
        )
    return lines


def format_status_table(rows: list[LoopStatusRow], max_reasons: int) -> list[str]:
    lines = [
        "| Status | Count |",
        "|--------|------:|",
    ]
    for status, count in status_counts(rows).most_common(max_reasons):
        lines.append(f"| {status or 'n/a'} | {count} |")
    return lines


def format_report(rows: list[LoopStatusRow], args) -> str:
    audit_rows = accepted_audit_rows(rows, args)
    lines = [
        "# MBES Loop Candidate Visual Audit",
        "",
        f"- Source CSV: `{args.csv}`",
        f"- Gate assumptions: fitness <= {args.max_fitness:g}, "
        f"translation <= {args.max_translation_m:g} m, "
        f"rotation <= {args.max_rotation_rad:g} rad",
        f"- Keyframe gap warning: <= {2 * args.min_keyframe_separation}",
        "",
        "## Summary",
        "",
        *format_summary(rows),
        "",
        "## Accepted Loop Audit Priority",
        "",
        *format_audit_table(audit_rows, args.max_accepted),
        "",
        "## Status Counts",
        "",
        *format_status_table(rows, args.max_reasons),
        "",
        "## Audit Rule",
        "",
        "Mark an accepted loop as usable evidence only after its accepted RViz/rerun "
        "edge connects a plausible revisit, not an adjacent duplicate or an obvious "
        "registration jump. Keep the benchmark row labelled unaudited until every "
        "accepted loop above has a note.",
        "",
    ]
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path, help="Loop-status CSV")
    parser.add_argument("--out", type=Path, help="Optional Markdown output path")
    parser.add_argument("--max-accepted", type=int, default=50)
    parser.add_argument("--max-reasons", type=int, default=12)
    parser.add_argument("--max-fitness", type=float, default=2.0)
    parser.add_argument("--max-translation-m", type=float, default=5.0)
    parser.add_argument("--max-rotation-rad", type=float, default=0.5)
    parser.add_argument("--min-keyframe-separation", type=int, default=20)
    parser.add_argument("--descriptor-extent-warn", type=float, default=5.0)
    parser.add_argument("--descriptor-point-ratio-warn", type=float, default=0.5)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        rows = read_loop_status_csv(args.csv)
    except OSError as exc:
        print(f"failed to read MBES loop-status CSV: {exc}", file=sys.stderr)
        return 2

    text = format_report(rows, args)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"wrote audit report to {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
