#!/usr/bin/env python3
"""Create an MBES accepted-loop geometry review worksheet."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Iterable


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import audit_mbes_loop_candidates as audit
import publish_mbes_loop_audit_markers as marker_helpers


@dataclass(frozen=True)
class GeometryAuditRow:
    audit_row: audit.AuditRow
    candidate_xyz: tuple[float, float, float]
    current_xyz: tuple[float, float, float]
    plan_xy_distance_m: float
    depth_delta_m: float
    review_focus: str


@dataclass(frozen=True)
class MissingGeometryRow:
    audit_row: audit.AuditRow
    missing_side: str


def euclidean_xy(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def format_float(value: float, precision: int = 3) -> str:
    if not math.isfinite(value):
        return "n/a"
    if value == 0.0:
        return "0"
    return f"{value:.{precision}f}".rstrip("0").rstrip(".")


def review_focus(
    item: audit.AuditRow,
    *,
    plan_xy_distance_m: float,
    min_plan_distance_m: float,
) -> str:
    notes = []
    if item.priority == "high":
        notes.append("high-risk gate margin")
    if item.flags:
        notes.append(", ".join(item.flags))
    if plan_xy_distance_m <= min_plan_distance_m:
        notes.append("short plan-view edge")
    if not notes:
        return "geometry-only review"
    return "; ".join(notes)


def build_geometry_rows(
    audit_rows: Iterable[audit.AuditRow],
    keyframes: dict[int, marker_helpers.KeyframePose],
    *,
    max_rows: int,
    min_plan_distance_m: float,
) -> list[GeometryAuditRow]:
    rows: list[GeometryAuditRow] = []
    for item in audit_rows:
        if len(rows) >= max_rows:
            break
        candidate = keyframes.get(item.row.candidate_id)
        current = keyframes.get(item.row.current_id)
        if candidate is None or current is None:
            continue
        candidate_xyz = (candidate.x, candidate.y, candidate.z)
        current_xyz = (current.x, current.y, current.z)
        plan_xy = euclidean_xy(candidate_xyz, current_xyz)
        rows.append(
            GeometryAuditRow(
                audit_row=item,
                candidate_xyz=candidate_xyz,
                current_xyz=current_xyz,
                plan_xy_distance_m=plan_xy,
                depth_delta_m=abs(candidate.z - current.z),
                review_focus=review_focus(
                    item,
                    plan_xy_distance_m=plan_xy,
                    min_plan_distance_m=min_plan_distance_m,
                ),
            )
        )
    return rows


def missing_geometry_rows(
    audit_rows: Iterable[audit.AuditRow],
    keyframes: dict[int, marker_helpers.KeyframePose],
    *,
    max_rows: int,
) -> list[MissingGeometryRow]:
    missing: list[MissingGeometryRow] = []
    for item in list(audit_rows)[:max_rows]:
        candidate_missing = item.row.candidate_id not in keyframes
        current_missing = item.row.current_id not in keyframes
        if not candidate_missing and not current_missing:
            continue
        if candidate_missing and current_missing:
            missing_side = "candidate,current"
        elif candidate_missing:
            missing_side = "candidate"
        else:
            missing_side = "current"
        missing.append(MissingGeometryRow(item, missing_side))
    return missing


def keyframe_id_range(
    keyframes: dict[int, marker_helpers.KeyframePose],
) -> tuple[int, int] | None:
    if not keyframes:
        return None
    ids = keyframes.keys()
    return min(ids), max(ids)


def format_xyz(xyz: tuple[float, float, float]) -> str:
    return (
        f"{format_float(xyz[0])}, "
        f"{format_float(xyz[1])}, "
        f"{format_float(xyz[2])}"
    )


def format_summary(
    rows: list[GeometryAuditRow],
    *,
    total_accepted: int,
    keyframe_count: int,
    missing_count: int,
    keyframe_range: tuple[int, int] | None,
) -> list[str]:
    priorities = Counter(row.audit_row.priority for row in rows)
    range_text = (
        f"{keyframe_range[0]} -> {keyframe_range[1]}"
        if keyframe_range is not None else "n/a"
    )
    return [
        f"- Accepted loops in CSV: {total_accepted}",
        f"- Accepted loops with keyframe geometry: {len(rows)}",
        f"- Accepted loops missing keyframe geometry: {missing_count}",
        f"- Keyframes loaded: {keyframe_count}",
        f"- Loaded keyframe ID range: {range_text}",
        f"- High / medium / low review rows: "
        f"{priorities['high']} / {priorities['medium']} / {priorities['low']}",
    ]


def format_geometry_table(rows: list[GeometryAuditRow]) -> list[str]:
    lines = [
        "| Rank | Priority | Candidate -> Current | Gap | Plan XY m | Depth delta m | Correction m | Rotation rad | Candidate xyz | Current xyz | Review focus |",
        "|-----:|----------|----------------------|----:|----------:|--------------:|-------------:|-------------:|---------------|-------------|--------------|",
    ]
    for rank, item in enumerate(rows, start=1):
        audit_row = item.audit_row
        row = audit_row.row
        lines.append(
            "| "
            f"{rank} | {audit_row.priority} | "
            f"{row.candidate_id} -> {row.current_id} | "
            f"{audit_row.keyframe_gap} | "
            f"{format_float(item.plan_xy_distance_m)} | "
            f"{format_float(item.depth_delta_m)} | "
            f"{format_float(row.correction_translation_m)} | "
            f"{format_float(row.correction_rotation_rad)} | "
            f"{format_xyz(item.candidate_xyz)} | "
            f"{format_xyz(item.current_xyz)} | "
            f"{item.review_focus} |"
        )
    return lines


def format_missing_table(rows: list[MissingGeometryRow]) -> list[str]:
    if not rows:
        return ["- None"]
    lines = [
        "| Rank | Priority | Candidate -> Current | Missing side | Gap | Flags |",
        "|-----:|----------|----------------------|--------------|----:|-------|",
    ]
    for rank, item in enumerate(rows, start=1):
        audit_row = item.audit_row
        flags = ", ".join(audit_row.flags) if audit_row.flags else "none"
        lines.append(
            "| "
            f"{rank} | {audit_row.priority} | "
            f"{audit_row.row.candidate_id} -> {audit_row.row.current_id} | "
            f"{item.missing_side} | "
            f"{audit_row.keyframe_gap} | {flags} |"
        )
    return lines


def format_report(
    rows: list[GeometryAuditRow],
    *,
    args: argparse.Namespace,
    total_accepted: int,
    keyframe_count: int,
    keyframe_range: tuple[int, int] | None,
    missing_rows: list[MissingGeometryRow] | None = None,
) -> str:
    missing_rows = missing_rows or []
    lines = [
        "# MBES Accepted Loop Geometry Review",
        "",
        f"- Source bag: `{args.bag}`",
        f"- Source CSV: `{args.csv}`",
        f"- Gate assumptions: fitness <= {args.max_fitness:g}, "
        f"translation <= {args.max_translation_m:g} m, "
        f"rotation <= {args.max_rotation_rad:g} rad",
        "",
        "## Summary",
        "",
        *format_summary(
            rows,
            total_accepted=total_accepted,
            keyframe_count=keyframe_count,
            missing_count=len(missing_rows),
            keyframe_range=keyframe_range,
        ),
        "",
        "## Review Table",
        "",
        *format_geometry_table(rows),
        "",
        "## Missing Keyframe Geometry",
        "",
        *format_missing_table(missing_rows),
        "",
        "## Interpretation",
        "",
        "Use this worksheet with the RViz audit markers or plan-view PNG. A row is "
        "not a validated loop-closure claim until its marker edge has been checked "
        "against the replayed map geometry and marked as a plausible revisit.",
        "",
    ]
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path,
                        help="Results-included rosbag2 directory")
    parser.add_argument("--csv", required=True, type=Path,
                        help="Loop-status CSV")
    parser.add_argument("--out", type=Path,
                        help="Optional Markdown output path")
    parser.add_argument("--keyframe-topic", default="/aqua_pose_graph/keyframe")
    parser.add_argument("--max-accepted", type=int, default=100)
    parser.add_argument("--min-plan-distance-m", type=float, default=2.0)
    parser.add_argument(
        "--require-complete",
        action="store_true",
        help="Return a non-zero status if any accepted loop lacks keyframe geometry",
    )

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
        status_rows = audit.read_loop_status_csv(args.csv)
        keyframes = marker_helpers.read_keyframe_poses(args.bag, args.keyframe_topic)
    except (OSError, RuntimeError) as exc:
        print(f"failed to build MBES loop geometry review: {exc}", file=sys.stderr)
        return 2

    audit_rows = audit.accepted_audit_rows(status_rows, args)
    geometry_rows = build_geometry_rows(
        audit_rows,
        keyframes,
        max_rows=args.max_accepted,
        min_plan_distance_m=args.min_plan_distance_m,
    )
    missing_rows = missing_geometry_rows(
        audit_rows,
        keyframes,
        max_rows=args.max_accepted,
    )
    text = format_report(
        geometry_rows,
        args=args,
        total_accepted=sum(1 for row in status_rows if row.accepted),
        keyframe_count=len(keyframes),
        keyframe_range=keyframe_id_range(keyframes),
        missing_rows=missing_rows,
    )
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"wrote MBES loop geometry review to {args.out}")
    else:
        print(text)
    if args.require_complete and missing_rows:
        print(
            f"accepted-loop geometry is incomplete: {len(missing_rows)} row(s) missing",
            file=sys.stderr,
        )
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
