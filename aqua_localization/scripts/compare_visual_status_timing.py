#!/usr/bin/env python3
"""Compare visual frontend status CSV timing between two replay paths.

The tool matches target rows to the nearest baseline timestamp and emits a
correspondence table with frame-index deltas, timestamp deltas, stereo sync
deltas, and selected frontend diagnostics. It is meant to explain why ROS
replay and direct rosbag2-sqlite replay produce different visual trajectories.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys


@dataclass(frozen=True)
class StatusSample:
    timestamp: float
    right_timestamp: float
    stereo_sync_delta_ms: float
    frame_index: int
    accepted: bool
    status: str
    left_features: int
    right_features: int
    stereo_matches: int
    stereo_points: int
    temporal_matches: int
    pnp_inliers: int
    step_translation_m: float


@dataclass(frozen=True)
class TimingComparisonSummary:
    baseline_samples: int
    target_samples: int
    matched_samples: int
    unmatched_target_samples: int
    unmatched_baseline_samples: int
    median_abs_timestamp_delta_ms: float
    max_abs_timestamp_delta_ms: float
    over_slop_samples: int
    median_frame_index_delta: float
    max_abs_frame_index_delta: int
    acceptance_mismatches: int
    status_mismatches: int
    median_abs_sync_delta_diff_ms: float
    max_abs_sync_delta_diff_ms: float
    first_over_slop_target_frame: int | None
    first_over_slop_offset_s: float | None
    first_acceptance_mismatch_target_frame: int | None
    first_acceptance_mismatch_offset_s: float | None


def parse_float(row: dict[str, str], field: str) -> float:
    value = row.get(field, "")
    if value == "":
        return math.nan
    return float(value)


def parse_int(row: dict[str, str], field: str) -> int:
    value = row.get(field, "")
    if value == "":
        return 0
    return int(float(value))


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes")


def read_status_csv(path: Path) -> list[StatusSample]:
    with path.open(newline="", encoding="utf-8") as fp:
        rows = []
        for row in csv.DictReader(fp):
            rows.append(StatusSample(
                timestamp=parse_float(row, "timestamp"),
                right_timestamp=parse_float(row, "right_timestamp"),
                stereo_sync_delta_ms=parse_float(row, "stereo_sync_delta_ms"),
                frame_index=parse_int(row, "frame_index"),
                accepted=parse_bool(row.get("accepted", "")),
                status=str(row.get("status", "")),
                left_features=parse_int(row, "left_features"),
                right_features=parse_int(row, "right_features"),
                stereo_matches=parse_int(row, "stereo_matches"),
                stereo_points=parse_int(row, "stereo_points"),
                temporal_matches=parse_int(row, "temporal_matches"),
                pnp_inliers=parse_int(row, "pnp_inliers"),
                step_translation_m=parse_float(row, "step_translation_m"),
            ))
    return sorted(rows, key=lambda sample: sample.timestamp)


def nearest_sample(samples: list[StatusSample], timestamp: float, start_index: int) -> tuple[int, StatusSample]:
    if not samples:
        raise ValueError("cannot match against an empty baseline")
    index = min(max(start_index, 0), len(samples) - 1)
    while (
        index + 1 < len(samples)
        and abs(samples[index + 1].timestamp - timestamp)
        <= abs(samples[index].timestamp - timestamp)
    ):
        index += 1
    while (
        index > 0
        and abs(samples[index - 1].timestamp - timestamp)
        < abs(samples[index].timestamp - timestamp)
    ):
        index -= 1
    return index, samples[index]


def compare_status_timing(
    baseline: list[StatusSample],
    target: list[StatusSample],
    *,
    timestamp_slop_s: float,
) -> tuple[TimingComparisonSummary, list[dict]]:
    if timestamp_slop_s < 0.0:
        raise ValueError("timestamp_slop_s must be non-negative")
    if not baseline:
        raise ValueError("baseline status CSV is empty")
    if not target:
        raise ValueError("target status CSV is empty")

    rows = []
    baseline_index = 0
    matched_baseline_indices = set()
    first_target_time = target[0].timestamp
    for target_sample in target:
        baseline_index, baseline_sample = nearest_sample(
            baseline, target_sample.timestamp, baseline_index
        )
        matched_baseline_indices.add(baseline_index)
        timestamp_delta_ms = (target_sample.timestamp - baseline_sample.timestamp) * 1000.0
        sync_delta_diff_ms = math.nan
        if (
            math.isfinite(target_sample.stereo_sync_delta_ms)
            and math.isfinite(baseline_sample.stereo_sync_delta_ms)
        ):
            sync_delta_diff_ms = (
                target_sample.stereo_sync_delta_ms - baseline_sample.stereo_sync_delta_ms
            )
        rows.append({
            "target_frame_index": target_sample.frame_index,
            "baseline_frame_index": baseline_sample.frame_index,
            "frame_index_delta": target_sample.frame_index - baseline_sample.frame_index,
            "target_timestamp": target_sample.timestamp,
            "baseline_timestamp": baseline_sample.timestamp,
            "target_offset_s": target_sample.timestamp - first_target_time,
            "timestamp_delta_ms": timestamp_delta_ms,
            "target_right_timestamp": target_sample.right_timestamp,
            "baseline_right_timestamp": baseline_sample.right_timestamp,
            "target_sync_delta_ms": target_sample.stereo_sync_delta_ms,
            "baseline_sync_delta_ms": baseline_sample.stereo_sync_delta_ms,
            "sync_delta_diff_ms": sync_delta_diff_ms,
            "target_accepted": target_sample.accepted,
            "baseline_accepted": baseline_sample.accepted,
            "accepted_match": target_sample.accepted == baseline_sample.accepted,
            "target_status": target_sample.status,
            "baseline_status": baseline_sample.status,
            "status_match": target_sample.status == baseline_sample.status,
            "target_left_features": target_sample.left_features,
            "baseline_left_features": baseline_sample.left_features,
            "left_features_delta": target_sample.left_features - baseline_sample.left_features,
            "target_stereo_points": target_sample.stereo_points,
            "baseline_stereo_points": baseline_sample.stereo_points,
            "stereo_points_delta": target_sample.stereo_points - baseline_sample.stereo_points,
            "target_temporal_matches": target_sample.temporal_matches,
            "baseline_temporal_matches": baseline_sample.temporal_matches,
            "temporal_matches_delta": (
                target_sample.temporal_matches - baseline_sample.temporal_matches
            ),
            "target_pnp_inliers": target_sample.pnp_inliers,
            "baseline_pnp_inliers": baseline_sample.pnp_inliers,
            "pnp_inliers_delta": target_sample.pnp_inliers - baseline_sample.pnp_inliers,
            "target_step_translation_m": target_sample.step_translation_m,
            "baseline_step_translation_m": baseline_sample.step_translation_m,
            "step_translation_delta_m": (
                target_sample.step_translation_m - baseline_sample.step_translation_m
                if (
                    math.isfinite(target_sample.step_translation_m)
                    and math.isfinite(baseline_sample.step_translation_m)
                )
                else math.nan
            ),
        })

    abs_timestamp_deltas = [abs(float(row["timestamp_delta_ms"])) for row in rows]
    frame_index_deltas = [float(row["frame_index_delta"]) for row in rows]
    abs_sync_diffs = [
        abs(float(row["sync_delta_diff_ms"]))
        for row in rows
        if math.isfinite(float(row["sync_delta_diff_ms"]))
    ]
    over_slop = [
        row for row in rows if abs(float(row["timestamp_delta_ms"])) > timestamp_slop_s * 1000.0
    ]
    acceptance_mismatches = [row for row in rows if not bool(row["accepted_match"])]
    summary = TimingComparisonSummary(
        baseline_samples=len(baseline),
        target_samples=len(target),
        matched_samples=len(rows),
        unmatched_target_samples=0,
        unmatched_baseline_samples=len(baseline) - len(matched_baseline_indices),
        median_abs_timestamp_delta_ms=median(abs_timestamp_deltas),
        max_abs_timestamp_delta_ms=max(abs_timestamp_deltas),
        over_slop_samples=len(over_slop),
        median_frame_index_delta=median(frame_index_deltas),
        max_abs_frame_index_delta=int(max(abs(value) for value in frame_index_deltas)),
        acceptance_mismatches=len(acceptance_mismatches),
        status_mismatches=sum(1 for row in rows if not bool(row["status_match"])),
        median_abs_sync_delta_diff_ms=median(abs_sync_diffs),
        max_abs_sync_delta_diff_ms=max(abs_sync_diffs) if abs_sync_diffs else math.nan,
        first_over_slop_target_frame=(
            int(over_slop[0]["target_frame_index"]) if over_slop else None
        ),
        first_over_slop_offset_s=(
            float(over_slop[0]["target_offset_s"]) if over_slop else None
        ),
        first_acceptance_mismatch_target_frame=(
            int(acceptance_mismatches[0]["target_frame_index"])
            if acceptance_mismatches else None
        ),
        first_acceptance_mismatch_offset_s=(
            float(acceptance_mismatches[0]["target_offset_s"])
            if acceptance_mismatches else None
        ),
    )
    return summary, rows


def median(values: list[float]) -> float:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    mid = len(finite) // 2
    if len(finite) % 2:
        return finite[mid]
    return 0.5 * (finite[mid - 1] + finite[mid])


def format_float(value: float | None, precision: int = 4) -> str:
    if value is None:
        return "n/a"
    if not math.isfinite(float(value)):
        return "n/a"
    return f"{float(value):.{precision}f}"


def format_optional_int(value: int | None) -> str:
    return "n/a" if value is None else str(value)


def write_correspondence_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "target_frame_index",
        "baseline_frame_index",
        "frame_index_delta",
        "target_timestamp",
        "baseline_timestamp",
        "target_offset_s",
        "timestamp_delta_ms",
        "target_right_timestamp",
        "baseline_right_timestamp",
        "target_sync_delta_ms",
        "baseline_sync_delta_ms",
        "sync_delta_diff_ms",
        "target_accepted",
        "baseline_accepted",
        "accepted_match",
        "target_status",
        "baseline_status",
        "status_match",
        "target_left_features",
        "baseline_left_features",
        "left_features_delta",
        "target_stereo_points",
        "baseline_stereo_points",
        "stereo_points_delta",
        "target_temporal_matches",
        "baseline_temporal_matches",
        "temporal_matches_delta",
        "target_pnp_inliers",
        "baseline_pnp_inliers",
        "pnp_inliers_delta",
        "target_step_translation_m",
        "baseline_step_translation_m",
        "step_translation_delta_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def format_markdown(
    baseline_path: Path,
    target_path: Path,
    summary: TimingComparisonSummary,
    *,
    timestamp_slop_s: float,
    csv_path: Path | None,
) -> str:
    lines = [
        "# Visual Status Timing Comparison",
        "",
        f"- Baseline status CSV: `{baseline_path}`",
        f"- Target status CSV: `{target_path}`",
        f"- Timestamp slop: {timestamp_slop_s * 1000.0:.3f} ms",
        f"- Baseline / target samples: {summary.baseline_samples} / {summary.target_samples}",
        f"- Matched target samples: {summary.matched_samples}",
        f"- Unmatched baseline samples: {summary.unmatched_baseline_samples}",
        "",
        "## Timing",
        "",
        f"- Median abs timestamp delta: {summary.median_abs_timestamp_delta_ms:.6f} ms",
        f"- Max abs timestamp delta: {summary.max_abs_timestamp_delta_ms:.6f} ms",
        f"- Over-slop samples: {summary.over_slop_samples}",
        f"- First over-slop target frame: {format_optional_int(summary.first_over_slop_target_frame)}",
        f"- First over-slop offset: {format_float(summary.first_over_slop_offset_s)} s",
        f"- Median frame-index delta: {format_float(summary.median_frame_index_delta, 2)}",
        f"- Max abs frame-index delta: {summary.max_abs_frame_index_delta}",
        "",
        "## Stereo Sync",
        "",
        f"- Median abs sync-delta difference: {format_float(summary.median_abs_sync_delta_diff_ms, 6)} ms",
        f"- Max abs sync-delta difference: {format_float(summary.max_abs_sync_delta_diff_ms, 6)} ms",
        "",
        "## Frontend State",
        "",
        f"- Acceptance mismatches: {summary.acceptance_mismatches}",
        f"- Status-string mismatches: {summary.status_mismatches}",
        f"- First acceptance mismatch target frame: {format_optional_int(summary.first_acceptance_mismatch_target_frame)}",
        f"- First acceptance mismatch offset: {format_float(summary.first_acceptance_mismatch_offset_s)} s",
    ]
    if csv_path is not None:
        lines.append(f"- Correspondence CSV: `{csv_path}`")
    lines.append("")
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Compare visual frontend status timing between replay paths."
    )
    parser.add_argument("--baseline-status", required=True, type=Path)
    parser.add_argument("--target-status", required=True, type=Path)
    parser.add_argument("--out", type=Path, help="Optional Markdown report path.")
    parser.add_argument("--csv", type=Path, help="Optional correspondence CSV path.")
    parser.add_argument("--timestamp-slop-ms", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.timestamp_slop_ms < 0.0:
        raise ValueError("--timestamp-slop-ms must be non-negative")
    baseline = read_status_csv(args.baseline_status)
    target = read_status_csv(args.target_status)
    summary, rows = compare_status_timing(
        baseline,
        target,
        timestamp_slop_s=args.timestamp_slop_ms / 1000.0,
    )
    if args.csv is not None:
        write_correspondence_csv(args.csv, rows)
    report = format_markdown(
        args.baseline_status,
        args.target_status,
        summary,
        timestamp_slop_s=args.timestamp_slop_ms / 1000.0,
        csv_path=args.csv,
    )
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
    print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
