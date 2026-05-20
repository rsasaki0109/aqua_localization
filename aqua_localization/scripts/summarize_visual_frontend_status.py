#!/usr/bin/env python3
"""Summarize experimental stereo visual frontend status CSV files.

The input CSV is produced by ``stereo_visual_odometry.py`` with
``diagnostics.status_csv_path``. The summary is meant to answer the practical
question: should the next tuning pass focus on features, stereo triangulation,
temporal matching, PnP gating, or motion outliers?
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Iterable


NUMERIC_FIELDS = [
    "left_features",
    "right_features",
    "stereo_matches",
    "stereo_points",
    "temporal_matches",
    "pnp_inliers",
    "inlier_ratio",
    "step_translation_m",
]


@dataclass(frozen=True)
class VisualStatusSample:
    timestamp: float
    frame_index: int
    accepted_count: int
    rejected_count: int
    left_features: int
    right_features: int
    stereo_matches: int
    stereo_points: int
    temporal_matches: int
    pnp_inliers: int
    inlier_ratio: float
    step_translation_m: float
    accepted: bool
    status: str


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes")


def parse_int(row: dict[str, str], field: str) -> int:
    value = row.get(field, "")
    if value == "":
        return 0
    return int(float(value))


def parse_float(row: dict[str, str], field: str) -> float:
    value = row.get(field, "")
    if value == "":
        return math.nan
    return float(value)


def sample_from_row(row: dict[str, str]) -> VisualStatusSample:
    return VisualStatusSample(
        timestamp=parse_float(row, "timestamp"),
        frame_index=parse_int(row, "frame_index"),
        accepted_count=parse_int(row, "accepted_count"),
        rejected_count=parse_int(row, "rejected_count"),
        left_features=parse_int(row, "left_features"),
        right_features=parse_int(row, "right_features"),
        stereo_matches=parse_int(row, "stereo_matches"),
        stereo_points=parse_int(row, "stereo_points"),
        temporal_matches=parse_int(row, "temporal_matches"),
        pnp_inliers=parse_int(row, "pnp_inliers"),
        inlier_ratio=parse_float(row, "inlier_ratio"),
        step_translation_m=parse_float(row, "step_translation_m"),
        accepted=parse_bool(row.get("accepted", "")),
        status=str(row.get("status", "")),
    )


def read_csv(path: Path) -> list[VisualStatusSample]:
    with path.open(newline="", encoding="utf-8") as fp:
        return [sample_from_row(row) for row in csv.DictReader(fp)]


def finite_values(samples: Iterable[VisualStatusSample], attr: str) -> list[float]:
    values = []
    for sample in samples:
        value = float(getattr(sample, attr))
        if math.isfinite(value):
            values.append(value)
    return values


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


def stats(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {
            "count": 0,
            "min": math.nan,
            "median": math.nan,
            "mean": math.nan,
            "p95": math.nan,
            "max": math.nan,
        }
    return {
        "count": len(values),
        "min": min(values),
        "median": percentile(values, 0.5),
        "mean": sum(values) / len(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def max_counter_value(counter: Counter) -> int:
    return max(counter.values()) if counter else 0


def summarize(samples: list[VisualStatusSample]) -> dict:
    accepted = [sample for sample in samples if sample.accepted]
    rejected = [sample for sample in samples if not sample.accepted]
    moving = [sample for sample in samples if sample.temporal_matches > 0]
    status_counts = Counter(sample.status for sample in samples)
    rejection_counts = Counter(sample.status for sample in rejected)
    return {
        "total": len(samples),
        "accepted": len(accepted),
        "rejected": len(rejected),
        "acceptance_ratio": len(accepted) / len(samples) if samples else math.nan,
        "duration_s": matched_duration(samples),
        "status_counts": status_counts,
        "rejection_counts": rejection_counts,
        "dominant_rejection": (
            rejection_counts.most_common(1)[0][0] if rejection_counts else "none"
        ),
        "numeric": {
            field: stats(finite_values(samples, field)) for field in NUMERIC_FIELDS
        },
        "moving_numeric": {
            field: stats(finite_values(moving, field)) for field in NUMERIC_FIELDS
        },
    }


def matched_duration(samples: list[VisualStatusSample]) -> float:
    finite_t = [sample.timestamp for sample in samples if math.isfinite(sample.timestamp)]
    if len(finite_t) < 2:
        return 0.0
    return max(finite_t) - min(finite_t)


def format_float(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.6g}"


def format_percent(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{100.0 * value:.1f}%"


def format_stats(label: str, summary: dict[str, float | int]) -> str:
    return (
        f"| {label} | {summary['count']} | {format_float(float(summary['min']))} | "
        f"{format_float(float(summary['median']))} | {format_float(float(summary['mean']))} | "
        f"{format_float(float(summary['p95']))} | {format_float(float(summary['max']))} |"
    )


def tuning_hints(summary: dict) -> list[str]:
    hints = []
    numeric = summary["moving_numeric"]
    total = int(summary["total"])
    rejected = int(summary["rejected"])
    rejection_counts: Counter = summary["rejection_counts"]

    if total == 0:
        return ["No samples were available; check the status CSV path and replay duration."]

    if rejected == 0:
        hints.append("No rejected frames were observed; focus next on scale, extrinsics, and drift.")
    elif rejected / total > 0.2:
        hints.append("More than 20% of frames were rejected; inspect the dominant rejection reason first.")

    stereo_points_median = float(numeric["stereo_points"]["median"])
    temporal_matches_median = float(numeric["temporal_matches"]["median"])
    inlier_ratio_median = float(numeric["inlier_ratio"]["median"])
    step_p95 = float(numeric["step_translation_m"]["p95"])

    if math.isfinite(stereo_points_median) and stereo_points_median < 80:
        hints.append("Low stereo point count: tune ORB/image preprocessing, masks, or stereo disparity gates.")
    if math.isfinite(temporal_matches_median) and temporal_matches_median < 40:
        hints.append("Low temporal match count: improve descriptor matching or add a stronger tracking stage.")
    if math.isfinite(inlier_ratio_median) and inlier_ratio_median < 0.5:
        hints.append("Low PnP inlier ratio: tighten outlier handling or revisit feature geometry.")
    if math.isfinite(step_p95) and step_p95 > 0.25:
        hints.append("Large step-translation tail: check scale, timestamps, motion gate, and camera extrinsics.")

    dominant = summary["dominant_rejection"]
    if dominant != "none" and max_counter_value(rejection_counts) > 0:
        hints.append(f"Dominant rejection reason is `{dominant}`; tune that gate before adding new algorithms.")

    if not hints:
        hints.append("Diagnostics look healthy; next gains likely require scale/extrinsic calibration or loop-level constraints.")
    return hints


def format_summary_markdown(summary: dict, source: str) -> str:
    lines = [
        "# Visual Frontend Status Summary",
        "",
        f"- Source: `{source}`",
        f"- Samples: {summary['total']}",
        f"- Duration: {summary['duration_s']:.2f} s",
        f"- Accepted: {summary['accepted']} ({format_percent(summary['acceptance_ratio'])})",
        f"- Rejected: {summary['rejected']}",
        "",
        "## Numeric Distributions",
        "",
        "| Metric | Count | Min | Median | Mean | P95 | Max |",
        "|--------|------:|----:|-------:|-----:|----:|----:|",
    ]
    for field in NUMERIC_FIELDS:
        lines.append(format_stats(field, summary["numeric"][field]))

    lines.extend(["", "## Moving-Frame Distributions", ""])
    lines.extend([
        "| Metric | Count | Min | Median | Mean | P95 | Max |",
        "|--------|------:|----:|-------:|-----:|----:|----:|",
    ])
    for field in NUMERIC_FIELDS:
        lines.append(format_stats(field, summary["moving_numeric"][field]))

    lines.extend(["", "## Rejection Reasons", ""])
    if summary["rejection_counts"]:
        for reason, count in summary["rejection_counts"].most_common():
            lines.append(f"- {reason}: {count}")
    else:
        lines.append("- none")

    lines.extend(["", "## All Status Values", ""])
    if summary["status_counts"]:
        for status, count in summary["status_counts"].most_common():
            lines.append(f"- {status}: {count}")
    else:
        lines.append("- none")

    lines.extend(["", "## Tuning Hints", ""])
    for hint in tuning_hints(summary):
        lines.append(f"- {hint}")
    lines.append("")
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize stereo visual frontend diagnostics CSV."
    )
    parser.add_argument("csv", type=Path, help="CSV from diagnostics.status_csv_path.")
    parser.add_argument("--summary-out", type=Path, help="Optional markdown output path.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    samples = read_csv(args.csv)
    text = format_summary_markdown(summarize(samples), str(args.csv))
    if args.summary_out:
        args.summary_out.parent.mkdir(parents=True, exist_ok=True)
        args.summary_out.write_text(text, encoding="utf-8")
    print(text)
    if args.summary_out:
        print(f"wrote summary to {args.summary_out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
