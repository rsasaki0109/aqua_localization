#!/usr/bin/env python3
"""Export MBES loop-closure status messages from a rosbag2 bag.

The CSV output is intended for threshold tuning. It preserves every
`aqua_msgs/LoopClosureStatus` sample and prints a compact markdown summary with
accepted/rejected/no-candidate counts, rejection reasons, and registration
fitness/correction/descriptor quantiles.

Example:

  ros2 run aqua_localization export_mbes_loop_status.py \\
    --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \\
    --out /tmp/mbes_loop_status.csv \\
    --summary-out /tmp/mbes_loop_status.md
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


NO_CANDIDATE_ID = 2**32 - 1

CSV_FIELDS = [
    "timestamp",
    "frame_id",
    "current_id",
    "candidate_id",
    "accepted",
    "converged",
    "fitness_score",
    "correction_translation_m",
    "correction_rotation_rad",
    "descriptor_centroid_distance_m",
    "descriptor_extent_ratio",
    "descriptor_point_count_ratio",
    "status",
]


@dataclass(frozen=True)
class LoopStatusSample:
    timestamp: float
    frame_id: str
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


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def optional_float(msg, attr: str) -> float:
    return float(getattr(msg, attr, math.nan))


def sample_from_msg(msg, fallback_time: float) -> LoopStatusSample:
    timestamp = stamp_to_seconds(msg.header.stamp)
    if timestamp <= 0.0:
        timestamp = fallback_time
    return LoopStatusSample(
        timestamp=timestamp,
        frame_id=str(msg.header.frame_id),
        current_id=int(msg.current_id),
        candidate_id=int(msg.candidate_id),
        accepted=bool(msg.accepted),
        converged=bool(msg.converged),
        fitness_score=float(msg.fitness_score),
        correction_translation_m=float(msg.correction_translation_m),
        correction_rotation_rad=float(msg.correction_rotation_rad),
        descriptor_centroid_distance_m=optional_float(
            msg, "descriptor_centroid_distance_m"
        ),
        descriptor_extent_ratio=optional_float(msg, "descriptor_extent_ratio"),
        descriptor_point_count_ratio=optional_float(
            msg, "descriptor_point_count_ratio"
        ),
        status=str(msg.status),
    )


def finite_values(samples: Iterable[LoopStatusSample], attr: str) -> list[float]:
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
            "p95": math.nan,
            "max": math.nan,
        }
    return {
        "count": len(values),
        "min": min(values),
        "median": percentile(values, 0.5),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def is_no_candidate(sample: LoopStatusSample) -> bool:
    return sample.candidate_id == NO_CANDIDATE_ID or "no candidate" in sample.status.lower()


def summarize(samples: list[LoopStatusSample]) -> dict:
    accepted = [sample for sample in samples if sample.accepted]
    no_candidate = [sample for sample in samples if is_no_candidate(sample)]
    rejected = [
        sample for sample in samples
        if not sample.accepted and not is_no_candidate(sample)
    ]
    return {
        "total": len(samples),
        "accepted": len(accepted),
        "rejected": len(rejected),
        "no_candidate": len(no_candidate),
        "converged": sum(1 for sample in samples if sample.converged),
        "status_counts": Counter(sample.status for sample in samples),
        "rejection_counts": Counter(sample.status for sample in rejected),
        "fitness": stats(finite_values(samples, "fitness_score")),
        "accepted_fitness": stats(finite_values(accepted, "fitness_score")),
        "correction_translation_m": stats(
            finite_values(samples, "correction_translation_m")
        ),
        "correction_rotation_rad": stats(
            finite_values(samples, "correction_rotation_rad")
        ),
        "descriptor_centroid_distance_m": stats(
            finite_values(samples, "descriptor_centroid_distance_m")
        ),
        "descriptor_extent_ratio": stats(
            finite_values(samples, "descriptor_extent_ratio")
        ),
        "descriptor_point_count_ratio": stats(
            finite_values(samples, "descriptor_point_count_ratio")
        ),
    }


def format_float(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.6g}"


def format_stats(label: str, summary: dict[str, float | int]) -> str:
    return (
        f"| {label} | {summary['count']} | {format_float(float(summary['min']))} | "
        f"{format_float(float(summary['median']))} | {format_float(float(summary['p95']))} | "
        f"{format_float(float(summary['max']))} |"
    )


def format_summary_markdown(summary: dict, topic: str) -> str:
    lines = [
        "# MBES Loop Closure Status Summary",
        "",
        f"- Topic: `{topic}`",
        f"- Samples: {summary['total']}",
        f"- Accepted: {summary['accepted']}",
        f"- Rejected: {summary['rejected']}",
        f"- No candidate: {summary['no_candidate']}",
        f"- Converged registrations: {summary['converged']}",
        "",
        "## Numeric Distributions",
        "",
        "| Metric | Count | Min | Median | P95 | Max |",
        "|--------|------:|----:|-------:|----:|----:|",
        format_stats("fitness_score", summary["fitness"]),
        format_stats("accepted fitness_score", summary["accepted_fitness"]),
        format_stats("correction_translation_m", summary["correction_translation_m"]),
        format_stats("correction_rotation_rad", summary["correction_rotation_rad"]),
        format_stats(
            "descriptor_centroid_distance_m",
            summary["descriptor_centroid_distance_m"],
        ),
        format_stats("descriptor_extent_ratio", summary["descriptor_extent_ratio"]),
        format_stats(
            "descriptor_point_count_ratio",
            summary["descriptor_point_count_ratio"],
        ),
        "",
        "## Rejection Reasons",
        "",
    ]
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
    lines.append("")
    return "\n".join(lines)


def write_csv(path: Path, samples: list[LoopStatusSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for sample in samples:
            writer.writerow({
                "timestamp": f"{sample.timestamp:.9f}",
                "frame_id": sample.frame_id,
                "current_id": sample.current_id,
                "candidate_id": sample.candidate_id,
                "accepted": int(sample.accepted),
                "converged": int(sample.converged),
                "fitness_score": f"{sample.fitness_score:.9f}",
                "correction_translation_m": f"{sample.correction_translation_m:.9f}",
                "correction_rotation_rad": f"{sample.correction_rotation_rad:.9f}",
                "descriptor_centroid_distance_m": (
                    f"{sample.descriptor_centroid_distance_m:.9f}"
                ),
                "descriptor_extent_ratio": f"{sample.descriptor_extent_ratio:.9f}",
                "descriptor_point_count_ratio": (
                    f"{sample.descriptor_point_count_ratio:.9f}"
                ),
                "status": sample.status,
            })


def read_bag_samples(bag: Path, topic: str) -> list[LoopStatusSample]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        raise RuntimeError("missing dependency: install rosbags to read rosbag2 files") from e

    bag_dir = bag if bag.is_dir() else bag.parent
    if not bag_dir.is_dir():
        raise RuntimeError(f"not a rosbag2 directory: {bag_dir}")

    samples: list[LoopStatusSample] = []
    with AnyReader([bag_dir]) as reader:
        wanted = [connection for connection in reader.connections if connection.topic == topic]
        if not wanted:
            available = ", ".join(sorted({connection.topic for connection in reader.connections}))
            raise RuntimeError(f"topic {topic!r} not found. Available topics: {available}")
        for connection, t_ns, raw in reader.messages(connections=wanted):
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception:
                continue
            samples.append(sample_from_msg(msg, t_ns * 1.0e-9))
    return samples


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path,
                        help="rosbag2 directory or .mcap file")
    parser.add_argument("--out", required=True, type=Path,
                        help="Output CSV path")
    parser.add_argument("--topic", default="/mbes_loop_closure/status",
                        help="LoopClosureStatus topic")
    parser.add_argument("--summary-out", type=Path,
                        help="Optional markdown summary output path")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        samples = read_bag_samples(args.bag, args.topic)
    except RuntimeError as e:
        sys.stderr.write(f"{e}\n")
        return 1

    write_csv(args.out, samples)
    summary_text = format_summary_markdown(summarize(samples), args.topic)
    if args.summary_out:
        args.summary_out.parent.mkdir(parents=True, exist_ok=True)
        args.summary_out.write_text(summary_text, encoding="utf-8")
    print(summary_text)
    print(f"wrote {len(samples)} samples to {args.out}", file=sys.stderr)
    if args.summary_out:
        print(f"wrote summary to {args.summary_out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
