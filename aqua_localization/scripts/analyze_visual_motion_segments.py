#!/usr/bin/env python3
"""Analyze visual-odometry relative motion segments from TUM trajectories.

This complements ``analyze_visual_drift.py``. Instead of fitting each window, it
compares short relative motions directly:

  visual segment length / reference segment length

The result helps separate constant scale error from direction-, speed-, or
time-dependent motion bias.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np


def load_compare_module():
    script_path = Path(__file__).resolve().parent / "compare_trajectories.py"
    spec = importlib.util.spec_from_file_location("compare_trajectories", script_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class MotionSegment:
    start_s: float
    end_s: float
    samples: int
    reference_length_m: float
    visual_length_m: float
    visual_to_reference_ratio: float
    correction_scale: float
    reference_speed_mps: float
    heading_deg: float


def load_matched_positions(reference_path: Path, estimate_path: Path):
    compare = load_compare_module()
    ref = compare.load_tum(reference_path)
    est = compare.load_tum(estimate_path)
    ref_at_est = compare.interpolate_positions(ref, est[:, 0])
    valid = ~np.isnan(ref_at_est).any(axis=1)
    if not np.any(valid):
        raise ValueError("no overlapping timestamps between reference and estimate")
    return est[valid, 0], est[valid, 1:4], ref_at_est[valid]


def interpolate_xyz(times: np.ndarray, xyz: np.ndarray, query_time: float) -> np.ndarray:
    return np.asarray([
        np.interp(query_time, times, xyz[:, axis]) for axis in range(3)
    ], dtype=np.float64)


def heading_deg(delta_xyz: np.ndarray) -> float:
    if float(np.linalg.norm(delta_xyz[:2])) <= 1.0e-12:
        return math.nan
    angle = math.degrees(math.atan2(float(delta_xyz[1]), float(delta_xyz[0])))
    return angle + 360.0 if angle < 0.0 else angle


def build_segments(
    times: np.ndarray,
    est_xyz: np.ndarray,
    ref_xyz: np.ndarray,
    segment_s: float,
    stride_s: float,
    min_reference_motion_m: float,
) -> list[MotionSegment]:
    if segment_s <= 0.0:
        raise ValueError("segment_s must be positive")
    if stride_s <= 0.0:
        raise ValueError("stride_s must be positive")
    if min_reference_motion_m < 0.0:
        raise ValueError("min_reference_motion_m must be non-negative")

    t0 = float(times.min())
    t1 = float(times.max())
    segments = []
    current = t0
    while current + segment_s <= t1 + 1.0e-9:
        end = current + segment_s
        start_ref = interpolate_xyz(times, ref_xyz, current)
        end_ref = interpolate_xyz(times, ref_xyz, end)
        start_est = interpolate_xyz(times, est_xyz, current)
        end_est = interpolate_xyz(times, est_xyz, end)
        ref_delta = end_ref - start_ref
        est_delta = end_est - start_est
        ref_length = float(np.linalg.norm(ref_delta))
        est_length = float(np.linalg.norm(est_delta))
        if ref_length >= min_reference_motion_m:
            ratio = est_length / ref_length if ref_length > 0.0 else math.nan
            correction = ref_length / est_length if est_length > 0.0 else math.nan
            samples = int(np.count_nonzero((times >= current) & (times <= end)))
            segments.append(MotionSegment(
                start_s=current - t0,
                end_s=end - t0,
                samples=samples,
                reference_length_m=ref_length,
                visual_length_m=est_length,
                visual_to_reference_ratio=ratio,
                correction_scale=correction,
                reference_speed_mps=ref_length / segment_s,
                heading_deg=heading_deg(ref_delta),
            ))
        current += stride_s
    return segments


def finite_values(segments: list[MotionSegment], attr: str) -> list[float]:
    values = []
    for segment in segments:
        value = float(getattr(segment, attr))
        if math.isfinite(value):
            values.append(value)
    return values


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if q <= 0.0:
        return ordered[0]
    if q >= 1.0:
        return ordered[-1]
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    alpha = pos - lo
    return ordered[lo] * (1.0 - alpha) + ordered[hi] * alpha


def stats(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "min": math.nan, "median": math.nan, "mean": math.nan, "p95": math.nan, "max": math.nan, "std": math.nan}
    mean = sum(values) / len(values)
    return {
        "count": len(values),
        "min": min(values),
        "median": percentile(values, 0.5),
        "mean": mean,
        "p95": percentile(values, 0.95),
        "max": max(values),
        "std": float(np.std(np.asarray(values, dtype=np.float64))),
    }


def heading_bucket(deg: float) -> str:
    if not math.isfinite(deg):
        return "unknown"
    labels = ["E", "NE", "N", "NW", "W", "SW", "S", "SE"]
    index = int(((deg + 22.5) % 360.0) // 45.0)
    return labels[index]


def bucket_stats(segments: list[MotionSegment], attr: str, bucket_fn) -> list[tuple[str, dict]]:
    buckets: dict[str, list[float]] = {}
    for segment in segments:
        key = bucket_fn(segment)
        value = float(getattr(segment, attr))
        if math.isfinite(value):
            buckets.setdefault(key, []).append(value)
    return [(key, stats(values)) for key, values in sorted(buckets.items())]


def format_float(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.6g}"


def format_stats(label: str, summary: dict) -> str:
    return (
        f"| {label} | {summary['count']} | {format_float(float(summary['min']))} | "
        f"{format_float(float(summary['median']))} | {format_float(float(summary['mean']))} | "
        f"{format_float(float(summary['p95']))} | {format_float(float(summary['max']))} | "
        f"{format_float(float(summary['std']))} |"
    )


def interpretation(ratio_stats: dict, correction_stats: dict) -> list[str]:
    hints = []
    rel_std = (
        float(ratio_stats["std"]) / float(ratio_stats["mean"])
        if math.isfinite(float(ratio_stats["mean"])) and float(ratio_stats["mean"]) != 0.0
        else math.nan
    )
    median_correction = float(correction_stats["median"])
    if math.isfinite(rel_std) and rel_std > 0.15:
        hints.append("Segment scale ratio varies strongly; inspect direction/speed buckets before choosing one fixed scale.")
    elif math.isfinite(rel_std):
        hints.append("Segment scale ratio is comparatively stable; a fixed scale may be acceptable after held-out validation.")
    if math.isfinite(median_correction):
        hints.append(f"Median correction scale from segment lengths is {median_correction:.9f}.")
    return hints


def format_markdown(reference: Path, estimate: Path, segments: list[MotionSegment]) -> str:
    ratio_stats = stats(finite_values(segments, "visual_to_reference_ratio"))
    correction_stats = stats(finite_values(segments, "correction_scale"))
    speed_stats = stats(finite_values(segments, "reference_speed_mps"))
    heading_rows = bucket_stats(
        segments, "correction_scale", lambda segment: heading_bucket(segment.heading_deg))

    lines = [
        "# Visual Motion Segment Analysis",
        "",
        f"- Reference: `{reference}`",
        f"- Estimate: `{estimate}`",
        f"- Segments: {len(segments)}",
        "",
        "## Summary",
        "",
        "| Metric | Count | Min | Median | Mean | P95 | Max | Std |",
        "|--------|------:|----:|-------:|-----:|----:|----:|----:|",
        format_stats("visual/reference length ratio", ratio_stats),
        format_stats("reference/visual correction scale", correction_stats),
        format_stats("reference speed m/s", speed_stats),
        "",
        "## Direction Buckets",
        "",
        "| Heading | Count | Min scale | Median scale | Mean scale | P95 scale | Max scale | Std scale |",
        "|---------|------:|----------:|-------------:|-----------:|----------:|----------:|----------:|",
    ]
    for key, summary in heading_rows:
        lines.append(format_stats(key, summary))

    lines.extend([
        "",
        "## Segments",
        "",
        "| Start s | End s | Samples | Ref m | Visual m | Visual/ref | Correction scale | Ref speed m/s | Heading deg |",
        "|--------:|------:|--------:|------:|---------:|-----------:|-----------------:|--------------:|------------:|",
    ])
    for segment in segments:
        lines.append(
            f"| {segment.start_s:.2f} | {segment.end_s:.2f} | {segment.samples} | "
            f"{segment.reference_length_m:.4f} | {segment.visual_length_m:.4f} | "
            f"{segment.visual_to_reference_ratio:.6f} | {segment.correction_scale:.9f} | "
            f"{segment.reference_speed_mps:.4f} | {format_float(segment.heading_deg)} |"
        )

    lines.extend(["", "## Interpretation", ""])
    for hint in interpretation(ratio_stats, correction_stats):
        lines.append(f"- {hint}")
    lines.append("")
    return "\n".join(lines)


def run_analysis(args) -> str:
    times, est_xyz, ref_xyz = load_matched_positions(args.reference, args.estimate)
    segments = build_segments(
        times,
        est_xyz,
        ref_xyz,
        args.segment_s,
        args.stride_s,
        args.min_reference_motion_m,
    )
    if not segments:
        raise ValueError("no motion segments available; lower --min-reference-motion-m or --segment-s")
    return format_markdown(args.reference, args.estimate, segments)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Analyze relative motion-length bias in visual odometry."
    )
    parser.add_argument("reference", type=Path, help="Reference TUM trajectory.")
    parser.add_argument("estimate", type=Path, help="Visual estimate TUM trajectory.")
    parser.add_argument("--segment-s", type=float, default=1.0)
    parser.add_argument("--stride-s", type=float, default=0.5)
    parser.add_argument("--min-reference-motion-m", type=float, default=0.01)
    parser.add_argument("--out", type=Path, help="Optional markdown output path.")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    text = run_analysis(args)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    print(text)
    if args.out is not None:
        print(f"wrote motion segment analysis to {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
