#!/usr/bin/env python3
"""Analyze Tank DVL velocity as a visual-odometry motion prior."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np

from tank_dvl_prior_core import (
    DvlPriorStep,
    build_dvl_prior_steps,
    finite_values,
    format_fixed,
    format_float,
    stats,
    worst_steps,
)
from tank_rosbag_motion_inputs import (
    DEFAULT_DVL_TOPIC,
    DEFAULT_IMU_TOPIC,
    read_dvl_records,
    read_imu_yaw_records,
)


def load_compare_module():
    script_path = Path(__file__).resolve().parent / "compare_trajectories.py"
    spec = importlib.util.spec_from_file_location("compare_trajectories", script_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def matched_reference_positions(reference_path: Path, visual_path: Path):
    compare = load_compare_module()
    reference = compare.load_tum(reference_path)
    visual = compare.load_tum(visual_path)
    ref_at_visual = compare.interpolate_positions(reference, visual[:, 0])
    valid = ~np.isnan(ref_at_visual).any(axis=1)
    if np.count_nonzero(valid) < 2:
        raise ValueError("need at least two visual timestamps overlapping reference")
    return visual[valid, 0], ref_at_visual[valid], reference


def write_csv(path: Path, steps: list[DvlPriorStep]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "start_stamp_s",
        "end_stamp_s",
        "offset_s",
        "dt_s",
        "dvl_step_m",
        "reference_step_m",
        "length_ratio",
        "direction_cosine",
        "heading_error_deg",
        "dvl_cumulative_m",
        "reference_cumulative_m",
        "dvl_samples",
        "covered",
        "score",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for step in steps:
            writer.writerow({field: getattr(step, field) for field in fieldnames})


def format_stats(label: str, summary: dict) -> str:
    return (
        f"| {label} | {summary['count']} | {format_float(float(summary['min']))} | "
        f"{format_float(float(summary['median']))} | {format_float(float(summary['mean']))} | "
        f"{format_float(float(summary['p95']))} | {format_float(float(summary['max']))} | "
        f"{format_float(float(summary['std']))} |"
    )


def format_markdown(args, steps: list[DvlPriorStep], dvl_count: int, imu_count: int = 0) -> str:
    covered = [step for step in steps if step.covered]
    coverage = len(covered) / len(steps) if steps else math.nan
    dvl_total = covered[-1].dvl_cumulative_m if covered else math.nan
    ref_total = steps[-1].reference_cumulative_m if steps else math.nan
    total_ratio = dvl_total / ref_total if math.isfinite(ref_total) and ref_total > 0.0 else math.nan
    ratio_stats = stats(finite_values(steps, "length_ratio"))
    cosine_stats = stats(finite_values(steps, "direction_cosine"))
    heading_stats = stats([abs(v) for v in finite_values(steps, "heading_error_deg")])
    lines = [
        "# Tank DVL Motion Prior Analysis",
        "",
        f"- Bag: `{args.bag}`",
        f"- DVL topic: `{args.dvl_topic}`",
        f"- IMU topic: `{args.imu_topic}`",
        f"- Reference: `{args.reference}`",
        f"- Visual timestamps: `{args.visual}`",
        f"- Mode: `{args.mode}`",
        f"- DVL frame yaw offset: {format_float(float(args.dvl_frame_yaw_offset_deg), 4)} deg",
        f"- IMU yaw offset: {format_float(float(args.imu_yaw_offset_deg), 4)} deg",
        f"- DVL samples: {dvl_count}",
        f"- IMU yaw samples: {imu_count}",
        f"- Steps: {len(steps)}",
        f"- Covered steps: {len(covered)} ({format_fixed(100.0 * coverage, 1)}%)",
        f"- DVL cumulative distance: {format_float(dvl_total)} m",
        f"- Reference cumulative distance: {format_float(ref_total)} m",
        f"- DVL/reference cumulative ratio: {format_float(total_ratio)}",
        f"- CSV: `{args.csv}`",
        "",
        "## Summary",
        "",
        "| Metric | Count | Min | Median | Mean | P95 | Max | Std |",
        "|--------|------:|----:|-------:|-----:|----:|----:|----:|",
        format_stats("DVL/reference length ratio", ratio_stats),
        format_stats("direction cosine", cosine_stats),
        format_stats("absolute heading error deg", heading_stats),
        "",
        "## Worst Steps",
        "",
        "| Rank | Offset s | dt s | DVL m | Ref m | Ratio | Direction cosine | Heading error deg | Covered | Samples | Score |",
        "|-----:|---------:|-----:|------:|------:|------:|-----------------:|------------------:|:-------:|--------:|------:|",
    ]
    for rank, step in enumerate(worst_steps(steps, args.top_k), start=1):
        lines.append(
            f"| {rank} | {step.offset_s:.3f} | {step.dt_s:.3f} | "
            f"{step.dvl_step_m:.5f} | {step.reference_step_m:.5f} | "
            f"{format_float(step.length_ratio)} | {format_float(step.direction_cosine)} | "
            f"{format_float(step.heading_error_deg)} | {step.covered} | {step.dvl_samples} | "
            f"{format_float(step.score)} |"
        )
    lines.extend(["", "## Interpretation", ""])
    if math.isfinite(total_ratio):
        if total_ratio < 0.8 or total_ratio > 1.2:
            lines.append("- DVL cumulative magnitude is far from reference; scale/bias calibration is required before using it as a magnitude prior.")
        else:
            lines.append("- DVL cumulative magnitude is close enough to investigate as a visual step magnitude prior.")
    median_cosine = float(cosine_stats["median"])
    if math.isfinite(median_cosine) and median_cosine > 0.7:
        if args.mode == "gt_yaw":
            lines.append("- DVL step direction is broadly aligned in this mode; replacing GT yaw with IMU yaw is a sensible next check.")
        else:
            lines.append("- DVL step direction is broadly aligned in this deployable-input mode; this is a candidate motion prior.")
    elif math.isfinite(median_cosine):
        lines.append("- DVL direction is weak in this mode; yaw/frame conventions or sensor quality need more work before fusion.")
    lines.append("")
    return "\n".join(lines)


def run_analysis(args) -> tuple[str, list[DvlPriorStep]]:
    visual_times, reference_xyz, reference_tum = matched_reference_positions(
        args.reference, args.visual
    )
    dvl_records = read_dvl_records(args.bag, args.dvl_topic)
    imu_records = read_imu_yaw_records(args.bag, args.imu_topic) if args.mode == "imu_yaw" else None
    steps = build_dvl_prior_steps(
        visual_times,
        reference_xyz,
        dvl_records,
        reference_tum,
        args.mode,
        args.min_reference_step_m,
        math.radians(args.dvl_frame_yaw_offset_deg),
        imu_records,
        math.radians(args.imu_yaw_offset_deg),
    )
    write_csv(args.csv, steps)
    text = format_markdown(args, steps, len(dvl_records), len(imu_records or []))
    return text, steps


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Analyze Tank DVL velocity as a visual motion prior."
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--visual", required=True, type=Path, help="Visual TUM used only for timestamps.")
    parser.add_argument("--dvl-topic", default=DEFAULT_DVL_TOPIC)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--mode", choices=["body_raw", "gt_yaw", "imu_yaw"], default="gt_yaw")
    parser.add_argument(
        "--dvl-frame-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Yaw rotation from the DVL horizontal velocity axes into the body frame.",
    )
    parser.add_argument(
        "--imu-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Fixed yaw offset added to IMU orientation yaw before rotating DVL velocity.",
    )
    parser.add_argument("--min-reference-step-m", type=float, default=0.0)
    parser.add_argument("--top-k", type=int, default=12)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--csv", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.min_reference_step_m < 0.0:
        raise ValueError("--min-reference-step-m must be non-negative")
    if args.top_k < 0:
        raise ValueError("--top-k must be non-negative")
    text, _ = run_analysis(args)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
