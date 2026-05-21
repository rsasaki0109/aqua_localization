#!/usr/bin/env python3
"""Compare two visual TUM trajectories on their shared timestamp window.

This is intended for replay diagnostics: use the older ROS replay trajectory as
the baseline and the direct rosbag2-sqlite trajectory as the target. The
baseline is interpolated at target timestamps, the target is aligned onto the
baseline, and per-sample errors are written so drift starts can be inspected
directly.
"""

from __future__ import annotations

import argparse
import csv
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
class VisualTrajectoryComparison:
    samples: int
    matched_seconds: float
    alignment_scale: float
    error_mean_m: float
    error_median_m: float
    error_rmse_m: float
    error_max_m: float
    baseline_path_length_m: float
    target_raw_path_length_m: float
    target_aligned_path_length_m: float
    raw_path_length_ratio: float
    aligned_path_length_ratio: float
    yaw_drift_range_deg: float
    yaw_drift_final_deg: float
    drift_start_stamp_s: float | None
    drift_start_offset_s: float | None
    drift_start_error_m: float | None


def yaw_from_quaternion_rows(traj: np.ndarray) -> np.ndarray:
    qx = traj[:, 4]
    qy = traj[:, 5]
    qz = traj[:, 6]
    qw = traj[:, 7]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.unwrap(np.arctan2(siny_cosp, cosy_cosp))


def interpolate_yaw(traj: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    times = traj[:, 0]
    yaw = yaw_from_quaternion_rows(traj)
    out = np.full(query_times.shape[0], np.nan, dtype=np.float64)
    in_range = (query_times >= times[0]) & (query_times <= times[-1])
    if np.any(in_range):
        out[in_range] = np.interp(query_times[in_range], times, yaw)
    return out


def path_length(points: np.ndarray) -> float:
    if points.shape[0] < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def safe_ratio(numerator: float, denominator: float) -> float:
    if denominator == 0.0:
        return math.nan
    return float(numerator / denominator)


def find_drift_start(
    times: np.ndarray,
    errors_m: np.ndarray,
    threshold_m: float,
    consecutive_samples: int,
) -> tuple[float | None, float | None, float | None]:
    if threshold_m <= 0.0:
        raise ValueError("threshold_m must be positive")
    if consecutive_samples < 1:
        raise ValueError("consecutive_samples must be positive")
    if errors_m.shape[0] < consecutive_samples:
        return None, None, None
    over = errors_m >= threshold_m
    for start in range(0, errors_m.shape[0] - consecutive_samples + 1):
        stop = start + consecutive_samples
        if bool(np.all(over[start:stop])):
            return float(times[start]), float(times[start] - times[0]), float(errors_m[start])
    return None, None, None


def matched_trajectory_points(
    baseline: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    compare = load_compare_module()
    baseline_at_target = compare.interpolate_positions(baseline, target[:, 0])
    valid = ~np.isnan(baseline_at_target).any(axis=1)
    if not np.any(valid):
        raise ValueError("no overlapping timestamps between baseline and target")
    times = target[valid, 0]
    target_xyz = target[valid, 1:4]
    baseline_xyz = baseline_at_target[valid]
    target_yaw = interpolate_yaw(target, times)
    baseline_yaw = interpolate_yaw(baseline, times)
    return times, baseline_xyz, target_xyz, baseline_yaw, target_yaw


def compare_visual_trajectories(
    baseline_path: Path,
    target_path: Path,
    *,
    with_scale: bool,
    no_align: bool,
    drift_threshold_m: float,
    drift_consecutive_samples: int,
) -> tuple[VisualTrajectoryComparison, list[dict[str, float]]]:
    compare = load_compare_module()
    baseline = compare.load_tum(baseline_path)
    target = compare.load_tum(target_path)
    times, baseline_xyz, target_xyz, baseline_yaw, target_yaw = matched_trajectory_points(
        baseline, target
    )

    if no_align:
        rotation = np.eye(3)
        translation = np.zeros(3)
        scale = 1.0
    else:
        rotation, translation, scale = compare.umeyama_alignment(
            target_xyz, baseline_xyz, with_scale
        )
    target_aligned_xyz = compare.apply_transform(target_xyz, rotation, translation, scale)
    errors_m = np.linalg.norm(target_aligned_xyz - baseline_xyz, axis=1)
    stats = compare.ape_statistics(errors_m)

    yaw_delta = target_yaw - baseline_yaw
    yaw_valid = ~np.isnan(yaw_delta)
    if np.any(yaw_valid):
        yaw_drift = np.unwrap(yaw_delta[yaw_valid])
        yaw_drift = yaw_drift - yaw_drift[0]
        yaw_drift_range_deg = float(math.degrees(yaw_drift.max() - yaw_drift.min()))
        yaw_drift_final_deg = float(math.degrees(yaw_drift[-1]))
    else:
        yaw_drift_range_deg = math.nan
        yaw_drift_final_deg = math.nan

    drift_stamp, drift_offset, drift_error = find_drift_start(
        times,
        errors_m,
        drift_threshold_m,
        drift_consecutive_samples,
    )

    baseline_len = path_length(baseline_xyz)
    target_raw_len = path_length(target_xyz)
    target_aligned_len = path_length(target_aligned_xyz)
    summary = VisualTrajectoryComparison(
        samples=int(times.shape[0]),
        matched_seconds=float(times.max() - times.min()),
        alignment_scale=float(scale),
        error_mean_m=float(stats["mean"]),
        error_median_m=float(stats["median"]),
        error_rmse_m=float(stats["rmse"]),
        error_max_m=float(stats["max"]),
        baseline_path_length_m=baseline_len,
        target_raw_path_length_m=target_raw_len,
        target_aligned_path_length_m=target_aligned_len,
        raw_path_length_ratio=safe_ratio(target_raw_len, baseline_len),
        aligned_path_length_ratio=safe_ratio(target_aligned_len, baseline_len),
        yaw_drift_range_deg=yaw_drift_range_deg,
        yaw_drift_final_deg=yaw_drift_final_deg,
        drift_start_stamp_s=drift_stamp,
        drift_start_offset_s=drift_offset,
        drift_start_error_m=drift_error,
    )

    rows = []
    for i, stamp_s in enumerate(times):
        rows.append({
            "stamp_s": float(stamp_s),
            "offset_s": float(stamp_s - times[0]),
            "baseline_x_m": float(baseline_xyz[i, 0]),
            "baseline_y_m": float(baseline_xyz[i, 1]),
            "baseline_z_m": float(baseline_xyz[i, 2]),
            "target_raw_x_m": float(target_xyz[i, 0]),
            "target_raw_y_m": float(target_xyz[i, 1]),
            "target_raw_z_m": float(target_xyz[i, 2]),
            "target_aligned_x_m": float(target_aligned_xyz[i, 0]),
            "target_aligned_y_m": float(target_aligned_xyz[i, 1]),
            "target_aligned_z_m": float(target_aligned_xyz[i, 2]),
            "error_m": float(errors_m[i]),
        })
    return summary, rows


def format_float(value: float | None, precision: int = 4) -> str:
    if value is None:
        return "n/a"
    if not math.isfinite(float(value)):
        return "n/a"
    return f"{float(value):.{precision}f}"


def format_markdown(
    baseline_path: Path,
    target_path: Path,
    summary: VisualTrajectoryComparison,
    *,
    with_scale: bool,
    no_align: bool,
    drift_threshold_m: float,
    drift_consecutive_samples: int,
    csv_path: Path | None,
) -> str:
    alignment = "none" if no_align else "Sim(3)" if with_scale else "SE(3)"
    lines = [
        "# Visual Trajectory Comparison",
        "",
        f"- Baseline: `{baseline_path}`",
        f"- Target: `{target_path}`",
        f"- Alignment: `{alignment}`",
        f"- Samples: {summary.samples}",
        f"- Matched duration: {summary.matched_seconds:.2f} s",
        f"- RMSE: {summary.error_rmse_m:.4f} m",
        f"- Mean / median / max: {summary.error_mean_m:.4f} / {summary.error_median_m:.4f} / {summary.error_max_m:.4f} m",
        f"- Alignment scale: {summary.alignment_scale:.9f}",
        "",
        "## Motion Length",
        "",
        f"- Baseline path length: {summary.baseline_path_length_m:.4f} m",
        f"- Target raw path length: {summary.target_raw_path_length_m:.4f} m",
        f"- Target aligned path length: {summary.target_aligned_path_length_m:.4f} m",
        f"- Raw / baseline length ratio: {format_float(summary.raw_path_length_ratio, 6)}",
        f"- Aligned / baseline length ratio: {format_float(summary.aligned_path_length_ratio, 6)}",
        "",
        "## Drift",
        "",
        f"- Threshold: {drift_threshold_m:.4f} m for {drift_consecutive_samples} consecutive samples",
        f"- Drift start offset: {format_float(summary.drift_start_offset_s)} s",
        f"- Drift start stamp: {format_float(summary.drift_start_stamp_s, 9)}",
        f"- Drift start error: {format_float(summary.drift_start_error_m)} m",
        f"- Yaw drift range: {format_float(summary.yaw_drift_range_deg)} deg",
        f"- Final yaw drift: {format_float(summary.yaw_drift_final_deg)} deg",
    ]
    if csv_path is not None:
        lines.append(f"- Per-sample CSV: `{csv_path}`")
    lines.append("")
    return "\n".join(lines)


def write_error_csv(path: Path, rows: list[dict[str, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "stamp_s",
        "offset_s",
        "baseline_x_m",
        "baseline_y_m",
        "baseline_z_m",
        "target_raw_x_m",
        "target_raw_y_m",
        "target_raw_z_m",
        "target_aligned_x_m",
        "target_aligned_y_m",
        "target_aligned_z_m",
        "error_m",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Compare two visual TUM trajectories and emit replay-drift diagnostics."
    )
    parser.add_argument("--baseline", required=True, type=Path, help="Baseline visual TUM trajectory.")
    parser.add_argument("--target", required=True, type=Path, help="Target visual TUM trajectory.")
    parser.add_argument("--out", type=Path, help="Optional Markdown report path.")
    parser.add_argument("--csv", type=Path, help="Optional per-sample CSV path.")
    parser.add_argument("--scale", action="store_true", help="Use Sim(3) instead of SE(3) alignment.")
    parser.add_argument("--no-align", action="store_true", help="Compare raw positions without alignment.")
    parser.add_argument("--drift-threshold-m", type=float, default=0.05)
    parser.add_argument("--drift-consecutive-samples", type=int, default=5)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    summary, rows = compare_visual_trajectories(
        args.baseline,
        args.target,
        with_scale=args.scale,
        no_align=args.no_align,
        drift_threshold_m=args.drift_threshold_m,
        drift_consecutive_samples=args.drift_consecutive_samples,
    )
    if args.csv is not None:
        write_error_csv(args.csv, rows)
    report = format_markdown(
        args.baseline,
        args.target,
        summary,
        with_scale=args.scale,
        no_align=args.no_align,
        drift_threshold_m=args.drift_threshold_m,
        drift_consecutive_samples=args.drift_consecutive_samples,
        csv_path=args.csv,
    )
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
    print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
