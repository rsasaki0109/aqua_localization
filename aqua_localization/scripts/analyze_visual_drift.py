#!/usr/bin/env python3
"""Analyze visual-odometry scale stability and drift from TUM trajectories.

The tool compares a reference trajectory and a visual estimate in sliding time
windows. Each window reports rigid SE(3) APE, Sim(3) APE, and the Sim(3) scale
that would map the visual estimate to the reference. Stable scale with rising
SE(3)/Sim(3) error points to drift; unstable scale points to calibration or
stereo scale issues.
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
class WindowDrift:
    start_s: float
    end_s: float
    samples: int
    se3_rmse_m: float
    sim3_rmse_m: float
    sim3_scale: float
    se3_max_m: float
    sim3_max_m: float


def matched_positions(reference_path: Path, estimate_path: Path):
    compare = load_compare_module()
    ref = compare.load_tum(reference_path)
    est = compare.load_tum(estimate_path)
    ref_at_est = compare.interpolate_positions(ref, est[:, 0])
    valid = ~np.isnan(ref_at_est).any(axis=1)
    if not np.any(valid):
        raise ValueError("no overlapping timestamps between reference and estimate")
    return est[valid, 0], est[valid, 1:4], ref_at_est[valid], compare


def compare_points(est_xyz: np.ndarray, ref_xyz: np.ndarray, with_scale: bool, compare) -> dict:
    if est_xyz.shape[0] < 2:
        raise ValueError("need at least two samples for alignment")
    rotation, translation, scale = compare.umeyama_alignment(est_xyz, ref_xyz, with_scale)
    aligned = compare.apply_transform(est_xyz, rotation, translation, scale)
    errors = np.linalg.norm(aligned - ref_xyz, axis=1)
    stats = compare.ape_statistics(errors)
    stats["scale"] = float(scale)
    return stats


def sliding_windows(
    times: np.ndarray,
    est_xyz: np.ndarray,
    ref_xyz: np.ndarray,
    window_s: float,
    stride_s: float,
    min_samples: int,
    compare,
) -> list[WindowDrift]:
    if window_s <= 0.0:
        raise ValueError("window_s must be positive")
    if stride_s <= 0.0:
        raise ValueError("stride_s must be positive")
    if min_samples < 2:
        raise ValueError("min_samples must be at least 2")

    start = float(times.min())
    end = float(times.max())
    windows = []
    current = start
    while current <= end:
        stop = current + window_s
        mask = (times >= current) & (times <= stop)
        if int(np.count_nonzero(mask)) >= min_samples:
            se3 = compare_points(est_xyz[mask], ref_xyz[mask], False, compare)
            sim3 = compare_points(est_xyz[mask], ref_xyz[mask], True, compare)
            windows.append(WindowDrift(
                start_s=current - start,
                end_s=min(stop, end) - start,
                samples=int(np.count_nonzero(mask)),
                se3_rmse_m=float(se3["rmse"]),
                sim3_rmse_m=float(sim3["rmse"]),
                sim3_scale=float(sim3["scale"]),
                se3_max_m=float(se3["max"]),
                sim3_max_m=float(sim3["max"]),
            ))
        current += stride_s
    return windows


def scale_stats(windows: list[WindowDrift]) -> dict:
    scales = np.asarray([window.sim3_scale for window in windows], dtype=np.float64)
    if scales.size == 0:
        return {"count": 0, "mean": math.nan, "std": math.nan, "min": math.nan, "max": math.nan}
    mean = float(scales.mean())
    return {
        "count": int(scales.size),
        "mean": mean,
        "std": float(scales.std()),
        "min": float(scales.min()),
        "max": float(scales.max()),
        "relative_std": float(scales.std() / mean) if mean != 0.0 else math.nan,
    }


def format_float(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.6g}"


def interpretation(overall_se3: dict, overall_sim3: dict, scale_summary: dict) -> list[str]:
    hints = []
    se3_rmse = float(overall_se3["rmse"])
    sim3_rmse = float(overall_sim3["rmse"])
    rel_std = float(scale_summary.get("relative_std", math.nan))
    if math.isfinite(rel_std) and rel_std > 0.05:
        hints.append("Window scale varies by more than 5%; fixed stereo scale is likely not stable enough.")
    elif math.isfinite(rel_std):
        hints.append("Window scale is fairly stable; a fixed scale calibration is plausible.")

    if sim3_rmse > 0.5 * se3_rmse:
        hints.append("Sim(3) alignment does not remove most error; drift or geometry errors dominate.")
    else:
        hints.append("Sim(3) alignment removes most error; scale/extrinsic calibration is the first target.")
    return hints


def format_markdown(
    reference: Path,
    estimate: Path,
    overall_se3: dict,
    overall_sim3: dict,
    windows: list[WindowDrift],
) -> str:
    scale_summary = scale_stats(windows)
    lines = [
        "# Visual Drift Analysis",
        "",
        f"- Reference: `{reference}`",
        f"- Estimate: `{estimate}`",
        f"- Samples: {overall_se3['count']}",
        f"- Matched duration: {overall_se3['matched_seconds']:.2f} s",
        f"- Overall SE(3) RMSE: {overall_se3['rmse']:.4f} m",
        f"- Overall Sim(3) RMSE: {overall_sim3['rmse']:.4f} m",
        f"- Overall Sim(3) scale: {overall_sim3['alignment']['scale']:.9f}",
        "",
        "## Scale Stability",
        "",
        f"- Windows: {scale_summary['count']}",
        f"- Mean scale: {format_float(float(scale_summary['mean']))}",
        f"- Std scale: {format_float(float(scale_summary['std']))}",
        f"- Relative std: {format_float(float(scale_summary.get('relative_std', math.nan)))}",
        f"- Min / max scale: {format_float(float(scale_summary['min']))} / {format_float(float(scale_summary['max']))}",
        "",
        "## Window Drift",
        "",
        "| Start s | End s | Samples | SE(3) RMSE m | Sim(3) RMSE m | Sim(3) scale | SE(3) max m | Sim(3) max m |",
        "|--------:|------:|--------:|-------------:|--------------:|-------------:|------------:|-------------:|",
    ]
    for window in windows:
        lines.append(
            f"| {window.start_s:.2f} | {window.end_s:.2f} | {window.samples} | "
            f"{window.se3_rmse_m:.4f} | {window.sim3_rmse_m:.4f} | "
            f"{window.sim3_scale:.9f} | {window.se3_max_m:.4f} | {window.sim3_max_m:.4f} |"
        )
    lines.extend(["", "## Interpretation", ""])
    for hint in interpretation(overall_se3, overall_sim3, scale_summary):
        lines.append(f"- {hint}")
    lines.append("")
    return "\n".join(lines)


def run_analysis(args) -> str:
    times, est_xyz, ref_xyz, compare = matched_positions(args.reference, args.estimate)
    overall_se3, _ = compare.compare(args.reference, args.estimate, with_scale=False, no_align=False)
    overall_sim3, _ = compare.compare(args.reference, args.estimate, with_scale=True, no_align=False)
    windows = sliding_windows(
        times,
        est_xyz,
        ref_xyz,
        args.window_s,
        args.stride_s,
        args.min_samples,
        compare,
    )
    if not windows:
        raise ValueError("no windows had enough samples; reduce --min-samples or --window-s")
    return format_markdown(args.reference, args.estimate, overall_se3, overall_sim3, windows)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Analyze visual odometry scale stability and drift with sliding windows."
    )
    parser.add_argument("reference", type=Path, help="Reference TUM trajectory.")
    parser.add_argument("estimate", type=Path, help="Visual estimate TUM trajectory.")
    parser.add_argument("--window-s", type=float, default=3.0)
    parser.add_argument("--stride-s", type=float, default=1.0)
    parser.add_argument("--min-samples", type=int, default=20)
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
        print(f"wrote drift analysis to {args.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
