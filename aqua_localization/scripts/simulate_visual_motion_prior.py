#!/usr/bin/env python3
"""Simulate a motion-prior gate for visual odometry TUM trajectories.

This is an offline upper-bound diagnostic. It uses the reference trajectory as a
stand-in for a future IMU/DVL motion prior, applies that prior to visual steps
that fail simple magnitude/direction gates, and evaluates the corrected path.
Do not use this as a paper metric; use it to decide whether motion-prior work is
likely to pay off.
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

import analyze_visual_step_errors as step_errors


def load_compare_module():
    script_path = Path(__file__).resolve().parent / "compare_trajectories.py"
    spec = importlib.util.spec_from_file_location("compare_trajectories", script_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class PriorStep:
    start_stamp_s: float
    end_stamp_s: float
    offset_s: float
    dt_s: float
    visual_step_m: float
    prior_step_m: float
    corrected_step_m: float
    length_ratio: float
    direction_cosine: float
    heading_error_deg: float
    used_prior: bool
    reason: str


@dataclass(frozen=True)
class PriorSimulationResult:
    original_rmse_m: float
    corrected_rmse_m: float
    original_matched_s: float
    corrected_matched_s: float
    samples: int
    steps: int
    prior_steps: int
    prior_ratio: float
    corrected_tum: Path


def tum_rows_from_positions(times: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    rows = np.zeros((times.shape[0], 8), dtype=np.float64)
    rows[:, 0] = times
    rows[:, 1:4] = xyz
    rows[:, 7] = 1.0
    return rows


def write_tum(path: Path, times: np.ndarray, xyz: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = tum_rows_from_positions(times, xyz)
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(
                f"{row[0]:.9f} {row[1]:.9f} {row[2]:.9f} {row[3]:.9f} "
                "0.000000000 0.000000000 0.000000000 1.000000000\n"
            )


def step_reason(
    visual_delta: np.ndarray,
    prior_delta: np.ndarray,
    *,
    min_reference_step_m: float,
    min_length_ratio: float,
    max_length_ratio: float,
    min_direction_cosine: float,
) -> tuple[bool, str, float, float, float, float]:
    visual_step = float(np.linalg.norm(visual_delta))
    prior_step = float(np.linalg.norm(prior_delta))
    if prior_step < min_reference_step_m:
        return False, "small prior step", visual_step, prior_step, math.nan, math.nan

    length_ratio = visual_step / prior_step if prior_step > 0.0 else math.nan
    cosine = step_errors.direction_cosine(visual_delta, prior_delta)
    reasons = []
    if math.isfinite(length_ratio) and length_ratio < min_length_ratio:
        reasons.append("short step")
    if math.isfinite(length_ratio) and length_ratio > max_length_ratio:
        reasons.append("long step")
    if math.isfinite(cosine) and cosine < min_direction_cosine:
        reasons.append("direction mismatch")
    if not math.isfinite(cosine):
        reasons.append("undefined direction")
    return bool(reasons), "; ".join(reasons) if reasons else "visual", visual_step, prior_step, length_ratio, cosine


def corrected_delta_for_mode(
    visual_delta: np.ndarray,
    prior_delta: np.ndarray,
    *,
    mode: str,
    blend_alpha: float,
    use_prior: bool,
) -> np.ndarray:
    if mode == "replace-outliers":
        return prior_delta if use_prior else visual_delta
    if mode == "blend-outliers":
        if use_prior:
            return (1.0 - blend_alpha) * visual_delta + blend_alpha * prior_delta
        return visual_delta
    if mode == "blend-all":
        return (1.0 - blend_alpha) * visual_delta + blend_alpha * prior_delta
    raise ValueError(f"unsupported mode: {mode}")


def simulate_prior(
    times: np.ndarray,
    visual_xyz: np.ndarray,
    prior_xyz: np.ndarray,
    *,
    mode: str,
    blend_alpha: float,
    min_reference_step_m: float,
    min_length_ratio: float,
    max_length_ratio: float,
    min_direction_cosine: float,
) -> tuple[np.ndarray, list[PriorStep]]:
    if not 0.0 <= blend_alpha <= 1.0:
        raise ValueError("blend_alpha must be in [0, 1]")
    if min_length_ratio < 0.0:
        raise ValueError("min_length_ratio must be non-negative")
    if max_length_ratio <= 0.0 or max_length_ratio < min_length_ratio:
        raise ValueError("max_length_ratio must be positive and >= min_length_ratio")
    if not -1.0 <= min_direction_cosine <= 1.0:
        raise ValueError("min_direction_cosine must be in [-1, 1]")

    corrected = np.zeros_like(visual_xyz)
    corrected[0] = visual_xyz[0]
    rows = []
    t0 = float(times[0])
    for i in range(1, times.shape[0]):
        visual_delta = visual_xyz[i] - visual_xyz[i - 1]
        prior_delta = prior_xyz[i] - prior_xyz[i - 1]
        use_prior, reason, visual_step, prior_step, length_ratio, cosine = step_reason(
            visual_delta,
            prior_delta,
            min_reference_step_m=min_reference_step_m,
            min_length_ratio=min_length_ratio,
            max_length_ratio=max_length_ratio,
            min_direction_cosine=min_direction_cosine,
        )
        if mode == "blend-all":
            use_prior = True
            if reason == "visual":
                reason = "blend all"
        corrected_delta = corrected_delta_for_mode(
            visual_delta,
            prior_delta,
            mode=mode,
            blend_alpha=blend_alpha,
            use_prior=use_prior,
        )
        corrected[i] = corrected[i - 1] + corrected_delta
        heading_error = step_errors.angle_difference_deg(
            step_errors.yaw_from_delta(visual_delta),
            step_errors.yaw_from_delta(prior_delta),
        )
        rows.append(PriorStep(
            start_stamp_s=float(times[i - 1]),
            end_stamp_s=float(times[i]),
            offset_s=float(times[i] - t0),
            dt_s=float(times[i] - times[i - 1]),
            visual_step_m=visual_step,
            prior_step_m=prior_step,
            corrected_step_m=float(np.linalg.norm(corrected_delta)),
            length_ratio=length_ratio,
            direction_cosine=cosine,
            heading_error_deg=heading_error,
            used_prior=use_prior,
            reason=reason,
        ))
    return corrected, rows


def run_simulation(args) -> tuple[PriorSimulationResult, list[PriorStep]]:
    compare = load_compare_module()
    times, visual_xyz, prior_xyz = step_errors.matched_aligned_positions(args.reference, args.estimate)
    corrected_xyz, rows = simulate_prior(
        times,
        visual_xyz,
        prior_xyz,
        mode=args.mode,
        blend_alpha=args.blend_alpha,
        min_reference_step_m=args.min_reference_step_m,
        min_length_ratio=args.min_length_ratio,
        max_length_ratio=args.max_length_ratio,
        min_direction_cosine=args.min_direction_cosine,
    )
    write_tum(args.corrected_out, times, corrected_xyz)
    original_path = args.out_dir / "aligned_visual_input.tum"
    write_tum(original_path, times, visual_xyz)
    original_stats, _ = compare.compare(args.reference, original_path, with_scale=False, no_align=False)
    corrected_stats, _ = compare.compare(args.reference, args.corrected_out, with_scale=False, no_align=False)
    prior_count = sum(1 for row in rows if row.used_prior)
    result = PriorSimulationResult(
        original_rmse_m=float(original_stats["rmse"]),
        corrected_rmse_m=float(corrected_stats["rmse"]),
        original_matched_s=float(original_stats["matched_seconds"]),
        corrected_matched_s=float(corrected_stats["matched_seconds"]),
        samples=int(corrected_stats["count"]),
        steps=len(rows),
        prior_steps=prior_count,
        prior_ratio=prior_count / len(rows) if rows else math.nan,
        corrected_tum=args.corrected_out,
    )
    return result, rows


def write_step_csv(path: Path, rows: list[PriorStep]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "start_stamp_s",
        "end_stamp_s",
        "offset_s",
        "dt_s",
        "visual_step_m",
        "prior_step_m",
        "corrected_step_m",
        "length_ratio",
        "direction_cosine",
        "heading_error_deg",
        "used_prior",
        "reason",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: getattr(row, field) for field in fieldnames})


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def format_markdown(args, result: PriorSimulationResult, rows: list[PriorStep]) -> str:
    delta = result.original_rmse_m - result.corrected_rmse_m
    improvement = 100.0 * delta / result.original_rmse_m if result.original_rmse_m > 0.0 else math.nan
    reason_counts: dict[str, int] = {}
    for row in rows:
        if row.used_prior:
            reason_counts[row.reason] = reason_counts.get(row.reason, 0) + 1
    lines = [
        "# Visual Motion Prior Simulation",
        "",
        f"- Reference/prior: `{args.reference}`",
        f"- Visual estimate: `{args.estimate}`",
        f"- Mode: `{args.mode}`",
        f"- Blend alpha: {format_float(args.blend_alpha, 3)}",
        f"- Length-ratio gate: [{format_float(args.min_length_ratio, 3)}, {format_float(args.max_length_ratio, 3)}]",
        f"- Min direction cosine: {format_float(args.min_direction_cosine, 3)}",
        f"- Original RMSE: {result.original_rmse_m:.4f} m",
        f"- Corrected RMSE: {result.corrected_rmse_m:.4f} m",
        f"- RMSE improvement: {format_float(improvement, 1)}%",
        f"- Prior-applied steps: {result.prior_steps}/{result.steps} ({format_float(100.0 * result.prior_ratio, 1)}%)",
        f"- Corrected TUM: `{result.corrected_tum}`",
        f"- Step CSV: `{args.csv_out}`",
        "",
        "## Prior Reasons",
        "",
        "| Reason | Count |",
        "|--------|------:|",
    ]
    if reason_counts:
        for reason, count in sorted(reason_counts.items(), key=lambda item: (-item[1], item[0])):
            lines.append(f"| {reason} | {count} |")
    else:
        lines.append("| none | 0 |")
    lines.extend([
        "",
        "## Interpretation",
        "",
    ])
    if result.corrected_rmse_m < result.original_rmse_m:
        lines.append("- A motion prior with these gates could reduce the current visual error upper bound.")
    else:
        lines.append("- These gates do not reduce the current visual error; try a different prior mode or thresholds.")
    lines.append("- This uses the reference trajectory as an oracle prior and is not a paper-safe benchmark result.")
    lines.append("")
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Simulate an oracle motion-prior gate for visual odometry."
    )
    parser.add_argument("reference", type=Path, help="Reference TUM trajectory used as oracle prior.")
    parser.add_argument("estimate", type=Path, help="Visual estimate TUM trajectory.")
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_visual_motion_prior_sim"))
    parser.add_argument("--summary-out", type=Path, default=None)
    parser.add_argument("--csv-out", type=Path, default=None)
    parser.add_argument("--corrected-out", type=Path, default=None)
    parser.add_argument(
        "--mode",
        choices=["replace-outliers", "blend-outliers", "blend-all"],
        default="replace-outliers",
    )
    parser.add_argument("--blend-alpha", type=float, default=0.5)
    parser.add_argument("--min-reference-step-m", type=float, default=0.0)
    parser.add_argument("--min-length-ratio", type=float, default=0.5)
    parser.add_argument("--max-length-ratio", type=float, default=1.5)
    parser.add_argument("--min-direction-cosine", type=float, default=0.0)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.blend_alpha < 0.0 or args.blend_alpha > 1.0:
        raise ValueError("--blend-alpha must be in [0, 1]")
    if args.min_reference_step_m < 0.0:
        raise ValueError("--min-reference-step-m must be non-negative")
    if args.summary_out is None:
        args.summary_out = args.out_dir / "visual_motion_prior_sim.md"
    if args.csv_out is None:
        args.csv_out = args.out_dir / "visual_motion_prior_steps.csv"
    if args.corrected_out is None:
        args.corrected_out = args.out_dir / "visual_motion_prior_corrected.tum"
    args.out_dir.mkdir(parents=True, exist_ok=True)
    result, rows = run_simulation(args)
    write_step_csv(args.csv_out, rows)
    summary = format_markdown(args, result, rows)
    args.summary_out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_out.write_text(summary, encoding="utf-8")
    print(summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
