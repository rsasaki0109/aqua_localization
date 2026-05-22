#!/usr/bin/env python3
"""Analyze per-step visual odometry motion errors against a reference TUM trajectory."""

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
class StepError:
    start_stamp_s: float
    end_stamp_s: float
    offset_s: float
    dt_s: float
    visual_step_m: float
    reference_step_m: float
    length_error_m: float
    length_ratio: float
    direction_cosine: float
    heading_error_deg: float
    visual_cumulative_m: float
    reference_cumulative_m: float
    score: float


def yaw_from_delta(delta_xyz: np.ndarray) -> float:
    if float(np.linalg.norm(delta_xyz[:2])) <= 1.0e-12:
        return math.nan
    return math.atan2(float(delta_xyz[1]), float(delta_xyz[0]))


def angle_difference_deg(a: float, b: float) -> float:
    if not math.isfinite(a) or not math.isfinite(b):
        return math.nan
    diff = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(diff)


def direction_cosine(a: np.ndarray, b: np.ndarray) -> float:
    norm_a = float(np.linalg.norm(a))
    norm_b = float(np.linalg.norm(b))
    if norm_a <= 1.0e-12 or norm_b <= 1.0e-12:
        return math.nan
    return float(np.dot(a, b) / (norm_a * norm_b))


def matched_aligned_positions(reference_path: Path, estimate_path: Path):
    compare = load_compare_module()
    ref = compare.load_tum(reference_path)
    est = compare.load_tum(estimate_path)
    ref_at_est = compare.interpolate_positions(ref, est[:, 0])
    valid = ~np.isnan(ref_at_est).any(axis=1)
    if np.count_nonzero(valid) < 2:
        raise ValueError("need at least two overlapping trajectory samples")
    times = est[valid, 0]
    est_xyz = est[valid, 1:4]
    ref_xyz = ref_at_est[valid]
    rotation, translation, scale = compare.umeyama_alignment(est_xyz, ref_xyz, with_scale=False)
    aligned_est = compare.apply_transform(est_xyz, rotation, translation, scale)
    return times, aligned_est, ref_xyz


def build_step_errors(
    times: np.ndarray,
    visual_xyz: np.ndarray,
    reference_xyz: np.ndarray,
    min_reference_step_m: float,
) -> list[StepError]:
    if min_reference_step_m < 0.0:
        raise ValueError("min_reference_step_m must be non-negative")
    visual_cumulative = 0.0
    reference_cumulative = 0.0
    steps = []
    t0 = float(times[0])
    for i in range(1, times.shape[0]):
        dt_s = float(times[i] - times[i - 1])
        if dt_s <= 0.0:
            continue
        visual_delta = visual_xyz[i] - visual_xyz[i - 1]
        reference_delta = reference_xyz[i] - reference_xyz[i - 1]
        visual_step = float(np.linalg.norm(visual_delta))
        reference_step = float(np.linalg.norm(reference_delta))
        visual_cumulative += visual_step
        reference_cumulative += reference_step
        if reference_step < min_reference_step_m:
            continue
        length_error = visual_step - reference_step
        ratio = visual_step / reference_step if reference_step > 0.0 else math.nan
        cosine = direction_cosine(visual_delta, reference_delta)
        heading_error = angle_difference_deg(yaw_from_delta(visual_delta), yaw_from_delta(reference_delta))
        # Score favors steps that are wrong in length and/or direction while still moving.
        score = abs(length_error)
        if math.isfinite(cosine):
            score += reference_step * max(0.0, 1.0 - cosine)
        steps.append(StepError(
            start_stamp_s=float(times[i - 1]),
            end_stamp_s=float(times[i]),
            offset_s=float(times[i] - t0),
            dt_s=dt_s,
            visual_step_m=visual_step,
            reference_step_m=reference_step,
            length_error_m=length_error,
            length_ratio=ratio,
            direction_cosine=cosine,
            heading_error_deg=heading_error,
            visual_cumulative_m=visual_cumulative,
            reference_cumulative_m=reference_cumulative,
            score=score,
        ))
    return steps


def finite_values(steps: list[StepError], attr: str) -> list[float]:
    values = []
    for step in steps:
        value = float(getattr(step, attr))
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
        return {
            "count": 0,
            "min": math.nan,
            "median": math.nan,
            "mean": math.nan,
            "p95": math.nan,
            "max": math.nan,
            "std": math.nan,
        }
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "min": float(arr.min()),
        "median": percentile(values, 0.5),
        "mean": float(arr.mean()),
        "p95": percentile(values, 0.95),
        "max": float(arr.max()),
        "std": float(arr.std()),
    }


def format_float(value: float, precision: int = 6) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}g}"


def format_stats(label: str, summary: dict) -> str:
    return (
        f"| {label} | {summary['count']} | {format_float(float(summary['min']))} | "
        f"{format_float(float(summary['median']))} | {format_float(float(summary['mean']))} | "
        f"{format_float(float(summary['p95']))} | {format_float(float(summary['max']))} | "
        f"{format_float(float(summary['std']))} |"
    )


def worst_steps(steps: list[StepError], top_k: int) -> list[StepError]:
    return sorted(steps, key=lambda step: step.score, reverse=True)[:max(0, top_k)]


def interpretation(steps: list[StepError]) -> list[str]:
    hints = []
    if not steps:
        return ["No valid steps were available; lower the minimum reference motion threshold."]
    ratio_stats = stats(finite_values(steps, "length_ratio"))
    cosine_stats = stats(finite_values(steps, "direction_cosine"))
    total_visual = steps[-1].visual_cumulative_m
    total_ref = steps[-1].reference_cumulative_m
    total_ratio = total_visual / total_ref if total_ref > 0.0 else math.nan
    if math.isfinite(total_ratio):
        if total_ratio < 0.9 or total_ratio > 1.1:
            hints.append(f"Cumulative visual/reference distance ratio is {total_ratio:.3f}; scale or motion magnitude bias remains important.")
        else:
            hints.append(f"Cumulative visual/reference distance ratio is {total_ratio:.3f}; per-step direction or local outliers may dominate.")
    if math.isfinite(float(ratio_stats["std"])) and float(ratio_stats["std"]) > 0.25:
        hints.append("Step length ratio varies strongly; a fixed scale alone will not explain the error.")
    if math.isfinite(float(cosine_stats["p95"])) and float(cosine_stats["median"]) < 0.8:
        hints.append("Many step directions disagree with the reference; motion priors should constrain direction, not only step length.")
    elif math.isfinite(float(cosine_stats["median"])):
        hints.append("Median step direction is broadly aligned; target the worst local updates first.")
    return hints


def write_csv(path: Path, steps: list[StepError]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "start_stamp_s",
        "end_stamp_s",
        "offset_s",
        "dt_s",
        "visual_step_m",
        "reference_step_m",
        "length_error_m",
        "length_ratio",
        "direction_cosine",
        "heading_error_deg",
        "visual_cumulative_m",
        "reference_cumulative_m",
        "score",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for step in steps:
            writer.writerow({field: getattr(step, field) for field in fieldnames})


def format_markdown(
    reference: Path,
    estimate: Path,
    steps: list[StepError],
    *,
    top_k: int,
    csv_path: Path | None,
) -> str:
    visual_stats = stats(finite_values(steps, "visual_step_m"))
    reference_stats = stats(finite_values(steps, "reference_step_m"))
    ratio_stats = stats(finite_values(steps, "length_ratio"))
    length_error_stats = stats([abs(v) for v in finite_values(steps, "length_error_m")])
    cosine_stats = stats(finite_values(steps, "direction_cosine"))
    heading_error_stats = stats([abs(v) for v in finite_values(steps, "heading_error_deg")])
    total_visual = steps[-1].visual_cumulative_m if steps else math.nan
    total_ref = steps[-1].reference_cumulative_m if steps else math.nan
    total_ratio = total_visual / total_ref if math.isfinite(total_ref) and total_ref > 0.0 else math.nan

    lines = [
        "# Visual Step Error Analysis",
        "",
        f"- Reference: `{reference}`",
        f"- Estimate: `{estimate}`",
        f"- Steps: {len(steps)}",
        f"- Visual cumulative distance: {format_float(total_visual)} m",
        f"- Reference cumulative distance: {format_float(total_ref)} m",
        f"- Cumulative visual/reference ratio: {format_float(total_ratio)}",
    ]
    if csv_path is not None:
        lines.append(f"- Per-step CSV: `{csv_path}`")
    lines.extend([
        "",
        "## Summary",
        "",
        "| Metric | Count | Min | Median | Mean | P95 | Max | Std |",
        "|--------|------:|----:|-------:|-----:|----:|----:|----:|",
        format_stats("visual step m", visual_stats),
        format_stats("reference step m", reference_stats),
        format_stats("visual/reference length ratio", ratio_stats),
        format_stats("absolute length error m", length_error_stats),
        format_stats("direction cosine", cosine_stats),
        format_stats("absolute heading error deg", heading_error_stats),
        "",
        "## Worst Steps",
        "",
        "| Rank | Offset s | dt s | Visual m | Reference m | Ratio | Direction cosine | Heading error deg | Score |",
        "|-----:|---------:|-----:|---------:|------------:|------:|-----------------:|------------------:|------:|",
    ])
    for rank, step in enumerate(worst_steps(steps, top_k), start=1):
        lines.append(
            f"| {rank} | {step.offset_s:.3f} | {step.dt_s:.3f} | "
            f"{step.visual_step_m:.5f} | {step.reference_step_m:.5f} | "
            f"{format_float(step.length_ratio)} | {format_float(step.direction_cosine)} | "
            f"{format_float(step.heading_error_deg)} | {step.score:.5f} |"
        )
    lines.extend(["", "## Interpretation", ""])
    for hint in interpretation(steps):
        lines.append(f"- {hint}")
    lines.append("")
    return "\n".join(lines)


def run_analysis(args) -> tuple[str, list[StepError]]:
    times, visual_xyz, reference_xyz = matched_aligned_positions(args.reference, args.estimate)
    steps = build_step_errors(
        times,
        visual_xyz,
        reference_xyz,
        min_reference_step_m=args.min_reference_step_m,
    )
    if args.csv is not None:
        write_csv(args.csv, steps)
    return (
        format_markdown(
            args.reference,
            args.estimate,
            steps,
            top_k=args.top_k,
            csv_path=args.csv,
        ),
        steps,
    )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Analyze per-step visual odometry motion errors."
    )
    parser.add_argument("reference", type=Path, help="Reference TUM trajectory.")
    parser.add_argument("estimate", type=Path, help="Visual estimate TUM trajectory.")
    parser.add_argument("--min-reference-step-m", type=float, default=0.0)
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--csv", type=Path, help="Optional per-step CSV output.")
    parser.add_argument("--out", type=Path, help="Optional Markdown output.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.min_reference_step_m < 0.0:
        raise ValueError("--min-reference-step-m must be non-negative")
    if args.top_k < 0:
        raise ValueError("--top-k must be non-negative")
    text, _ = run_analysis(args)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
