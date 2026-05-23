#!/usr/bin/env python3
"""Analyze where a Tank DVL prior validation run still leaves trajectory error."""

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
class ResidualRow:
    index: int
    stamp_s: float
    offset_s: float
    visual_error_m: float
    corrected_error_m: float
    improvement_m: float
    visual_step_m: float
    prior_step_m: float
    corrected_step_m: float
    used_prior: bool
    dvl_covered: bool
    reason: str
    direction_cosine: float
    length_ratio: float
    heading_error_deg: float


def parse_bool(value) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


def parse_float(value, default: float = math.nan) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def read_step_csv(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


def aligned_positions_at_times(reference_path: Path, estimate_path: Path, query_times: np.ndarray):
    compare = load_compare_module()
    reference = compare.load_tum(reference_path)
    estimate = compare.load_tum(estimate_path)
    reference_xyz = compare.interpolate_positions(reference, query_times)
    estimate_xyz = compare.interpolate_positions(estimate, query_times)
    valid = ~np.isnan(reference_xyz).any(axis=1) & ~np.isnan(estimate_xyz).any(axis=1)
    if np.count_nonzero(valid) < 2:
        raise ValueError("need at least two overlapping trajectory samples")
    rotation, translation, scale = compare.umeyama_alignment(
        estimate_xyz[valid],
        reference_xyz[valid],
        with_scale=False,
    )
    aligned = np.full_like(estimate_xyz, np.nan)
    aligned[valid] = compare.apply_transform(estimate_xyz[valid], rotation, translation, scale)
    return aligned, reference_xyz, valid


def build_residual_rows(
    times: np.ndarray,
    visual_xyz: np.ndarray,
    corrected_xyz: np.ndarray,
    reference_xyz: np.ndarray,
    valid: np.ndarray,
    step_rows: list[dict],
) -> list[ResidualRow]:
    rows = []
    t0 = float(times[0])
    max_steps = min(len(step_rows), times.shape[0] - 1)
    for step_index in range(max_steps):
        sample_index = step_index + 1
        if not valid[sample_index]:
            continue
        visual_error = float(np.linalg.norm(visual_xyz[sample_index] - reference_xyz[sample_index]))
        corrected_error = float(np.linalg.norm(corrected_xyz[sample_index] - reference_xyz[sample_index]))
        step = step_rows[step_index]
        rows.append(ResidualRow(
            index=sample_index,
            stamp_s=float(times[sample_index]),
            offset_s=float(times[sample_index] - t0),
            visual_error_m=visual_error,
            corrected_error_m=corrected_error,
            improvement_m=visual_error - corrected_error,
            visual_step_m=parse_float(step.get("visual_step_m")),
            prior_step_m=parse_float(step.get("prior_step_m")),
            corrected_step_m=parse_float(step.get("corrected_step_m")),
            used_prior=parse_bool(step.get("used_prior")),
            dvl_covered=parse_bool(step.get("dvl_covered")),
            reason=str(step.get("reason", "")),
            direction_cosine=parse_float(step.get("visual_prior_direction_cosine")),
            length_ratio=parse_float(step.get("visual_prior_length_ratio")),
            heading_error_deg=parse_float(step.get("visual_prior_heading_error_deg")),
        ))
    return rows


def finite(values: list[float]) -> list[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def percentile(values: list[float], q: float) -> float:
    values = sorted(finite(values))
    if not values:
        return math.nan
    if q <= 0.0:
        return values[0]
    if q >= 1.0:
        return values[-1]
    pos = (len(values) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    alpha = pos - lo
    return values[lo] * (1.0 - alpha) + values[hi] * alpha


def stats(values: list[float]) -> dict:
    values = finite(values)
    if not values:
        return {
            "count": 0,
            "mean": math.nan,
            "median": math.nan,
            "p95": math.nan,
            "max": math.nan,
            "sum": math.nan,
        }
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "median": percentile(values, 0.5),
        "p95": percentile(values, 0.95),
        "max": float(arr.max()),
        "sum": float(arr.sum()),
    }


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def format_stats_row(label: str, rows: list[ResidualRow]) -> str:
    visual = stats([row.visual_error_m for row in rows])
    corrected = stats([row.corrected_error_m for row in rows])
    improvement = stats([row.improvement_m for row in rows])
    return (
        f"| {label} | {len(rows)} | {format_float(visual['mean'])} | "
        f"{format_float(corrected['mean'])} | {format_float(improvement['mean'])} | "
        f"{format_float(improvement['sum'])} | {format_float(corrected['p95'])} | "
        f"{format_float(corrected['max'])} |"
    )


def reason_groups(rows: list[ResidualRow]) -> list[tuple[str, list[ResidualRow]]]:
    groups: dict[str, list[ResidualRow]] = {}
    for row in rows:
        key = row.reason or "none"
        groups.setdefault(key, []).append(row)
    return sorted(groups.items(), key=lambda item: (-len(item[1]), item[0]))


def worst_rows(rows: list[ResidualRow], top_k: int) -> list[ResidualRow]:
    return sorted(rows, key=lambda row: row.corrected_error_m, reverse=True)[:max(0, top_k)]


def regressions(rows: list[ResidualRow], top_k: int) -> list[ResidualRow]:
    regression_rows = [row for row in rows if row.improvement_m < 0.0]
    source = regression_rows if regression_rows else rows
    return sorted(source, key=lambda row: row.improvement_m)[:max(0, top_k)]


def interpretation(rows: list[ResidualRow]) -> list[str]:
    if not rows:
        return ["No residual rows were available."]
    used = [row for row in rows if row.used_prior]
    unused = [row for row in rows if not row.used_prior]
    hints = []
    used_improvement = stats([row.improvement_m for row in used])
    unused_improvement = stats([row.improvement_m for row in unused])
    if used:
        hints.append(
            f"Prior-applied steps have mean residual change {format_float(used_improvement['mean'])} m "
            f"over {len(used)} samples."
        )
    if unused:
        hints.append(
            f"Non-prior steps have mean residual change {format_float(unused_improvement['mean'])} m "
            f"over {len(unused)} samples; remaining gap may sit outside the prior gate."
        )
    bad_regressions = [row for row in rows if row.improvement_m < -0.005]
    if bad_regressions:
        hints.append(
            f"{len(bad_regressions)} samples regress by more than 5 mm; inspect the regression table before loosening gates."
        )
    else:
        hints.append("No sample regresses by more than 5 mm; tighter correction or coverage may be the next lever.")
    return hints


def write_csv(path: Path, rows: list[ResidualRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [field.name for field in ResidualRow.__dataclass_fields__.values()]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: getattr(row, field) for field in fieldnames})


def format_markdown(args, rows: list[ResidualRow]) -> str:
    used = [row for row in rows if row.used_prior]
    unused = [row for row in rows if not row.used_prior]
    covered = [row for row in rows if row.dvl_covered]
    uncovered = [row for row in rows if not row.dvl_covered]
    lines = [
        "# Tank DVL Prior Residual Analysis",
        "",
        f"- Reference: `{args.reference}`",
        f"- Visual aligned TUM: `{args.visual_aligned}`",
        f"- Corrected TUM: `{args.corrected}`",
        f"- Step CSV: `{args.step_csv}`",
        f"- Residual samples: {len(rows)}",
    ]
    if args.csv_out is not None:
        lines.append(f"- Residual CSV: `{args.csv_out}`")
    lines.extend([
        "",
        "## Summary",
        "",
        "| Group | Samples | Mean visual error m | Mean corrected error m | Mean improvement m | Total improvement m | Corrected P95 m | Corrected max m |",
        "|-------|--------:|--------------------:|-----------------------:|-------------------:|--------------------:|----------------:|----------------:|",
        format_stats_row("all", rows),
        format_stats_row("prior applied", used),
        format_stats_row("prior not applied", unused),
        format_stats_row("DVL covered", covered),
        format_stats_row("DVL not covered", uncovered),
        "",
        "## Reason Groups",
        "",
        "| Reason | Samples | Mean visual error m | Mean corrected error m | Mean improvement m | Total improvement m | Corrected P95 m | Corrected max m |",
        "|--------|--------:|--------------------:|-----------------------:|-------------------:|--------------------:|----------------:|----------------:|",
    ])
    for reason, group in reason_groups(rows):
        lines.append(format_stats_row(reason, group))
    lines.extend([
        "",
        "## Worst Corrected Residuals",
        "",
        "| Rank | Offset s | Corrected error m | Visual error m | Improvement m | Used prior | Reason | Ratio | Cosine | Heading deg |",
        "|-----:|---------:|------------------:|---------------:|--------------:|-----------:|--------|------:|-------:|------------:|",
    ])
    for rank, row in enumerate(worst_rows(rows, args.top_k), start=1):
        lines.append(
            f"| {rank} | {row.offset_s:.3f} | {row.corrected_error_m:.4f} | "
            f"{row.visual_error_m:.4f} | {row.improvement_m:.4f} | {row.used_prior} | "
            f"{row.reason} | {format_float(row.length_ratio)} | "
            f"{format_float(row.direction_cosine)} | {format_float(row.heading_error_deg)} |"
        )
    regression_title = "Largest Regressions" if any(row.improvement_m < 0.0 for row in rows) else "Smallest Improvements"
    lines.extend([
        "",
        f"## {regression_title}",
        "",
        "| Rank | Offset s | Corrected error m | Visual error m | Improvement m | Used prior | Reason | Ratio | Cosine | Heading deg |",
        "|-----:|---------:|------------------:|---------------:|--------------:|-----------:|--------|------:|-------:|------------:|",
    ])
    for rank, row in enumerate(regressions(rows, args.top_k), start=1):
        lines.append(
            f"| {rank} | {row.offset_s:.3f} | {row.corrected_error_m:.4f} | "
            f"{row.visual_error_m:.4f} | {row.improvement_m:.4f} | {row.used_prior} | "
            f"{row.reason} | {format_float(row.length_ratio)} | "
            f"{format_float(row.direction_cosine)} | {format_float(row.heading_error_deg)} |"
        )
    lines.extend(["", "## Interpretation", ""])
    for hint in interpretation(rows):
        lines.append(f"- {hint}")
    lines.append("")
    return "\n".join(lines)


def run_analysis(args) -> tuple[str, list[ResidualRow]]:
    corrected_tum = load_compare_module().load_tum(args.corrected)
    times = corrected_tum[:, 0]
    visual_xyz, reference_xyz, visual_valid = aligned_positions_at_times(
        args.reference,
        args.visual_aligned,
        times,
    )
    corrected_xyz, _reference_again, corrected_valid = aligned_positions_at_times(
        args.reference,
        args.corrected,
        times,
    )
    rows = build_residual_rows(
        times,
        visual_xyz,
        corrected_xyz,
        reference_xyz,
        visual_valid & corrected_valid,
        read_step_csv(args.step_csv),
    )
    if args.csv_out is not None:
        write_csv(args.csv_out, rows)
    return format_markdown(args, rows), rows


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Analyze residual error from a Tank DVL prior validation run."
    )
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--visual-aligned", required=True, type=Path)
    parser.add_argument("--corrected", required=True, type=Path)
    parser.add_argument("--step-csv", required=True, type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--csv-out", type=Path)
    parser.add_argument("--top-k", type=int, default=10)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.top_k < 0:
        raise ValueError("--top-k must be non-negative")
    text, _rows = run_analysis(args)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
