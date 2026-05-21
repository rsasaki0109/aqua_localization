#!/usr/bin/env python3
"""Build an AQUA-SLAM-facing error budget from Tank benchmark artifacts."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_gap_report as gap_report


@dataclass(frozen=True)
class DriftMetrics:
    se3_rmse_m: float | None = None
    sim3_rmse_m: float | None = None
    sim3_scale: float | None = None
    scale_relative_std: float | None = None
    worst_window_se3_rmse_m: float | None = None


@dataclass(frozen=True)
class MotionMetrics:
    median_correction_scale: float | None = None
    correction_scale_std: float | None = None
    correction_scale_min: float | None = None
    correction_scale_max: float | None = None


@dataclass(frozen=True)
class BudgetContext:
    baseline: gap_report.BenchmarkRow
    best_target: gap_report.BenchmarkRow
    anchor: gap_report.BenchmarkRow | None
    best_fused: gap_report.BenchmarkRow | None
    drift: DriftMetrics
    motion: MotionMetrics


def parse_optional_float(value: str) -> float | None:
    text = value.strip().strip("`")
    if text.lower() in {"", "n/a", "na", "tbd", "none"}:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def find_metric(text: str, label: str) -> float | None:
    match = re.search(rf"{re.escape(label)}\s*:?\s*`?([-+0-9.eE]+)", text)
    if not match:
        return None
    return parse_optional_float(match.group(1))


def markdown_rows_after_header(text: str, required_header: str) -> list[list[str]]:
    lines = text.splitlines()
    for index, line in enumerate(lines):
        cells = gap_report.split_markdown_row(line)
        if not cells or required_header not in cells:
            continue
        rows: list[list[str]] = []
        for row_line in lines[index + 2 :]:
            row = gap_report.split_markdown_row(row_line)
            if not row:
                break
            if gap_report.is_separator_row(row):
                continue
            rows.append(row)
        return rows
    return []


def parse_drift_report(path: Path | None) -> DriftMetrics:
    if path is None:
        return DriftMetrics()
    text = path.read_text(encoding="utf-8")
    window_rows = markdown_rows_after_header(text, "SE(3) RMSE m")
    worst_window = None
    for row in window_rows:
        if len(row) < 4:
            continue
        value = parse_optional_float(row[3])
        if value is not None and (worst_window is None or value > worst_window):
            worst_window = value
    return DriftMetrics(
        se3_rmse_m=find_metric(text, "- Overall SE(3) RMSE"),
        sim3_rmse_m=find_metric(text, "- Overall Sim(3) RMSE"),
        sim3_scale=find_metric(text, "- Overall Sim(3) scale"),
        scale_relative_std=find_metric(text, "- Relative std"),
        worst_window_se3_rmse_m=worst_window,
    )


def parse_motion_report(path: Path | None) -> MotionMetrics:
    if path is None:
        return MotionMetrics()
    text = path.read_text(encoding="utf-8")
    rows = markdown_rows_after_header(text, "Metric")
    for row in rows:
        if not row or row[0] != "reference/visual correction scale":
            continue
        values = [parse_optional_float(cell) for cell in row]
        return MotionMetrics(
            correction_scale_min=values[2] if len(values) > 2 else None,
            median_correction_scale=values[3] if len(values) > 3 else None,
            correction_scale_max=values[6] if len(values) > 6 else None,
            correction_scale_std=values[7] if len(values) > 7 else None,
        )
    return MotionMetrics()


def best_row(
    rows: list[gap_report.BenchmarkRow],
    system: str,
) -> gap_report.BenchmarkRow | None:
    matching = [
        row for row in gap_report.dedupe_rows(rows)
        if gap_report.matching_system(row, system) and row.rmse_m is not None
    ]
    return min(matching, key=lambda row: row.rmse_m) if matching else None


def best_prefixed_row(
    rows: list[gap_report.BenchmarkRow],
    prefixes: list[str],
    baseline_system: str,
) -> gap_report.BenchmarkRow | None:
    candidates = []
    for row in gap_report.dedupe_rows(rows):
        if row.rmse_m is None or gap_report.matching_system(row, baseline_system):
            continue
        system = row.system.lower()
        if any(system.startswith(prefix.lower()) for prefix in prefixes):
            candidates.append(row)
    return min(candidates, key=lambda row: row.rmse_m) if candidates else None


def load_context(args: argparse.Namespace) -> BudgetContext:
    rows: list[gap_report.BenchmarkRow] = []
    for path in args.markdown:
        rows.extend(gap_report.parse_markdown_benchmark_rows(path.read_text(encoding="utf-8")))
    baseline = best_row(rows, args.baseline_system)
    best_target = best_prefixed_row(rows, args.target_prefix, args.baseline_system)
    if baseline is None:
        raise ValueError(f"no measured `{args.baseline_system}` row found")
    if best_target is None:
        raise ValueError("no measured target row found")
    return BudgetContext(
        baseline=baseline,
        best_target=best_target,
        anchor=best_row(rows, args.anchor_system),
        best_fused=best_row(rows, args.fused_system),
        drift=parse_drift_report(args.drift_report),
        motion=parse_motion_report(args.motion_report),
    )


def finite(value: float | None) -> bool:
    return value is not None and math.isfinite(value)


def format_float(value: float | None, precision: int = 4) -> str:
    if not finite(value):
        return "TBD"
    return f"{value:.{precision}f}"


def format_percent(value: float | None, precision: int = 1) -> str:
    if not finite(value):
        return "TBD"
    return f"{value:.{precision}f}%"


def rmse_gap_m(target: gap_report.BenchmarkRow, baseline: gap_report.BenchmarkRow) -> float:
    assert target.rmse_m is not None
    assert baseline.rmse_m is not None
    return max(0.0, target.rmse_m - baseline.rmse_m)


def reduction_to_tie_percent(target: gap_report.BenchmarkRow, baseline: gap_report.BenchmarkRow) -> float:
    assert target.rmse_m is not None
    assert baseline.rmse_m is not None
    if target.rmse_m == 0.0:
        return 0.0
    return max(0.0, (1.0 - baseline.rmse_m / target.rmse_m) * 100.0)


def budget_rows(context: BudgetContext) -> list[tuple[str, str, float | None, str]]:
    rows: list[tuple[str, str, float | None, str]] = []
    baseline = context.baseline
    target = context.best_target
    rows.append((
        "Best target gap",
        f"{target.system} {format_float(target.rmse_m)} m vs AQUA-SLAM {format_float(baseline.rmse_m)} m",
        rmse_gap_m(target, baseline),
        "Reduce this residual before any accuracy win claim.",
    ))
    if context.anchor is not None and context.anchor.rmse_m is not None:
        rows.append((
            "Anchor improvement already banked",
            f"{context.anchor.system} {format_float(context.anchor.rmse_m)} m -> {target.system} {format_float(target.rmse_m)} m",
            max(0.0, context.anchor.rmse_m - target.rmse_m),
            "Keep this as the measured progress floor; do not regress below it.",
        ))
    if (
        context.best_fused is not None
        and context.best_fused.rmse_m is not None
        and target.rmse_m is not None
        and context.best_fused.rmse_m > target.rmse_m
    ):
        rows.append((
            "Fusion regression budget",
            f"{context.best_fused.system} is {format_float(context.best_fused.rmse_m - target.rmse_m)} m worse than standalone visual",
            context.best_fused.rmse_m - target.rmse_m,
            "Tune visual covariance, timing, and coupling before claiming fused-system progress.",
        ))
    if finite(context.drift.se3_rmse_m) and finite(context.drift.sim3_rmse_m):
        rows.append((
            "Scale/extrinsic removable component",
            f"Drift report SE(3) {format_float(context.drift.se3_rmse_m)} m, Sim(3) {format_float(context.drift.sim3_rmse_m)} m",
            max(0.0, context.drift.se3_rmse_m - context.drift.sim3_rmse_m),
            "If large, prioritize held-out scale and camera-to-base calibration.",
        ))
        rows.append((
            "Post-Sim(3) drift floor",
            f"Sim(3) floor {format_float(context.drift.sim3_rmse_m)} m vs AQUA-SLAM {format_float(baseline.rmse_m)} m",
            max(0.0, context.drift.sim3_rmse_m - baseline.rmse_m),
            "If this dominates, threshold tuning is not enough; improve tracking geometry or add loop/VI coupling.",
        ))
    if finite(context.drift.worst_window_se3_rmse_m):
        rows.append((
            "Worst drift window",
            f"Worst sliding-window SE(3) RMSE {format_float(context.drift.worst_window_se3_rmse_m)} m",
            context.drift.worst_window_se3_rmse_m,
            "Inspect this time window for outlier bursts, poor stereo geometry, or frame bias.",
        ))
    if finite(context.motion.median_correction_scale):
        deviation = abs(1.0 - context.motion.median_correction_scale)
        rows.append((
            "Motion scale bias",
            f"Median reference/visual correction scale {format_float(context.motion.median_correction_scale, 6)}",
            deviation,
            "Use direction buckets before applying one fixed scale across sequences.",
        ))
    return rows


def format_report(context: BudgetContext, source_paths: list[Path]) -> str:
    baseline = context.baseline
    target = context.best_target
    gap = rmse_gap_m(target, baseline)
    gap_x = target.rmse_m / baseline.rmse_m
    lines = [
        "# AQUA-SLAM Error Budget",
        "",
        f"- Sources: {', '.join(f'`{path}`' for path in source_paths)}",
        f"- Baseline: `{baseline.system}` `{baseline.sequence}` {format_float(baseline.rmse_m)} m RMSE",
        f"- Best target: `{target.system}` `{target.sequence}` {format_float(target.rmse_m)} m RMSE",
        f"- Gap to tie: {format_float(gap)} m ({format_float(gap_x, 2)}x baseline, {format_percent(reduction_to_tie_percent(target, baseline))} reduction needed)",
        "",
        "## Budget",
        "",
        "| Bucket | Evidence | Effect m | Next action |",
        "|--------|----------|---------:|-------------|",
    ]
    for bucket, evidence, effect, action in budget_rows(context):
        lines.append(
            f"| {bucket} | {evidence} | {format_float(effect)} | {action} |"
        )
    lines.extend([
        "",
        "## Development Readout",
        "",
    ])
    if context.best_fused is not None and context.best_fused.rmse_m is not None:
        if context.best_fused.rmse_m > target.rmse_m:
            lines.append(
                "- Standalone visual is currently the accuracy leader; fused visual+DVL is still a regression relative to that row."
            )
        else:
            lines.append("- Fused visual+DVL is the current accuracy leader.")
    if finite(context.drift.sim3_rmse_m):
        if context.drift.sim3_rmse_m > baseline.rmse_m * 2.0:
            lines.append(
                "- Sim(3) still leaves more than 2x the AQUA-SLAM RMSE; focus on drift/geometry, not only scale."
            )
        else:
            lines.append("- Sim(3) gets close to the AQUA-SLAM anchor; scale/extrinsic validation is the next bottleneck.")
    else:
        lines.append(
            "- Add `--drift-report` and `--motion-report` from the best visual run to split scale, extrinsic, and drift terms."
        )
    lines.append("")
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("markdown", nargs="+", type=Path, help="Tank benchmark markdown files.")
    parser.add_argument("--baseline-system", default="AQUA-SLAM")
    parser.add_argument("--anchor-system", default="aqua_localization")
    parser.add_argument("--fused-system", default="aqua_localization+visual")
    parser.add_argument(
        "--target-prefix",
        action="append",
        default=["aqua_"],
        help="System prefix to consider as a target. May be repeated.",
    )
    parser.add_argument("--drift-report", type=Path, help="Optional visual drift Markdown report.")
    parser.add_argument("--motion-report", type=Path, help="Optional visual motion-segment Markdown report.")
    parser.add_argument("--out", type=Path, help="Optional Markdown output path.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        context = load_context(args)
    except (OSError, ValueError) as exc:
        print(f"failed to build AQUA-SLAM error budget: {exc}", file=sys.stderr)
        return 2
    text = format_report(context, args.markdown)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"wrote AQUA-SLAM error budget to {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
