#!/usr/bin/env python3
"""Compare normal and selected-loop MBES trajectory metric reports."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys


@dataclass(frozen=True)
class TrajectoryMetric:
    estimate: str
    matched_samples: int
    matched_s: float
    mean_m: float
    median_m: float
    rmse_m: float
    max_m: float


def markdown_cells(line: str) -> list[str]:
    return [cell.strip() for cell in line.strip().strip("|").split("|")]


def parse_metric_report(path: Path) -> dict[str, TrajectoryMetric]:
    metrics: dict[str, TrajectoryMetric] = {}
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            if not line.startswith("|"):
                continue
            cells = markdown_cells(line)
            if len(cells) != 7:
                continue
            if cells[0] not in {"Input odometry", "Pose graph path"}:
                continue
            try:
                metrics[cells[0]] = TrajectoryMetric(
                    estimate=cells[0],
                    matched_samples=int(cells[1]),
                    matched_s=float(cells[2]),
                    mean_m=float(cells[3]),
                    median_m=float(cells[4]),
                    rmse_m=float(cells[5]),
                    max_m=float(cells[6]),
                )
            except ValueError as exc:
                raise ValueError(f"invalid metric row in {path}: {line.strip()}") from exc
    missing = {"Input odometry", "Pose graph path"} - set(metrics)
    if missing:
        raise ValueError(f"{path}: missing metric rows: {', '.join(sorted(missing))}")
    return metrics


def format_delta(delta_m: float) -> str:
    if delta_m > 0.0:
        return f"improved by {delta_m:.4f} m"
    if delta_m < 0.0:
        return f"worse by {abs(delta_m):.4f} m"
    return "unchanged"


def pose_graph_delta(normal: dict[str, TrajectoryMetric],
                     selected: dict[str, TrajectoryMetric],
                     attr: str) -> float:
    return getattr(normal["Pose graph path"], attr) - getattr(selected["Pose graph path"], attr)


def run_loop_delta(metrics: dict[str, TrajectoryMetric], attr: str) -> float:
    return getattr(metrics["Input odometry"], attr) - getattr(metrics["Pose graph path"], attr)


def metric_row(label: str, metrics: dict[str, TrajectoryMetric]) -> str:
    input_metric = metrics["Input odometry"]
    graph_metric = metrics["Pose graph path"]
    graph_delta = input_metric.rmse_m - graph_metric.rmse_m
    return (
        f"| {label} | {input_metric.rmse_m:.4f} | {graph_metric.rmse_m:.4f} | "
        f"{format_delta(graph_delta)} | {graph_metric.median_m:.4f} | "
        f"{graph_metric.matched_samples} | {graph_metric.matched_s:.2f} |"
    )


def format_comparison(
    normal_path: Path,
    selected_path: Path,
    normal: dict[str, TrajectoryMetric],
    selected: dict[str, TrajectoryMetric],
) -> str:
    selected_vs_normal_rmse = pose_graph_delta(normal, selected, "rmse_m")
    selected_vs_normal_median = pose_graph_delta(normal, selected, "median_m")
    input_vs_input_rmse = (
        normal["Input odometry"].rmse_m - selected["Input odometry"].rmse_m)
    input_vs_input_matched_s = (
        normal["Input odometry"].matched_s - selected["Input odometry"].matched_s)
    coverage_delta_s = (
        normal["Pose graph path"].matched_s - selected["Pose graph path"].matched_s)
    baseline_warning = (
        abs(input_vs_input_rmse) > 1.0 or
        abs(input_vs_input_matched_s) > 5.0 or
        abs(coverage_delta_s) > 5.0
    )
    lines = [
        "# MBES Selected-Loop Replay Comparison",
        "",
        f"- Normal metrics: `{normal_path}`",
        f"- Selected-loop metrics: `{selected_path}`",
        "",
        "## APE RMSE",
        "",
        "| Replay | Input odom RMSE m | Pose graph RMSE m | Pose graph vs input | Pose graph median m | Matched samples | Matched s |",
        "|--------|------------------:|------------------:|---------------------|--------------------:|----------------:|----------:|",
        metric_row("normal", normal),
        metric_row("selected-loop", selected),
        "",
        "## Selected vs Normal Pose Graph",
        "",
        f"- Pose graph RMSE: {format_delta(selected_vs_normal_rmse)}",
        f"- Pose graph median: {format_delta(selected_vs_normal_median)}",
        "",
        "## Replay Baseline Drift",
        "",
        f"- Input odometry RMSE: {format_delta(input_vs_input_rmse)}",
        f"- Input odometry matched seconds delta: {input_vs_input_matched_s:.2f} s",
        f"- Pose graph matched seconds delta: {coverage_delta_s:.2f} s",
    ]
    if baseline_warning:
        lines.extend([
            "",
            "WARNING: normal and selected replays do not share a close input "
            "odometry baseline or matched-time coverage. Treat selected-vs-normal "
            "pose graph deltas as diagnostic until replay determinism and coverage "
            "are aligned.",
        ])
    lines.extend([
        "",
        "Use this only with the matching loop-status, batch-consistency, and "
        "accepted-loop audit artifacts from the same replay pair. A lower "
        "selected-loop RMSE is useful evidence, but not a validated claim until "
        "accepted loops have false-positive review notes.",
        "",
    ])
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--normal", required=True, type=Path,
                        help="Normal replay mbes_loop_trajectory_metrics.md")
    parser.add_argument("--selected", required=True, type=Path,
                        help="Selected-loop replay mbes_loop_trajectory_metrics.md")
    parser.add_argument("--out", type=Path, help="Optional Markdown output path")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        normal = parse_metric_report(args.normal)
        selected = parse_metric_report(args.selected)
        text = format_comparison(args.normal, args.selected, normal, selected)
    except (OSError, ValueError) as exc:
        print(f"failed to compare MBES loop metric reports: {exc}", file=sys.stderr)
        return 1
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
        print(f"wrote MBES selected-loop comparison to {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
