#!/usr/bin/env python3
"""Summarize MBES candidate descriptor-weight benchmark sweeps."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import sys


STATUS_CSV_NAME = "mbes_beach_pond_loop_status.csv"
TRAJECTORY_METRICS_NAME = "mbes_beach_pond_loop_trajectory_metrics.md"


@dataclass(frozen=True)
class CaseSpec:
    weight: float
    out_dir: Path

    @property
    def status_csv(self) -> Path:
        return self.out_dir / STATUS_CSV_NAME

    @property
    def metrics_report(self) -> Path:
        return self.out_dir / TRAJECTORY_METRICS_NAME


@dataclass(frozen=True)
class CaseResult:
    spec: CaseSpec
    input_rmse_m: float
    input_matched_s: float
    pose_graph_rmse_m: float
    pose_graph_median_m: float
    pose_graph_matched_s: float
    samples: int
    accepted: int
    rejected: int
    no_candidate: int
    converged: int
    median_fitness: float
    p95_correction_m: float


def load_script_module(name: str):
    script_path = Path(__file__).resolve().parent / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, script_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def parse_case(text: str) -> CaseSpec:
    if ":" not in text:
        raise ValueError(f"case must use WEIGHT:OUT_DIR format: {text}")
    weight_text, out_dir_text = text.split(":", 1)
    if not weight_text.strip() or not out_dir_text.strip():
        raise ValueError(f"case must use WEIGHT:OUT_DIR format: {text}")
    try:
        weight = float(weight_text)
    except ValueError as exc:
        raise ValueError(f"invalid descriptor weight in case: {text}") from exc
    if not math.isfinite(weight) or weight < 0.0:
        raise ValueError(f"descriptor weight must be finite and non-negative: {text}")
    return CaseSpec(weight=weight, out_dir=Path(out_dir_text))


def format_float(value: float, digits: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def format_count(value: int) -> str:
    return str(int(value))


def format_delta(delta_m: float) -> str:
    if not math.isfinite(delta_m):
        return "n/a"
    if delta_m > 0.0:
        return f"+{delta_m:.4f}"
    if delta_m < 0.0:
        return f"-{abs(delta_m):.4f}"
    return "0.0000"


def load_case(spec: CaseSpec) -> CaseResult:
    benchmark = load_script_module("mbes_loop_benchmark_row")
    compare = load_script_module("compare_mbes_loop_metric_reports")
    rows = benchmark.read_loop_status_csv(spec.status_csv)
    summary = benchmark.summarize_rows(rows)
    metrics = compare.parse_metric_report(spec.metrics_report)
    input_metric = metrics["Input odometry"]
    graph_metric = metrics["Pose graph path"]
    return CaseResult(
        spec=spec,
        input_rmse_m=input_metric.rmse_m,
        input_matched_s=input_metric.matched_s,
        pose_graph_rmse_m=graph_metric.rmse_m,
        pose_graph_median_m=graph_metric.median_m,
        pose_graph_matched_s=graph_metric.matched_s,
        samples=int(summary["samples"]),
        accepted=int(summary["accepted"]),
        rejected=int(summary["rejected"]),
        no_candidate=int(summary["no_candidate"]),
        converged=int(summary["converged"]),
        median_fitness=float(summary["median_fitness"]),
        p95_correction_m=float(summary["p95_correction_m"]),
    )


def baseline_result(results: list[CaseResult]) -> CaseResult | None:
    if not results:
        return None
    for result in results:
        if abs(result.spec.weight) < 1.0e-12:
            return result
    return results[0]


def best_result(results: list[CaseResult]) -> CaseResult | None:
    finite = [result for result in results if math.isfinite(result.pose_graph_rmse_m)]
    if not finite:
        return None
    return min(finite, key=lambda result: result.pose_graph_rmse_m)


def coverage_warning(result: CaseResult, baseline: CaseResult | None) -> bool:
    if baseline is None:
        return False
    input_drift = abs(result.input_rmse_m - baseline.input_rmse_m)
    input_coverage_drift = abs(result.input_matched_s - baseline.input_matched_s)
    graph_coverage_drift = abs(result.pose_graph_matched_s - baseline.pose_graph_matched_s)
    return (
        input_drift > 1.0
        or input_coverage_drift > 5.0
        or graph_coverage_drift > 5.0
    )


def status_label(
    result: CaseResult,
    baseline: CaseResult | None,
    best: CaseResult | None,
) -> str:
    labels = []
    if baseline is not None and result is baseline:
        labels.append("baseline")
    if best is not None and result is best:
        labels.append("best")
    if coverage_warning(result, baseline):
        labels.append("check coverage")
    if not labels:
        labels.append("ok")
    return ", ".join(labels)


def format_markdown(
    results: list[CaseResult],
    args: argparse.Namespace,
) -> str:
    ordered = sorted(results, key=lambda result: result.spec.weight)
    baseline = baseline_result(ordered)
    best = best_result(ordered)
    lines = [
        "# MBES Candidate Descriptor Weight Sweep",
        "",
        f"- Dataset: {args.dataset}",
        f"- Sequence: {args.sequence}",
        f"- Cases: {len(ordered)}",
        f"- Descriptor scales: centroid={format_float(args.descriptor_centroid_scale_m, 3)} m, "
        f"extent={format_float(args.descriptor_extent_scale, 3)}, "
        f"point_ratio={format_float(args.descriptor_point_ratio_scale, 3)}",
        "",
    ]
    if baseline is None:
        lines.append("No cases were available.")
        lines.append("")
        return "\n".join(lines)
    if abs(baseline.spec.weight) >= 1.0e-12:
        lines.extend([
            "WARNING: no `0.0` descriptor-weight baseline was provided; the first "
            "case is used as the comparison baseline.",
            "",
        ])
    lines.extend([
        "Positive RMSE deltas mean the candidate-weight case reduced pose-graph "
        "RMSE versus the baseline replay. Treat rows marked `check coverage` as "
        "diagnostic until input odometry and matched-time coverage are aligned.",
        "",
        "| Weight | Status | Input RMSE m | Pose graph RMSE m | Graph vs input m | Graph vs baseline m | Graph median m | Matched s | Accepted | Rejected | No candidate | Median fitness | P95 correction m | Output |",
        "|-------:|--------|-------------:|------------------:|-----------------:|--------------------:|---------------:|----------:|---------:|---------:|-------------:|---------------:|-----------------:|--------|",
    ])
    for result in ordered:
        graph_vs_input = result.input_rmse_m - result.pose_graph_rmse_m
        graph_vs_baseline = baseline.pose_graph_rmse_m - result.pose_graph_rmse_m
        lines.append(
            "| "
            + " | ".join([
                format_float(result.spec.weight, 4),
                status_label(result, baseline, best),
                format_float(result.input_rmse_m),
                format_float(result.pose_graph_rmse_m),
                format_delta(graph_vs_input),
                format_delta(graph_vs_baseline),
                format_float(result.pose_graph_median_m),
                format_float(result.pose_graph_matched_s, 2),
                format_count(result.accepted),
                format_count(result.rejected),
                format_count(result.no_candidate),
                format_float(result.median_fitness),
                format_float(result.p95_correction_m),
                f"`{result.spec.out_dir}`",
            ])
            + " |"
        )
    if best is not None:
        lines.extend([
            "",
            "## Readout",
            "",
            (
                f"Best pose-graph RMSE: `{format_float(best.pose_graph_rmse_m)}` m "
                f"at `candidates.descriptor_weight={format_float(best.spec.weight, 4)}` "
                f"({format_delta(baseline.pose_graph_rmse_m - best.pose_graph_rmse_m)} m "
                "versus baseline)."
            ),
        ])
    lines.append("")
    return "\n".join(lines)


def write_csv(path: Path, results: list[CaseResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "descriptor_weight",
        "input_rmse_m",
        "input_matched_s",
        "pose_graph_rmse_m",
        "pose_graph_median_m",
        "pose_graph_matched_s",
        "samples",
        "accepted",
        "rejected",
        "no_candidate",
        "converged",
        "median_fitness",
        "p95_correction_m",
        "out_dir",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for result in sorted(results, key=lambda item: item.spec.weight):
            writer.writerow({
                "descriptor_weight": result.spec.weight,
                "input_rmse_m": result.input_rmse_m,
                "input_matched_s": result.input_matched_s,
                "pose_graph_rmse_m": result.pose_graph_rmse_m,
                "pose_graph_median_m": result.pose_graph_median_m,
                "pose_graph_matched_s": result.pose_graph_matched_s,
                "samples": result.samples,
                "accepted": result.accepted,
                "rejected": result.rejected,
                "no_candidate": result.no_candidate,
                "converged": result.converged,
                "median_fitness": result.median_fitness,
                "p95_correction_m": result.p95_correction_m,
                "out_dir": str(result.spec.out_dir),
            })


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case",
        action="append",
        default=[],
        help="Case in WEIGHT:OUT_DIR format. May be repeated.",
    )
    parser.add_argument("--out", type=Path, help="Optional Markdown output path")
    parser.add_argument("--csv-out", type=Path, help="Optional CSV output path")
    parser.add_argument("--dataset", default="MBES-SLAM")
    parser.add_argument("--sequence", default="beach_pond")
    parser.add_argument("--descriptor-centroid-scale-m", type=float, default=0.6)
    parser.add_argument("--descriptor-extent-scale", type=float, default=0.15)
    parser.add_argument("--descriptor-point-ratio-scale", type=float, default=0.10)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        if min(
            args.descriptor_centroid_scale_m,
            args.descriptor_extent_scale,
            args.descriptor_point_ratio_scale,
        ) <= 0.0:
            raise ValueError("descriptor scales must be positive")
        cases = [parse_case(case) for case in args.case]
        results = [load_case(case) for case in cases]
        text = format_markdown(results, args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"failed to summarize MBES candidate descriptor-weight sweep: {exc}", file=sys.stderr)
        return 1
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
        print(f"wrote MBES candidate descriptor-weight sweep summary to {args.out}")
    else:
        print(text)
    if args.csv_out is not None:
        write_csv(args.csv_out, results)
        print(f"wrote MBES candidate descriptor-weight sweep CSV to {args.csv_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
