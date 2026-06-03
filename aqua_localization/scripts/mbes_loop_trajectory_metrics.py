#!/usr/bin/env python3
"""Export and compare MBES loop-closure pose-graph trajectory metrics."""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys


def load_script_module(name: str, script_name: str):
    script_path = Path(__file__).resolve().parent / script_name
    spec = importlib.util.spec_from_file_location(name, script_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def default_out_dir(out: Path | None, out_dir: Path | None) -> Path:
    if out_dir is not None:
        return out_dir
    if out is not None:
        return out.parent / "mbes_loop_trajectory_metrics"
    return Path("/tmp/aqua_mbes_loop_trajectory_metrics")


def format_float(value: float) -> str:
    return f"{value:.4f}"


def metric_row(label: str, stats: dict) -> str:
    return (
        f"| {label} | {stats['count']} | {stats['matched_seconds']:.2f} | "
        f"{format_float(stats['mean'])} | {format_float(stats['median'])} | "
        f"{format_float(stats['rmse'])} | {format_float(stats['max'])} |"
    )


def improvement_value(baseline_stats: dict, pose_graph_stats: dict, key: str) -> float:
    return float(baseline_stats[key]) - float(pose_graph_stats[key])


def improvement_label(value: float) -> str:
    if value > 0.0:
        return f"improved by {format_float(value)} m"
    if value < 0.0:
        return f"worse by {format_float(abs(value))} m"
    return "unchanged"


def alignment_label(with_scale: bool, no_align: bool) -> str:
    if no_align:
        return "raw"
    return "Sim(3)" if with_scale else "SE(3)"


def format_report(
    args: argparse.Namespace,
    paths: dict[str, Path],
    counts: dict[str, int],
    baseline_stats: dict,
    pose_graph_stats: dict,
) -> str:
    rmse_delta = improvement_value(baseline_stats, pose_graph_stats, "rmse")
    median_delta = improvement_value(baseline_stats, pose_graph_stats, "median")
    lines = [
        "# MBES Loop Closure Trajectory Metrics",
        "",
        f"- Dataset: {args.dataset}",
        f"- Sequence: {args.sequence}",
        f"- Bag: `{args.bag}`",
        f"- Reference topic: `{args.reference_topic}`",
        f"- Input odometry topic: `{args.input_odometry_topic}`",
        f"- Pose graph path topic: `{args.pose_graph_path_topic}`",
        f"- Alignment: {alignment_label(args.scale, args.no_align)}",
        "",
        "## Exported Trajectories",
        "",
        "| Trajectory | TUM path | Samples |",
        "|------------|----------|--------:|",
        f"| Reference | `{paths['reference']}` | {counts['reference']} |",
        f"| Input odometry | `{paths['input_odometry']}` | {counts['input_odometry']} |",
        f"| Pose graph path | `{paths['pose_graph']}` | {counts['pose_graph']} |",
        "",
        "## APE Translation",
        "",
        "| Estimate | Matched samples | Matched s | Mean m | Median m | RMSE m | Max m |",
        "|----------|----------------:|----------:|-------:|---------:|-------:|------:|",
        metric_row("Input odometry", baseline_stats),
        metric_row("Pose graph path", pose_graph_stats),
        "",
        "## Loop Impact",
        "",
        f"- RMSE delta: {improvement_label(rmse_delta)}",
        f"- Median delta: {improvement_label(median_delta)}",
        "",
        "Positive deltas mean the pose graph path is closer to the dataset "
        "reference than the input odometry under the selected alignment. This "
        "report is still only a trajectory metric; accepted-loop false-positive "
        "audit notes must be attached before using it as a loop-closure claim.",
        "",
    ]
    return "\n".join(lines)


def evaluate(args: argparse.Namespace) -> tuple[str, dict[str, Path]]:
    out_dir = default_out_dir(args.out, args.out_dir)
    paths = {
        "reference": out_dir / "reference_odometry.tum",
        "input_odometry": out_dir / "input_odometry.tum",
        "pose_graph": out_dir / "pose_graph_path.tum",
    }
    odom_export = load_script_module("export_rosbag_odometry_tum", "export_rosbag_odometry_tum.py")
    path_export = load_script_module("export_rosbag_path_tum", "export_rosbag_path_tum.py")
    compare = load_script_module("compare_trajectories", "compare_trajectories.py")

    counts = {
        "reference": odom_export.export_odometry_topic(
            args.bag,
            args.reference_topic,
            paths["reference"],
        ),
        "input_odometry": odom_export.export_odometry_topic(
            args.bag,
            args.input_odometry_topic,
            paths["input_odometry"],
        ),
        "pose_graph": path_export.export_path_topic(
            args.bag,
            args.pose_graph_path_topic,
            paths["pose_graph"],
        ),
    }
    baseline_stats, _ = compare.compare(
        paths["reference"],
        paths["input_odometry"],
        with_scale=args.scale,
        no_align=args.no_align,
    )
    pose_graph_stats, _ = compare.compare(
        paths["reference"],
        paths["pose_graph"],
        with_scale=args.scale,
        no_align=args.no_align,
    )
    report = format_report(args, paths, counts, baseline_stats, pose_graph_stats)
    return report, paths


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="Results-included MBES replay bag")
    parser.add_argument("--out", type=Path, help="Optional markdown report path")
    parser.add_argument("--out-dir", type=Path, help="Directory for exported TUM trajectories")
    parser.add_argument("--dataset", default="MBES-SLAM")
    parser.add_argument("--sequence", default="beach_pond")
    parser.add_argument("--reference-topic", default="/nav/processed/odometry")
    parser.add_argument("--input-odometry-topic", default="/aqua_imu_loc/odometry")
    parser.add_argument("--pose-graph-path-topic", default="/aqua_pose_graph/path")
    parser.add_argument("--scale", action="store_true", help="Use Sim(3) alignment")
    parser.add_argument("--no-align", action="store_true", help="Compare raw positions")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        report, _paths = evaluate(args)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"failed to compute MBES loop trajectory metrics: {exc}", file=sys.stderr)
        return 1
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
        print(f"wrote MBES loop trajectory metrics to {args.out}")
    else:
        print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
