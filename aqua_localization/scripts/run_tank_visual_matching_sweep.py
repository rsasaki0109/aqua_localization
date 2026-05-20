#!/usr/bin/env python3
"""Run Tank visual matching threshold sweeps.

This is a thin orchestration layer around ``run_tank_visual_benchmark.py``. It
replays the same Tank stereo bag with several ORB descriptor-distance gates and
collects the trajectory/status metrics into one Markdown table.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import subprocess
import sys

import compare_trajectories
import run_tank_visual_benchmark
import summarize_visual_frontend_status


DEFAULT_DISTANCE_SET = "64,80,96,112,0"


@dataclass(frozen=True)
class SweepCase:
    stereo_distance: float
    temporal_distance: float

    @property
    def label(self) -> str:
        return (
            f"stereo_{format_distance_label(self.stereo_distance)}__"
            f"temporal_{format_distance_label(self.temporal_distance)}"
        )


@dataclass(frozen=True)
class SweepResult:
    case: SweepCase
    sequence: str
    out_dir: Path
    estimate_tum: Path
    status_csv: Path
    returncode: int
    rmse_m: float
    matched_seconds: float
    accepted_ratio: float
    median_pnp_inliers: float
    median_temporal_matches: float
    command: list[str]


def parse_distance(value: str) -> float:
    text = value.strip().lower()
    if text in {"disabled", "disable", "off", "none", "all"}:
        return 0.0
    distance = float(text)
    if distance < 0.0:
        raise ValueError("descriptor distance thresholds must be non-negative")
    return distance


def parse_distance_list(value: str) -> list[float]:
    distances = [parse_distance(part) for part in value.split(",") if part.strip()]
    if not distances:
        raise ValueError("distance list is empty")
    return distances


def parse_pair(value: str) -> SweepCase:
    if ":" in value:
        left, right = value.split(":", 1)
    elif "/" in value:
        left, right = value.split("/", 1)
    else:
        left = right = value
    return SweepCase(parse_distance(left), parse_distance(right))


def parse_pairs(value: str) -> list[SweepCase]:
    cases = [parse_pair(part) for part in value.split(",") if part.strip()]
    if not cases:
        raise ValueError("pair list is empty")
    return cases


def build_cases(args) -> list[SweepCase]:
    if args.pairs:
        return dedupe_cases(parse_pairs(args.pairs))

    stereo = parse_distance_list(args.stereo_distances)
    temporal = parse_distance_list(args.temporal_distances)
    if args.matrix:
        return dedupe_cases(SweepCase(s, t) for s in stereo for t in temporal)
    return dedupe_cases(SweepCase(s, t) for s, t in zip(stereo, temporal))


def dedupe_cases(cases) -> list[SweepCase]:
    seen = set()
    unique = []
    for case in cases:
        key = (case.stereo_distance, case.temporal_distance)
        if key in seen:
            continue
        seen.add(key)
        unique.append(case)
    return unique


def format_distance(value: float) -> str:
    return "disabled" if value <= 0.0 else f"{value:g}"


def format_distance_label(value: float) -> str:
    return "disabled" if value <= 0.0 else f"{value:g}".replace(".", "p")


def sanitize_sequence(sequence: str, case: SweepCase) -> str:
    return run_tank_visual_benchmark.sanitize_name(f"{sequence}_{case.label}")


def case_odom_topic(case: SweepCase) -> str:
    return f"/aqua_visual_frontend/{case.label}/odometry"


def benchmark_command(args, case: SweepCase, sequence: str, out_dir: Path) -> list[str]:
    command = [
        "ros2",
        "run",
        "aqua_localization",
        "run_tank_visual_benchmark.py",
        "--bag",
        str(args.bag),
        "--reference",
        str(args.reference),
        "--out-dir",
        str(out_dir),
        "--sequence",
        sequence,
        "--dataset",
        args.dataset,
        "--system",
        args.system,
        "--odom-topic",
        case_odom_topic(case),
        "--translation-scale",
        str(args.translation_scale),
        "--max-stereo-descriptor-distance",
        str(case.stereo_distance),
        "--max-temporal-descriptor-distance",
        str(case.temporal_distance),
        "--play-rate",
        str(args.play_rate),
        "--startup-delay",
        str(args.startup_delay),
        "--stop-timeout",
        str(args.stop_timeout),
    ]
    if not args.use_sim_time:
        command.append("--no-sim-time")
    return command


def load_status_metrics(status_csv: Path) -> tuple[float, float, float]:
    if not status_csv.exists():
        return math.nan, math.nan, math.nan
    summary = summarize_visual_frontend_status.summarize(
        summarize_visual_frontend_status.read_csv(status_csv)
    )
    moving = summary["moving_numeric"]
    return (
        float(summary["acceptance_ratio"]),
        float(moving["pnp_inliers"]["median"]),
        float(moving["temporal_matches"]["median"]),
    )


def evaluate_result(args, case: SweepCase, sequence: str, out_dir: Path, command: list[str], returncode: int):
    paths = run_tank_visual_benchmark.default_paths(out_dir, sequence)
    rmse_m = math.nan
    matched_seconds = math.nan
    if paths.estimate_tum.exists():
        stats, _ = compare_trajectories.compare(
            args.reference, paths.estimate_tum, with_scale=False, no_align=False
        )
        rmse_m = float(stats["rmse"])
        matched_seconds = float(stats["matched_seconds"])
    accepted_ratio, median_pnp_inliers, median_temporal_matches = load_status_metrics(paths.status_csv)
    return SweepResult(
        case=case,
        sequence=sequence,
        out_dir=out_dir,
        estimate_tum=paths.estimate_tum,
        status_csv=paths.status_csv,
        returncode=returncode,
        rmse_m=rmse_m,
        matched_seconds=matched_seconds,
        accepted_ratio=accepted_ratio,
        median_pnp_inliers=median_pnp_inliers,
        median_temporal_matches=median_temporal_matches,
        command=command,
    )


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def format_percent(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{100.0 * value:.1f}%"


def gap_ratio(rmse_m: float, baseline_rmse_m: float) -> float:
    if not math.isfinite(rmse_m) or not math.isfinite(baseline_rmse_m):
        return math.nan
    if baseline_rmse_m <= 0.0:
        return math.inf
    return rmse_m / baseline_rmse_m


def improvement_to_tie_percent(rmse_m: float, baseline_rmse_m: float) -> float:
    if not math.isfinite(rmse_m) or not math.isfinite(baseline_rmse_m):
        return math.nan
    if rmse_m <= 0.0:
        return 0.0
    return max(0.0, (1.0 - baseline_rmse_m / rmse_m) * 100.0)


def has_baseline(args) -> bool:
    return math.isfinite(float(getattr(args, "baseline_rmse_m", math.nan)))


def format_markdown(results: list[SweepResult], args) -> str:
    valid = [result for result in results if math.isfinite(result.rmse_m)]
    best = min(valid, key=lambda result: result.rmse_m) if valid else None
    include_baseline = has_baseline(args)
    header = [
        "Stereo dist",
        "Temporal dist",
        "Status",
        "RMSE m",
    ]
    separator = [
        "-------------",
        "---------------",
        "--------",
        "-------:",
    ]
    if include_baseline:
        header.extend(["Gap x", "Improvement to tie"])
        separator.extend(["------:", "-------------------:"])
    header.extend([
        "Matched s",
        "Accepted",
        "Median PnP inliers",
        "Median temporal matches",
        "Output",
    ])
    separator.extend([
        "----------:",
        "---------:",
        "-------------------:",
        "------------------------:",
        "--------",
    ])
    lines = [
        "# Tank Visual Matching Sweep",
        "",
        f"Sequence: `{args.sequence}`",
        f"Reference: `{args.reference}`",
        f"Translation scale: `{args.translation_scale:g}`",
    ]
    if include_baseline:
        lines.append(f"Baseline RMSE: `{args.baseline_rmse_m:g}` m")
    lines.extend([
        "",
        "| " + " | ".join(header) + " |",
        "| " + " | ".join(separator) + " |",
    ])
    for result in results:
        marker = "best" if best is not None and result is best else "ok"
        if result.returncode != 0:
            marker = f"failed ({result.returncode})"
        cells = [
            format_distance(result.case.stereo_distance),
            format_distance(result.case.temporal_distance),
            marker,
            format_float(result.rmse_m),
        ]
        if include_baseline:
            cells.extend([
                format_float(gap_ratio(result.rmse_m, args.baseline_rmse_m), precision=2),
                f"{format_float(improvement_to_tie_percent(result.rmse_m, args.baseline_rmse_m), precision=1)}%",
            ])
        cells.extend([
            format_float(result.matched_seconds, precision=2),
            format_percent(result.accepted_ratio),
            format_float(result.median_pnp_inliers, precision=1),
            format_float(result.median_temporal_matches, precision=1),
            f"`{result.out_dir}`",
        ])
        lines.append(
            "| "
            + " | ".join(cells)
            + " |"
        )

    lines.extend(["", "## Commands", ""])
    for result in results:
        lines.append(f"### {result.sequence}")
        lines.append("")
        lines.append("```bash")
        lines.append(run_tank_visual_benchmark.shell_join(result.command))
        lines.append("```")
        lines.append("")
    if best is not None:
        lines.extend(
            [
                "## Readout",
                "",
                (
                    f"Best RMSE in this sweep: `{format_float(best.rmse_m)}` m with "
                    f"`matching.max_stereo_descriptor_distance={format_distance(best.case.stereo_distance)}` "
                    f"and `matching.max_temporal_descriptor_distance="
                    f"{format_distance(best.case.temporal_distance)}`."
                ),
            ]
        )
        if include_baseline:
            lines.extend(
                [
                    (
                        f"Best gap to baseline: "
                        f"`{format_float(gap_ratio(best.rmse_m, args.baseline_rmse_m), precision=2)}x`; "
                        f"RMSE reduction still needed to tie: "
                        f"`{format_float(improvement_to_tie_percent(best.rmse_m, args.baseline_rmse_m), precision=1)}%`."
                    ),
                ]
            )
    return "\n".join(lines)


def run_sweep(args) -> list[SweepResult]:
    results = []
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for case in build_cases(args):
        sequence = sanitize_sequence(args.sequence, case)
        out_dir = args.out_dir / case.label
        command = benchmark_command(args, case, sequence, out_dir)
        returncode = 0
        if not args.dry_run:
            proc = subprocess.run(command, text=True)
            returncode = int(proc.returncode)
            if returncode != 0 and args.stop_on_failure:
                results.append(evaluate_result(args, case, sequence, out_dir, command, returncode))
                break
        results.append(evaluate_result(args, case, sequence, out_dir, command, returncode))
    return results


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Sweep Tank visual ORB matching thresholds and write a Markdown summary."
    )
    parser.add_argument("--bag", required=True, type=Path, help="ROS 2 Tank bag with stereo images.")
    parser.add_argument("--reference", required=True, type=Path, help="Reference TUM trajectory.")
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_visual_matching_sweep"))
    parser.add_argument("--summary-out", type=Path, default=None)
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--sequence", default="short_test")
    parser.add_argument("--system", default="aqua_visual_frontend")
    parser.add_argument("--translation-scale", type=float, default=1.0)
    parser.add_argument(
        "--baseline-rmse-m",
        type=float,
        default=math.nan,
        help="Optional baseline RMSE for gap-to-baseline columns, e.g. AQUA-SLAM 0.0194.",
    )
    parser.add_argument("--stereo-distances", default=DEFAULT_DISTANCE_SET)
    parser.add_argument("--temporal-distances", default=DEFAULT_DISTANCE_SET)
    parser.add_argument(
        "--pairs",
        default="",
        help="Comma-separated stereo:temporal pairs. Overrides --*-distances.",
    )
    parser.add_argument("--matrix", action="store_true", help="Run full Cartesian threshold matrix.")
    parser.add_argument("--play-rate", type=float, default=1.0)
    parser.add_argument("--startup-delay", type=float, default=1.0)
    parser.add_argument("--stop-timeout", type=float, default=5.0)
    parser.add_argument("--no-sim-time", dest="use_sim_time", action="store_false")
    parser.add_argument("--dry-run", action="store_true", help="Only write planned commands.")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.set_defaults(use_sim_time=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.translation_scale <= 0.0:
        raise ValueError("--translation-scale must be positive")
    if math.isfinite(args.baseline_rmse_m) and args.baseline_rmse_m <= 0.0:
        raise ValueError("--baseline-rmse-m must be positive")
    if args.play_rate <= 0.0:
        raise ValueError("--play-rate must be positive")

    results = run_sweep(args)
    summary = format_markdown(results, args)
    summary_out = args.summary_out or (args.out_dir / "visual_matching_sweep.md")
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    summary_out.write_text(summary + "\n", encoding="utf-8")
    print(f"wrote sweep summary: {summary_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
