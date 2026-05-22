#!/usr/bin/env python3
"""Sweep Tank visual fusion covariance and timing knobs."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import subprocess
import sys

import compare_trajectories
import run_tank_visual_benchmark
import run_tank_visual_fusion_benchmark


DEFAULT_VARIANCE_FLOORS = "0.0025,0.005,0.01,0.02,0.04,0.08"
DEFAULT_MAX_AGE_S = "0.1,0.25,0.5,1.0"


@dataclass(frozen=True)
class SweepCase:
    variance_floor: float
    max_age_s: float

    @property
    def label(self) -> str:
        return (
            f"var_{format_label(self.variance_floor)}__"
            f"age_{format_label(self.max_age_s)}"
        )


@dataclass(frozen=True)
class SweepResult:
    case: SweepCase
    sequence: str
    out_dir: Path
    fused_tum: Path
    status_csv: Path
    returncode: int
    rmse_m: float
    matched_seconds: float
    visual_frames: int
    visual_coverage_ratio: float
    command: list[str]


def parse_positive_float(value: str, label: str) -> float:
    parsed = float(value.strip())
    if parsed <= 0.0:
        raise ValueError(f"{label} values must be positive")
    return parsed


def parse_float_list(value: str, label: str) -> list[float]:
    values = [parse_positive_float(part, label) for part in value.split(",") if part.strip()]
    if not values:
        raise ValueError(f"{label} list is empty")
    return values


def parse_pair(value: str) -> SweepCase:
    if ":" in value:
        left, right = value.split(":", 1)
    elif "/" in value:
        left, right = value.split("/", 1)
    else:
        left = right = value
    return SweepCase(
        parse_positive_float(left, "variance floor"),
        parse_positive_float(right, "max age"),
    )


def parse_pairs(value: str) -> list[SweepCase]:
    cases = [parse_pair(part) for part in value.split(",") if part.strip()]
    if not cases:
        raise ValueError("pair list is empty")
    return dedupe_cases(cases)


def build_cases(args) -> list[SweepCase]:
    if args.pairs:
        return parse_pairs(args.pairs)
    variances = parse_float_list(args.variance_floors, "variance floor")
    ages = parse_float_list(args.max_age_s_values, "max age")
    if args.matrix:
        return dedupe_cases(SweepCase(var, age) for var in variances for age in ages)
    return dedupe_cases(SweepCase(var, age) for var, age in zip(variances, ages))


def dedupe_cases(cases) -> list[SweepCase]:
    seen = set()
    unique = []
    for case in cases:
        key = (case.variance_floor, case.max_age_s)
        if key in seen:
            continue
        seen.add(key)
        unique.append(case)
    return unique


def format_label(value: float) -> str:
    return f"{value:g}".replace(".", "p")


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def format_percent_ratio(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{100.0 * value:.1f}%"


def format_ratio(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.2f}x"


def gap_ratio(rmse_m: float, baseline_rmse_m: float) -> float:
    if not math.isfinite(rmse_m) or not math.isfinite(baseline_rmse_m):
        return math.nan
    if baseline_rmse_m <= 0.0:
        return math.inf
    return rmse_m / baseline_rmse_m


def standalone_delta_m(rmse_m: float, standalone_rmse_m: float) -> float:
    if not math.isfinite(rmse_m) or not math.isfinite(standalone_rmse_m):
        return math.nan
    return rmse_m - standalone_rmse_m


def sequence_name(base: str, case: SweepCase) -> str:
    return run_tank_visual_benchmark.sanitize_name(f"{base}_{case.label}")


def case_visual_topic(case: SweepCase) -> str:
    return f"/aqua_visual_frontend/fusion_sweep/{case.label}/odometry"


def case_fused_topic(case: SweepCase) -> str:
    return f"/aqua_imu_loc/fusion_sweep/{case.label}/odometry"


def benchmark_command(args, case: SweepCase, sequence: str, out_dir: Path) -> list[str]:
    command = [
        "ros2",
        "run",
        "aqua_localization",
        "run_tank_visual_fusion_benchmark.py",
        "--bag",
        str(args.bag),
        "--reference",
        str(args.reference),
        "--out-dir",
        str(out_dir),
        "--imu-params",
        str(args.imu_params),
        "--dataset",
        args.dataset,
        "--sequence",
        sequence,
        "--system",
        args.system,
        "--visual-odom-topic",
        case_visual_topic(case),
        "--fused-odom-topic",
        case_fused_topic(case),
        "--translation-scale",
        str(args.translation_scale),
        "--visual-position-variance-floor",
        str(case.variance_floor),
        "--visual-max-age-s",
        str(case.max_age_s),
        "--base-from-camera-x-m",
        str(args.base_from_camera_x_m),
        "--base-from-camera-y-m",
        str(args.base_from_camera_y_m),
        "--base-from-camera-z-m",
        str(args.base_from_camera_z_m),
        "--base-from-camera-roll-rad",
        str(args.base_from_camera_roll_rad),
        "--base-from-camera-pitch-rad",
        str(args.base_from_camera_pitch_rad),
        "--base-from-camera-yaw-rad",
        str(args.base_from_camera_yaw_rad),
        "--max-stereo-descriptor-distance",
        str(args.max_stereo_descriptor_distance),
        "--max-temporal-descriptor-distance",
        str(args.max_temporal_descriptor_distance),
        "--orb-n-features",
        str(args.orb_n_features),
        "--orb-fast-threshold",
        str(args.orb_fast_threshold),
        "--opencv-threads",
        str(args.opencv_threads),
        "--play-rate",
        str(args.play_rate),
        "--expected-visual-frames",
        str(args.expected_visual_frames),
        "--min-visual-coverage",
        str(args.min_visual_coverage),
        "--startup-delay",
        str(args.startup_delay),
        "--post-play-delay",
        str(args.post_play_delay),
        "--stop-timeout",
        str(args.stop_timeout),
    ]
    if args.visual_calibration_profile is not None:
        command.extend(["--visual-calibration-profile", str(args.visual_calibration_profile)])
    if not args.use_sim_time:
        command.append("--no-sim-time")
    return command


def evaluate_result(args, case: SweepCase, sequence: str, out_dir: Path, command: list[str], returncode: int):
    paths = run_tank_visual_fusion_benchmark.default_paths(out_dir, sequence)
    rmse_m = math.nan
    matched_seconds = math.nan
    if paths.fused_tum.exists():
        try:
            stats, _ = compare_trajectories.compare(
                args.reference, paths.fused_tum, with_scale=False, no_align=False
            )
            rmse_m = float(stats["rmse"])
            matched_seconds = float(stats["matched_seconds"])
        except (OSError, ValueError):
            rmse_m = math.nan
            matched_seconds = math.nan
    status_exists = paths.visual_status_csv.exists()
    coverage = run_tank_visual_fusion_benchmark.VisualCoverage(
        processed_frames=(
            run_tank_visual_fusion_benchmark.count_visual_status_rows(paths.visual_status_csv)
            if status_exists else 0
        ),
        expected_frames=(
            args.expected_visual_frames
            if status_exists and args.expected_visual_frames > 0 else None
        ),
        min_coverage=args.min_visual_coverage,
    )
    return SweepResult(
        case=case,
        sequence=sequence,
        out_dir=out_dir,
        fused_tum=paths.fused_tum,
        status_csv=paths.visual_status_csv,
        returncode=returncode,
        rmse_m=rmse_m,
        matched_seconds=matched_seconds,
        visual_frames=coverage.processed_frames,
        visual_coverage_ratio=coverage.ratio if coverage.ratio is not None else math.nan,
        command=command,
    )


def run_sweep(args) -> list[SweepResult]:
    results = []
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for case in build_cases(args):
        sequence = sequence_name(args.sequence, case)
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


def has_baseline(args) -> bool:
    return math.isfinite(float(getattr(args, "baseline_rmse_m", math.nan)))


def has_standalone(args) -> bool:
    return math.isfinite(float(getattr(args, "standalone_visual_rmse_m", math.nan)))


def format_markdown(results: list[SweepResult], args) -> str:
    valid = [result for result in results if math.isfinite(result.rmse_m)]
    best = min(valid, key=lambda result: result.rmse_m) if valid else None
    header = ["Visual variance", "Visual max age s", "Status", "RMSE m"]
    separator = ["--------------:", "-----------------:", "--------", "-------:"]
    if has_baseline(args):
        header.append("Gap to AQUA-SLAM")
        separator.append("-----------------:")
    if has_standalone(args):
        header.append("Delta vs standalone")
        separator.append("-------------------:")
    header.extend(["Matched s", "Visual coverage", "Output"])
    separator.extend(["----------:", "---------------:", "--------"])

    lines = [
        "# Tank Visual Fusion Sweep",
        "",
        f"Sequence: `{args.sequence}`",
        f"Reference: `{args.reference}`",
        f"Translation scale: `{args.translation_scale:g}`",
        f"Visual frontend: ORB {args.orb_n_features} features, FAST {args.orb_fast_threshold}, OpenCV threads {args.opencv_threads}",
    ]
    if has_baseline(args):
        lines.append(f"AQUA-SLAM baseline RMSE: `{args.baseline_rmse_m:g}` m")
    if has_standalone(args):
        lines.append(f"Standalone visual RMSE: `{args.standalone_visual_rmse_m:g}` m")
    lines.extend([
        "",
        "| " + " | ".join(header) + " |",
        "| " + " | ".join(separator) + " |",
    ])
    for result in results:
        status = "best" if best is not None and result is best else "ok"
        if result.returncode != 0:
            status = f"failed ({result.returncode})"
        cells = [
            f"{result.case.variance_floor:g}",
            f"{result.case.max_age_s:g}",
            status,
            format_float(result.rmse_m),
        ]
        if has_baseline(args):
            cells.append(format_ratio(gap_ratio(result.rmse_m, args.baseline_rmse_m)))
        if has_standalone(args):
            cells.append(format_float(standalone_delta_m(result.rmse_m, args.standalone_visual_rmse_m)))
        cells.extend([
            format_float(result.matched_seconds, precision=2),
            format_percent_ratio(result.visual_coverage_ratio),
            f"`{result.out_dir}`",
        ])
        lines.append("| " + " | ".join(cells) + " |")

    lines.extend(["", "## Commands", ""])
    for result in results:
        lines.append(f"### {result.sequence}")
        lines.append("")
        lines.append("```bash")
        lines.append(run_tank_visual_benchmark.shell_join(result.command))
        lines.append("```")
        lines.append("")
    if best is not None:
        lines.extend([
            "## Readout",
            "",
            (
                f"Best fused RMSE in this sweep: `{format_float(best.rmse_m)}` m "
                f"with `imu.visual.position_variance_floor={best.case.variance_floor:g}` "
                f"and `imu.visual.max_age_s={best.case.max_age_s:g}`."
            ),
        ])
        if has_standalone(args):
            delta = standalone_delta_m(best.rmse_m, args.standalone_visual_rmse_m)
            if delta > 0.0:
                lines.append(
                    f"Best fused row is still `{format_float(delta)}` m worse than standalone visual."
                )
            else:
                lines.append(
                    f"Best fused row beats standalone visual by `{format_float(abs(delta))}` m."
                )
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Sweep visual fusion covariance/timing and write a Markdown summary."
    )
    parser.add_argument("--visual-calibration-profile", type=Path, default=None)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_visual_fusion_sweep"))
    parser.add_argument("--summary-out", type=Path, default=None)
    parser.add_argument("--imu-params", type=Path, default=run_tank_visual_fusion_benchmark.DEFAULT_IMU_PARAMS)
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--sequence", default="short_test")
    parser.add_argument("--system", default="aqua_localization+visual")
    parser.add_argument("--translation-scale", type=float, default=1.0)
    parser.add_argument("--base-from-camera-x-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-y-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-z-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-roll-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-pitch-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-yaw-rad", type=float, default=0.0)
    parser.add_argument("--max-stereo-descriptor-distance", type=float, default=64.0)
    parser.add_argument("--max-temporal-descriptor-distance", type=float, default=64.0)
    parser.add_argument("--orb-n-features", type=int, default=700)
    parser.add_argument("--orb-fast-threshold", type=int, default=16)
    parser.add_argument("--opencv-threads", type=int, default=2)
    parser.add_argument("--play-rate", type=float, default=1.0)
    parser.add_argument("--expected-visual-frames", type=int, default=0)
    parser.add_argument("--min-visual-coverage", type=float, default=0.98)
    parser.add_argument("--baseline-rmse-m", type=float, default=math.nan)
    parser.add_argument("--standalone-visual-rmse-m", type=float, default=math.nan)
    parser.add_argument("--variance-floors", default=DEFAULT_VARIANCE_FLOORS)
    parser.add_argument("--max-age-s-values", default=DEFAULT_MAX_AGE_S)
    parser.add_argument(
        "--pairs",
        default="",
        help="Comma-separated variance:max_age pairs. Overrides --variance-floors and --max-age-s-values.",
    )
    parser.add_argument("--matrix", action="store_true")
    parser.add_argument("--startup-delay", type=float, default=1.0)
    parser.add_argument("--post-play-delay", type=float, default=2.0)
    parser.add_argument("--stop-timeout", type=float, default=5.0)
    parser.add_argument("--no-sim-time", dest="use_sim_time", action="store_false")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.set_defaults(use_sim_time=True)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.translation_scale <= 0.0:
        raise ValueError("--translation-scale must be positive")
    if args.orb_n_features <= 0:
        raise ValueError("--orb-n-features must be positive")
    if args.orb_fast_threshold < 0:
        raise ValueError("--orb-fast-threshold must be non-negative")
    if args.opencv_threads < 0:
        raise ValueError("--opencv-threads must be non-negative")
    if args.play_rate <= 0.0:
        raise ValueError("--play-rate must be positive")
    if args.expected_visual_frames < 0:
        raise ValueError("--expected-visual-frames must be non-negative")
    if not 0.0 < args.min_visual_coverage <= 1.0:
        raise ValueError("--min-visual-coverage must be in (0, 1]")
    if math.isfinite(args.baseline_rmse_m) and args.baseline_rmse_m <= 0.0:
        raise ValueError("--baseline-rmse-m must be positive")
    if math.isfinite(args.standalone_visual_rmse_m) and args.standalone_visual_rmse_m <= 0.0:
        raise ValueError("--standalone-visual-rmse-m must be positive")

    results = run_sweep(args)
    summary = format_markdown(results, args)
    summary_out = args.summary_out or (args.out_dir / "visual_fusion_sweep.md")
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    summary_out.write_text(summary + "\n", encoding="utf-8")
    print(f"wrote fusion sweep summary: {summary_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
