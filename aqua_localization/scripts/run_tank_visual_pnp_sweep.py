#!/usr/bin/env python3
"""Sweep Tank direct-visual PnP/RANSAC quality gates on a fixed window."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import itertools
import math
from pathlib import Path
from types import SimpleNamespace
import sys

import compare_trajectories
import run_tank_visual_benchmark
import run_tank_visual_direct_benchmark
import summarize_visual_frontend_status


@dataclass(frozen=True)
class PnpCase:
    reprojection_error_px: float
    min_inlier_ratio: float
    max_step_translation_m: float
    min_pnp_inliers: int
    ransac_iterations: int
    ransac_confidence: float

    @property
    def label(self) -> str:
        parts = [
            ("repr", self.reprojection_error_px),
            ("ratio", self.min_inlier_ratio),
            ("step", self.max_step_translation_m),
            ("inl", float(self.min_pnp_inliers)),
            ("iter", float(self.ransac_iterations)),
            ("conf", self.ransac_confidence),
        ]
        return "__".join(f"{name}_{format_label_number(value)}" for name, value in parts)


@dataclass(frozen=True)
class PnpSweepResult:
    case: PnpCase
    sequence: str
    out_dir: Path
    rmse_m: float
    matched_seconds: float
    samples: int
    accepted_ratio: float
    rejected_frames: int
    dominant_rejection: str
    median_pnp_inliers: float
    median_inlier_ratio: float
    median_temporal_matches: float
    error: str = ""


def parse_float_list(value: str) -> list[float]:
    values = [float(part.strip()) for part in value.split(",") if part.strip()]
    if not values:
        raise ValueError("candidate list is empty")
    return values


def parse_int_list(value: str) -> list[int]:
    values = [int(part.strip()) for part in value.split(",") if part.strip()]
    if not values:
        raise ValueError("candidate list is empty")
    return values


def format_label_number(value: float) -> str:
    if abs(value) < 1.0e-12:
        value = 0.0
    return f"{value:g}".replace("-", "m").replace(".", "p")


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


def build_cases(args) -> list[PnpCase]:
    cases = [
        PnpCase(reproj, ratio, step, min_inliers, iterations, confidence)
        for reproj, ratio, step, min_inliers, iterations, confidence in itertools.product(
            parse_float_list(args.reprojection_errors_px),
            parse_float_list(args.min_inlier_ratios),
            parse_float_list(args.max_step_translation_m),
            parse_int_list(args.min_pnp_inliers),
            parse_int_list(args.ransac_iterations),
            parse_float_list(args.ransac_confidences),
        )
    ]
    seen = set()
    unique = []
    for case in cases:
        key = (
            case.reprojection_error_px,
            case.min_inlier_ratio,
            case.max_step_translation_m,
            case.min_pnp_inliers,
            case.ransac_iterations,
            case.ransac_confidence,
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(case)
    return unique


def case_args(args, case: PnpCase, sequence: str, out_dir: Path):
    return SimpleNamespace(
        bag=args.bag,
        reference=args.reference,
        out_dir=out_dir,
        dataset=args.dataset,
        sequence=sequence,
        system=args.system,
        left_topic=args.left_topic,
        right_topic=args.right_topic,
        sync_slop_s=args.sync_slop_s,
        start_offset_s=args.start_offset_s,
        duration_s=args.duration_s,
        start_stamp_s=args.start_stamp_s,
        end_stamp_s=args.end_stamp_s,
        translation_scale=args.translation_scale,
        min_pnp_inliers=case.min_pnp_inliers,
        min_inlier_ratio=case.min_inlier_ratio,
        ransac_iterations=case.ransac_iterations,
        ransac_reprojection_error_px=case.reprojection_error_px,
        ransac_confidence=case.ransac_confidence,
        max_step_translation_m=case.max_step_translation_m,
        drift_window_s=args.drift_window_s,
        drift_stride_s=args.drift_stride_s,
        drift_min_samples=args.drift_min_samples,
        segment_s=args.segment_s,
        segment_stride_s=args.segment_stride_s,
        segment_min_reference_motion_m=args.segment_min_reference_motion_m,
        camera_fx=args.camera_fx,
        camera_fy=args.camera_fy,
        camera_cx=args.camera_cx,
        camera_cy=args.camera_cy,
        camera_bf=args.camera_bf,
        base_from_camera_x_m=args.base_from_camera_x_m,
        base_from_camera_y_m=args.base_from_camera_y_m,
        base_from_camera_z_m=args.base_from_camera_z_m,
        base_from_camera_roll_rad=args.base_from_camera_roll_rad,
        base_from_camera_pitch_rad=args.base_from_camera_pitch_rad,
        base_from_camera_yaw_rad=args.base_from_camera_yaw_rad,
        publish_base_pose=args.publish_base_pose,
        max_stereo_descriptor_distance=args.max_stereo_descriptor_distance,
        max_temporal_descriptor_distance=args.max_temporal_descriptor_distance,
        orb_n_features=args.orb_n_features,
        orb_fast_threshold=args.orb_fast_threshold,
        opencv_threads=args.opencv_threads,
        warmup=args.warmup,
        status_csv=None,
    )


def status_metrics(status_csv: Path) -> tuple[float, int, str, float, float, float]:
    if not status_csv.exists():
        return math.nan, 0, "missing status", math.nan, math.nan, math.nan
    samples = summarize_visual_frontend_status.read_csv(status_csv)
    summary = summarize_visual_frontend_status.summarize(samples)
    moving = summary["moving_numeric"]
    rejections: Counter = summary["rejection_counts"]
    dominant = rejections.most_common(1)[0][0] if rejections else "none"
    return (
        float(summary["acceptance_ratio"]),
        int(summary["rejected"]),
        dominant,
        float(moving["pnp_inliers"]["median"]),
        float(moving["inlier_ratio"]["median"]),
        float(moving["temporal_matches"]["median"]),
    )


def evaluate_case(args, case: PnpCase, sequence: str, out_dir: Path) -> PnpSweepResult:
    current_args = case_args(args, case, sequence, out_dir)
    paths = run_tank_visual_benchmark.default_paths(out_dir, sequence)
    current_args.status_csv = paths.status_csv
    run_tank_visual_direct_benchmark.run_direct(current_args, paths)
    error = ""
    try:
        stats, _ = compare_trajectories.compare(
            args.reference, paths.estimate_tum, with_scale=False, no_align=False
        )
    except Exception as exc:
        error = str(exc)
        stats = {"rmse": math.nan, "matched_seconds": math.nan, "count": 0}
    try:
        run_tank_visual_benchmark.evaluate(current_args, paths.estimate_tum, paths)
    except Exception as exc:
        error = str(exc)
    (
        accepted_ratio,
        rejected_frames,
        dominant_rejection,
        median_pnp_inliers,
        median_inlier_ratio,
        median_temporal_matches,
    ) = status_metrics(paths.status_csv)
    return PnpSweepResult(
        case=case,
        sequence=sequence,
        out_dir=out_dir,
        rmse_m=float(stats["rmse"]),
        matched_seconds=float(stats["matched_seconds"]),
        samples=int(stats["count"]),
        accepted_ratio=accepted_ratio,
        rejected_frames=rejected_frames,
        dominant_rejection=dominant_rejection,
        median_pnp_inliers=median_pnp_inliers,
        median_inlier_ratio=median_inlier_ratio,
        median_temporal_matches=median_temporal_matches,
        error=error,
    )


def run_sweep(args) -> list[PnpSweepResult]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for case in build_cases(args):
        sequence = run_tank_visual_benchmark.sanitize_name(f"{args.sequence}_{case.label}")
        out_dir = args.out_dir / case.label
        results.append(evaluate_case(args, case, sequence, out_dir))
    return results


def has_baseline(args) -> bool:
    return math.isfinite(float(getattr(args, "baseline_rmse_m", math.nan)))


def format_markdown(results: list[PnpSweepResult], args) -> str:
    valid = [result for result in results if not result.error and math.isfinite(result.rmse_m)]
    best = min(valid, key=lambda result: result.rmse_m) if valid else None
    include_baseline = has_baseline(args)
    header = ["reproj px", "min ratio", "max step", "min inl", "iters", "conf", "Status", "RMSE m"]
    separator = ["---------:", "---------:", "--------:", "-------:", "-----:", "----:", "--------", "-------:"]
    if include_baseline:
        header.extend(["Gap x", "Improvement to tie"])
        separator.extend(["------:", "-------------------:"])
    header.extend([
        "Matched s",
        "Samples",
        "Accepted",
        "Rejected",
        "Reject reason",
        "Median inliers",
        "Median ratio",
        "Median matches",
        "Output",
    ])
    separator.extend([
        "----------:",
        "--------:",
        "---------:",
        "---------:",
        "-------------",
        "--------------:",
        "------------:",
        "-------------:",
        "--------",
    ])
    lines = [
        "# Tank Visual PnP Quality Sweep",
        "",
        f"Sequence: `{args.sequence}`",
        f"Reference: `{args.reference}`",
        f"Window: start offset `{args.start_offset_s}` s, duration `{args.duration_s}` s",
        f"Translation scale: `{args.translation_scale:g}`",
    ]
    if include_baseline:
        lines.append(f"Baseline RMSE: `{args.baseline_rmse_m:g}` m")
    lines.extend(["", "| " + " | ".join(header) + " |", "| " + " | ".join(separator) + " |"])
    for result in results:
        c = result.case
        marker = "failed" if result.error else "best" if best is result else "ok"
        cells = [
            format_float(c.reprojection_error_px, 2),
            format_float(c.min_inlier_ratio, 2),
            format_float(c.max_step_translation_m, 3),
            str(c.min_pnp_inliers),
            str(c.ransac_iterations),
            format_float(c.ransac_confidence, 3),
            marker,
            format_float(result.rmse_m),
        ]
        if include_baseline:
            cells.extend([
                format_float(gap_ratio(result.rmse_m, args.baseline_rmse_m), 2),
                f"{format_float(improvement_to_tie_percent(result.rmse_m, args.baseline_rmse_m), 1)}%",
            ])
        cells.extend([
            format_float(result.matched_seconds, 2),
            str(result.samples),
            format_percent(result.accepted_ratio),
            str(result.rejected_frames),
            result.dominant_rejection,
            format_float(result.median_pnp_inliers, 1),
            format_float(result.median_inlier_ratio, 3),
            format_float(result.median_temporal_matches, 1),
            f"`{result.out_dir}`",
        ])
        lines.append("| " + " | ".join(cells) + " |")

    if best is not None:
        c = best.case
        lines.extend([
            "",
            "## Readout",
            "",
            (
                f"Best RMSE: `{format_float(best.rmse_m)}` m with "
                f"`reprojection_error={format_float(c.reprojection_error_px, 2)}px`, "
                f"`min_inlier_ratio={format_float(c.min_inlier_ratio, 2)}`, "
                f"`max_step_translation={format_float(c.max_step_translation_m, 3)}m`, "
                f"`min_pnp_inliers={c.min_pnp_inliers}`, "
                f"`ransac_iterations={c.ransac_iterations}`, "
                f"`ransac_confidence={format_float(c.ransac_confidence, 3)}`."
            ),
        ])
        if include_baseline:
            lines.append(
                f"Best gap to baseline: "
                f"`{format_float(gap_ratio(best.rmse_m, args.baseline_rmse_m), 2)}x`; "
                f"RMSE reduction still needed to tie: "
                f"`{format_float(improvement_to_tie_percent(best.rmse_m, args.baseline_rmse_m), 1)}%`."
            )
    lines.append("")
    return "\n".join(lines)


def write_results_csv(path: Path, results: list[PnpSweepResult], args) -> None:
    import csv

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "reprojection_error_px",
        "min_inlier_ratio",
        "max_step_translation_m",
        "min_pnp_inliers",
        "ransac_iterations",
        "ransac_confidence",
        "rmse_m",
        "gap_to_baseline",
        "matched_seconds",
        "samples",
        "accepted_ratio",
        "rejected_frames",
        "dominant_rejection",
        "median_pnp_inliers",
        "median_inlier_ratio",
        "median_temporal_matches",
        "error",
        "out_dir",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            c = result.case
            writer.writerow({
                "reprojection_error_px": c.reprojection_error_px,
                "min_inlier_ratio": c.min_inlier_ratio,
                "max_step_translation_m": c.max_step_translation_m,
                "min_pnp_inliers": c.min_pnp_inliers,
                "ransac_iterations": c.ransac_iterations,
                "ransac_confidence": c.ransac_confidence,
                "rmse_m": result.rmse_m,
                "gap_to_baseline": gap_ratio(result.rmse_m, args.baseline_rmse_m),
                "matched_seconds": result.matched_seconds,
                "samples": result.samples,
                "accepted_ratio": result.accepted_ratio,
                "rejected_frames": result.rejected_frames,
                "dominant_rejection": result.dominant_rejection,
                "median_pnp_inliers": result.median_pnp_inliers,
                "median_inlier_ratio": result.median_inlier_ratio,
                "median_temporal_matches": result.median_temporal_matches,
                "error": result.error,
                "out_dir": str(result.out_dir),
            })


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Sweep direct Tank visual PnP/RANSAC quality gates on a fixed window."
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_visual_pnp_sweep"))
    parser.add_argument("--summary-out", type=Path, default=None)
    parser.add_argument("--csv-out", type=Path, default=None)
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--sequence", default="short_test")
    parser.add_argument("--system", default="aqua_visual_frontend_direct")
    parser.add_argument("--left-topic", default=run_tank_visual_direct_benchmark.DEFAULT_LEFT_TOPIC)
    parser.add_argument("--right-topic", default=run_tank_visual_direct_benchmark.DEFAULT_RIGHT_TOPIC)
    parser.add_argument("--sync-slop-s", type=float, default=0.02)
    parser.add_argument("--start-offset-s", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float, default=11.25)
    parser.add_argument("--start-stamp-s", type=float, default=None)
    parser.add_argument("--end-stamp-s", type=float, default=None)
    parser.add_argument("--translation-scale", type=float, default=0.095)
    parser.add_argument("--reprojection-errors-px", default="2,3,4,6")
    parser.add_argument("--min-inlier-ratios", default="0.25,0.5,0.65,0.8")
    parser.add_argument("--max-step-translation-m", default="0.02,0.05,0.1,2.0")
    parser.add_argument("--min-pnp-inliers", default="12")
    parser.add_argument("--ransac-iterations", default="100")
    parser.add_argument("--ransac-confidences", default="0.99")
    parser.add_argument("--baseline-rmse-m", type=float, default=math.nan)
    parser.add_argument("--drift-window-s", type=float, default=3.0)
    parser.add_argument("--drift-stride-s", type=float, default=1.0)
    parser.add_argument("--drift-min-samples", type=int, default=20)
    parser.add_argument("--segment-s", type=float, default=1.0)
    parser.add_argument("--segment-stride-s", type=float, default=0.5)
    parser.add_argument("--segment-min-reference-motion-m", type=float, default=0.01)
    parser.add_argument("--camera-fx", type=float, default=run_tank_visual_benchmark.DEFAULT_FX)
    parser.add_argument("--camera-fy", type=float, default=run_tank_visual_benchmark.DEFAULT_FY)
    parser.add_argument("--camera-cx", type=float, default=run_tank_visual_benchmark.DEFAULT_CX)
    parser.add_argument("--camera-cy", type=float, default=run_tank_visual_benchmark.DEFAULT_CY)
    parser.add_argument("--camera-bf", type=float, default=run_tank_visual_benchmark.DEFAULT_BF)
    parser.add_argument("--base-from-camera-x-m", type=float, default=-0.25)
    parser.add_argument("--base-from-camera-y-m", type=float, default=-0.45)
    parser.add_argument("--base-from-camera-z-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-roll-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-pitch-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-yaw-rad", type=float, default=0.0)
    parser.add_argument("--publish-base-pose", action="store_true")
    parser.add_argument("--max-stereo-descriptor-distance", type=float, default=64.0)
    parser.add_argument("--max-temporal-descriptor-distance", type=float, default=64.0)
    parser.add_argument("--orb-n-features", type=int, default=700)
    parser.add_argument("--orb-fast-threshold", type=int, default=16)
    parser.add_argument("--opencv-threads", type=int, default=2)
    parser.add_argument("--no-warmup", dest="warmup", action="store_false")
    parser.set_defaults(warmup=True)
    return parser.parse_args(argv)


def validate_args(args) -> None:
    if args.translation_scale <= 0.0:
        raise ValueError("--translation-scale must be positive")
    if args.sync_slop_s < 0.0:
        raise ValueError("--sync-slop-s must be non-negative")
    if args.start_offset_s is not None and args.start_offset_s < 0.0:
        raise ValueError("--start-offset-s must be non-negative")
    if args.duration_s is not None and args.duration_s <= 0.0:
        raise ValueError("--duration-s must be positive")
    if args.orb_n_features <= 0:
        raise ValueError("--orb-n-features must be positive")
    if args.orb_fast_threshold < 0:
        raise ValueError("--orb-fast-threshold must be non-negative")
    if args.opencv_threads < 0:
        raise ValueError("--opencv-threads must be non-negative")
    if math.isfinite(args.baseline_rmse_m) and args.baseline_rmse_m <= 0.0:
        raise ValueError("--baseline-rmse-m must be positive")
    for value in parse_float_list(args.reprojection_errors_px):
        if value <= 0.0:
            raise ValueError("--reprojection-errors-px values must be positive")
    for value in parse_float_list(args.min_inlier_ratios):
        if not 0.0 <= value <= 1.0:
            raise ValueError("--min-inlier-ratios values must be in [0, 1]")
    for value in parse_float_list(args.max_step_translation_m):
        if value <= 0.0:
            raise ValueError("--max-step-translation-m values must be positive")
    for value in parse_int_list(args.min_pnp_inliers):
        if value < 0:
            raise ValueError("--min-pnp-inliers values must be non-negative")
    for value in parse_int_list(args.ransac_iterations):
        if value <= 0:
            raise ValueError("--ransac-iterations values must be positive")
    for value in parse_float_list(args.ransac_confidences):
        if not 0.0 < value < 1.0:
            raise ValueError("--ransac-confidences values must be in (0, 1)")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    validate_args(args)
    results = run_sweep(args)
    summary = format_markdown(results, args)
    summary_out = args.summary_out or (args.out_dir / "visual_pnp_sweep.md")
    csv_out = args.csv_out or (args.out_dir / "visual_pnp_sweep.csv")
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    summary_out.write_text(summary, encoding="utf-8")
    write_results_csv(csv_out, results, args)
    print(f"wrote PnP sweep summary: {summary_out}")
    print(f"wrote PnP sweep csv: {csv_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
