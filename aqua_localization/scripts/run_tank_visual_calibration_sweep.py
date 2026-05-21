#!/usr/bin/env python3
"""Sweep Tank direct-visual calibration parameters on a fixed replay window."""

from __future__ import annotations

import argparse
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
class CalibrationCase:
    translation_scale: float
    camera_bf_scale: float
    camera_f_scale: float
    base_x_m: float
    base_y_m: float

    @property
    def label(self) -> str:
        parts = [
            ("scale", self.translation_scale),
            ("bf", self.camera_bf_scale),
            ("f", self.camera_f_scale),
            ("x", self.base_x_m),
            ("y", self.base_y_m),
        ]
        return "__".join(f"{name}_{format_label_number(value)}" for name, value in parts)


@dataclass(frozen=True)
class CalibrationResult:
    case: CalibrationCase
    sequence: str
    out_dir: Path
    estimate_tum: Path
    status_csv: Path
    rmse_m: float
    matched_seconds: float
    samples: int
    sim3_scale: float
    accepted_ratio: float
    median_pnp_inliers: float
    median_temporal_matches: float
    processed_frames: int
    accepted_frames: int
    rejected_frames: int


def parse_float_list(value: str) -> list[float]:
    values = [float(part.strip()) for part in value.split(",") if part.strip()]
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


def build_cases(args) -> list[CalibrationCase]:
    cases = [
        CalibrationCase(scale, bf_scale, f_scale, base_x, base_y)
        for scale, bf_scale, f_scale, base_x, base_y in itertools.product(
            parse_float_list(args.translation_scales),
            parse_float_list(args.camera_bf_scales),
            parse_float_list(args.camera_f_scales),
            parse_float_list(args.base_from_camera_x_m),
            parse_float_list(args.base_from_camera_y_m),
        )
    ]
    seen = set()
    unique = []
    for case in cases:
        key = (
            case.translation_scale,
            case.camera_bf_scale,
            case.camera_f_scale,
            case.base_x_m,
            case.base_y_m,
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(case)
    return unique


def case_args(args, case: CalibrationCase, sequence: str, out_dir: Path):
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
        translation_scale=case.translation_scale,
        drift_window_s=args.drift_window_s,
        drift_stride_s=args.drift_stride_s,
        drift_min_samples=args.drift_min_samples,
        segment_s=args.segment_s,
        segment_stride_s=args.segment_stride_s,
        segment_min_reference_motion_m=args.segment_min_reference_motion_m,
        camera_fx=args.camera_fx * case.camera_f_scale,
        camera_fy=args.camera_fy * case.camera_f_scale,
        camera_cx=args.camera_cx,
        camera_cy=args.camera_cy,
        camera_bf=args.camera_bf * case.camera_bf_scale,
        base_from_camera_x_m=case.base_x_m,
        base_from_camera_y_m=case.base_y_m,
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


def evaluate_case(args, case: CalibrationCase, sequence: str, out_dir: Path) -> CalibrationResult:
    current_args = case_args(args, case, sequence, out_dir)
    paths = run_tank_visual_benchmark.default_paths(out_dir, sequence)
    current_args.status_csv = paths.status_csv
    result = run_tank_visual_direct_benchmark.run_direct(current_args, paths)
    run_tank_visual_benchmark.evaluate(current_args, paths.estimate_tum, paths)
    stats, _ = compare_trajectories.compare(
        args.reference, paths.estimate_tum, with_scale=False, no_align=False
    )
    sim3_stats, _ = compare_trajectories.compare(
        args.reference, paths.estimate_tum, with_scale=True, no_align=False
    )
    accepted_ratio, median_pnp_inliers, median_temporal_matches = load_status_metrics(
        paths.status_csv
    )
    return CalibrationResult(
        case=case,
        sequence=sequence,
        out_dir=out_dir,
        estimate_tum=paths.estimate_tum,
        status_csv=paths.status_csv,
        rmse_m=float(stats["rmse"]),
        matched_seconds=float(stats["matched_seconds"]),
        samples=int(stats["count"]),
        sim3_scale=float(sim3_stats["alignment"]["scale"]),
        accepted_ratio=accepted_ratio,
        median_pnp_inliers=median_pnp_inliers,
        median_temporal_matches=median_temporal_matches,
        processed_frames=result.processed_frames,
        accepted_frames=result.accepted,
        rejected_frames=result.rejected,
    )


def run_sweep(args) -> list[CalibrationResult]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for case in build_cases(args):
        sequence = run_tank_visual_benchmark.sanitize_name(f"{args.sequence}_{case.label}")
        out_dir = args.out_dir / case.label
        results.append(evaluate_case(args, case, sequence, out_dir))
    return results


def has_baseline(args) -> bool:
    return math.isfinite(float(getattr(args, "baseline_rmse_m", math.nan)))


def format_markdown(results: list[CalibrationResult], args) -> str:
    best = min(results, key=lambda result: result.rmse_m) if results else None
    include_baseline = has_baseline(args)
    header = [
        "scale",
        "bf x",
        "f x",
        "base x",
        "base y",
        "Status",
        "RMSE m",
    ]
    separator = [
        "-----:",
        "----:",
        "---:",
        "------:",
        "------:",
        "--------",
        "-------:",
    ]
    if include_baseline:
        header.extend(["Gap x", "Improvement to tie"])
        separator.extend(["------:", "-------------------:"])
    header.extend([
        "Matched s",
        "Samples",
        "Sim(3) scale",
        "Processed",
        "Accepted",
        "Median PnP inliers",
        "Median temporal matches",
        "Output",
    ])
    separator.extend([
        "----------:",
        "--------:",
        "------------:",
        "---------:",
        "---------:",
        "-------------------:",
        "------------------------:",
        "--------",
    ])
    lines = [
        "# Tank Visual Calibration Sweep",
        "",
        f"Sequence: `{args.sequence}`",
        f"Reference: `{args.reference}`",
        f"Window: start offset `{args.start_offset_s}` s, duration `{args.duration_s}` s",
        f"Camera base fx/fy/bf: `{args.camera_fx:g}` / `{args.camera_fy:g}` / `{args.camera_bf:g}`",
    ]
    if include_baseline:
        lines.append(f"Baseline RMSE: `{args.baseline_rmse_m:g}` m")
    lines.extend([
        "",
        "| " + " | ".join(header) + " |",
        "| " + " | ".join(separator) + " |",
    ])
    for result in results:
        c = result.case
        marker = "best" if best is result else "ok"
        cells = [
            format_float(c.translation_scale, 6),
            format_float(c.camera_bf_scale, 4),
            format_float(c.camera_f_scale, 4),
            format_float(c.base_x_m, 3),
            format_float(c.base_y_m, 3),
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
            format_float(result.sim3_scale, 6),
            str(result.processed_frames),
            format_percent(result.accepted_ratio),
            format_float(result.median_pnp_inliers, 1),
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
                f"`translation_scale={format_float(c.translation_scale, 9)}`, "
                f"`camera_bf_scale={format_float(c.camera_bf_scale, 6)}`, "
                f"`camera_f_scale={format_float(c.camera_f_scale, 6)}`, "
                f"`base_from_camera=({format_float(c.base_x_m, 3)}, "
                f"{format_float(c.base_y_m, 3)}, {format_float(args.base_from_camera_z_m, 3)})`."
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


def write_results_csv(path: Path, results: list[CalibrationResult], args) -> None:
    import csv

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "translation_scale",
        "camera_bf_scale",
        "camera_f_scale",
        "camera_fx",
        "camera_fy",
        "camera_bf",
        "base_from_camera_x_m",
        "base_from_camera_y_m",
        "rmse_m",
        "gap_to_baseline",
        "matched_seconds",
        "samples",
        "sim3_scale",
        "processed_frames",
        "accepted_ratio",
        "median_pnp_inliers",
        "median_temporal_matches",
        "out_dir",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            c = result.case
            writer.writerow({
                "translation_scale": c.translation_scale,
                "camera_bf_scale": c.camera_bf_scale,
                "camera_f_scale": c.camera_f_scale,
                "camera_fx": args.camera_fx * c.camera_f_scale,
                "camera_fy": args.camera_fy * c.camera_f_scale,
                "camera_bf": args.camera_bf * c.camera_bf_scale,
                "base_from_camera_x_m": c.base_x_m,
                "base_from_camera_y_m": c.base_y_m,
                "rmse_m": result.rmse_m,
                "gap_to_baseline": gap_ratio(result.rmse_m, args.baseline_rmse_m),
                "matched_seconds": result.matched_seconds,
                "samples": result.samples,
                "sim3_scale": result.sim3_scale,
                "processed_frames": result.processed_frames,
                "accepted_ratio": result.accepted_ratio,
                "median_pnp_inliers": result.median_pnp_inliers,
                "median_temporal_matches": result.median_temporal_matches,
                "out_dir": str(result.out_dir),
            })


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Sweep direct Tank visual calibration parameters on a fixed window."
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_visual_calibration_sweep"))
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
    parser.add_argument("--translation-scales", default="0.08,0.095,0.105024091,0.115,0.13")
    parser.add_argument("--camera-bf-scales", default="1.0")
    parser.add_argument("--camera-f-scales", default="1.0")
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
    parser.add_argument("--base-from-camera-x-m", default="-0.25")
    parser.add_argument("--base-from-camera-y-m", default="-0.45")
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
    if args.sync_slop_s < 0.0:
        raise ValueError("--sync-slop-s must be non-negative")
    if args.start_offset_s is not None and args.start_offset_s < 0.0:
        raise ValueError("--start-offset-s must be non-negative")
    if args.duration_s is not None and args.duration_s <= 0.0:
        raise ValueError("--duration-s must be positive")
    if args.camera_fx <= 0.0 or args.camera_fy <= 0.0:
        raise ValueError("--camera-fx and --camera-fy must be positive")
    if args.camera_bf <= 0.0:
        raise ValueError("--camera-bf must be positive")
    if args.orb_n_features <= 0:
        raise ValueError("--orb-n-features must be positive")
    if args.orb_fast_threshold < 0:
        raise ValueError("--orb-fast-threshold must be non-negative")
    if args.opencv_threads < 0:
        raise ValueError("--opencv-threads must be non-negative")
    if math.isfinite(args.baseline_rmse_m) and args.baseline_rmse_m <= 0.0:
        raise ValueError("--baseline-rmse-m must be positive")
    for scale in parse_float_list(args.translation_scales):
        if scale <= 0.0:
            raise ValueError("--translation-scales values must be positive")
    for scale in parse_float_list(args.camera_bf_scales):
        if scale <= 0.0:
            raise ValueError("--camera-bf-scales values must be positive")
    for scale in parse_float_list(args.camera_f_scales):
        if scale <= 0.0:
            raise ValueError("--camera-f-scales values must be positive")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    validate_args(args)
    results = run_sweep(args)
    summary = format_markdown(results, args)
    summary_out = args.summary_out or (args.out_dir / "visual_calibration_sweep.md")
    csv_out = args.csv_out or (args.out_dir / "visual_calibration_sweep.csv")
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    summary_out.write_text(summary, encoding="utf-8")
    write_results_csv(csv_out, results, args)
    print(f"wrote calibration sweep summary: {summary_out}")
    print(f"wrote calibration sweep csv: {csv_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
