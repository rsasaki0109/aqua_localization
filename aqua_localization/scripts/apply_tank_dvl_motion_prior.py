#!/usr/bin/env python3
"""Apply a real Tank DVL/IMU motion prior to a visual TUM trajectory."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np

import simulate_visual_motion_prior as prior_sim
import tank_dvl_prior_profile
from tank_dvl_prior_core import (
    DvlPriorDelta,
    build_dvl_prior_deltas,
    positions_from_deltas,
    score_prior_confidence,
)
from tank_rosbag_motion_inputs import (
    DEFAULT_DVL_TOPIC,
    DEFAULT_IMU_TOPIC,
    read_dvl_records,
    read_imu_yaw_records,
)


@dataclass(frozen=True)
class TankDvlPriorApplicationResult:
    original_rmse_m: float
    corrected_rmse_m: float
    rmse_improvement_percent: float
    samples: int
    matched_s: float
    steps: int
    covered_steps: int
    prior_steps: int
    corrected_tum: Path
    aligned_visual_tum: Path
    dvl_prior_tum: Path


PROFILE_DEFAULTS = {
    "dvl_yaw_mode": "imu_yaw",
    "dvl_frame_yaw_offset_deg": -90.0,
    "imu_yaw_offset_deg": 115.0,
    "prior_scale": 1.0,
    "mode": "blend-outliers",
    "blend_alpha": 0.5,
    "min_prior_step_m": 1.0e-4,
    "min_visual_step_m": 0.0,
    "min_length_ratio": 0.5,
    "max_length_ratio": 1.5,
    "min_direction_cosine": 0.5,
}


def load_compare_module():
    script_path = Path(__file__).resolve().parent / "compare_trajectories.py"
    spec = importlib.util.spec_from_file_location("compare_trajectories", script_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def matched_reference_and_aligned_visual(reference_path: Path, visual_path: Path):
    compare = load_compare_module()
    reference = compare.load_tum(reference_path)
    visual = compare.load_tum(visual_path)
    reference_xyz = compare.interpolate_positions(reference, visual[:, 0])
    valid = ~np.isnan(reference_xyz).any(axis=1)
    if np.count_nonzero(valid) < 2:
        raise ValueError("need at least two visual timestamps overlapping reference")
    times = visual[valid, 0]
    visual_xyz = visual[valid, 1:4]
    reference_xyz = reference_xyz[valid]
    rotation, translation, scale = compare.umeyama_alignment(
        visual_xyz,
        reference_xyz,
        with_scale=False,
    )
    aligned_visual_xyz = compare.apply_transform(visual_xyz, rotation, translation, scale)
    return times, reference_xyz, aligned_visual_xyz, reference


def prior_step_quality_rows(
    times: np.ndarray,
    visual_xyz: np.ndarray,
    prior_xyz: np.ndarray,
    deltas: list[DvlPriorDelta],
    sim_rows: list[prior_sim.PriorStep],
    *,
    min_visual_step_m: float = 0.0,
    min_prior_step_m: float = 0.0,
    min_length_ratio: float = 0.5,
    max_length_ratio: float = 1.5,
    min_direction_cosine: float = 0.5,
) -> list[dict]:
    rows = []
    t0 = float(times[0])
    for index, (delta, sim_row) in enumerate(zip(deltas, sim_rows), start=1):
        visual_delta = visual_xyz[index] - visual_xyz[index - 1]
        prior_delta = prior_xyz[index] - prior_xyz[index - 1]
        confidence = score_prior_confidence(
            visual_delta,
            prior_delta,
            min_visual_step_m=min_visual_step_m,
            min_prior_step_m=min_prior_step_m,
            min_length_ratio=min_length_ratio,
            max_length_ratio=max_length_ratio,
            min_direction_cosine=min_direction_cosine,
        )
        rows.append({
            "start_stamp_s": float(times[index - 1]),
            "end_stamp_s": float(times[index]),
            "offset_s": float(times[index] - t0),
            "dt_s": float(times[index] - times[index - 1]),
            "visual_step_m": float(np.linalg.norm(visual_delta)),
            "prior_step_m": float(np.linalg.norm(prior_delta)),
            "corrected_step_m": sim_row.corrected_step_m,
            "visual_prior_length_ratio": sim_row.length_ratio,
            "visual_prior_direction_cosine": sim_row.direction_cosine,
            "visual_prior_heading_error_deg": sim_row.heading_error_deg,
            "visual_prior_residual_m": confidence.residual_m,
            "prior_match_confidence": confidence.confidence,
            "prior_confidence": (
                sim_row.prior_confidence
                if math.isfinite(sim_row.prior_confidence)
                else confidence.confidence
            ),
            "prior_confidence_accepted": confidence.accepted,
            "prior_reject_reason": confidence.reject_reason,
            "effective_blend_alpha": sim_row.effective_blend_alpha,
            "confidence_mode": sim_row.confidence_mode,
            "dvl_covered": delta.covered,
            "dvl_samples": delta.dvl_samples,
            "used_prior": sim_row.used_prior,
            "reason": sim_row.reason,
        })
    return rows


def write_application_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "start_stamp_s",
        "end_stamp_s",
        "offset_s",
        "dt_s",
        "visual_step_m",
        "prior_step_m",
        "corrected_step_m",
        "visual_prior_length_ratio",
        "visual_prior_direction_cosine",
        "visual_prior_heading_error_deg",
        "visual_prior_residual_m",
        "prior_match_confidence",
        "prior_confidence",
        "prior_confidence_accepted",
        "prior_reject_reason",
        "effective_blend_alpha",
        "confidence_mode",
        "dvl_covered",
        "dvl_samples",
        "used_prior",
        "reason",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row[field] for field in fieldnames})


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def format_markdown(args, result: TankDvlPriorApplicationResult, sim_rows: list[prior_sim.PriorStep]) -> str:
    reason_counts: dict[str, int] = {}
    for row in sim_rows:
        if row.used_prior:
            reason_counts[row.reason] = reason_counts.get(row.reason, 0) + 1
    lines = [
        "# Tank DVL Motion Prior Application",
        "",
        f"- Bag: `{args.bag}`",
        f"- Reference: `{args.reference}`",
        f"- Visual estimate: `{args.visual}`",
        f"- Profile: `{args.profile}`",
        f"- Profile label: `{args.profile_label}`",
        f"- Calibration sequence: `{args.profile_calibration_sequence}`",
        f"- Validation sequence: `{args.profile_validation_sequence}`",
        f"- DVL topic: `{args.dvl_topic}`",
        f"- IMU topic: `{args.imu_topic}`",
        f"- DVL yaw mode: `{args.dvl_yaw_mode}`",
        f"- DVL frame yaw offset: {format_float(args.dvl_frame_yaw_offset_deg, 1)} deg",
        f"- IMU yaw offset: {format_float(args.imu_yaw_offset_deg, 1)} deg",
        f"- Prior scale: {format_float(args.prior_scale, 4)}",
        f"- Application mode: `{args.mode}`",
        f"- Blend alpha: {format_float(args.blend_alpha, 3)}",
        f"- Min visual step for confidence: {format_float(args.min_visual_step_m, 4)} m",
        f"- Length-ratio gate: [{format_float(args.min_length_ratio, 3)}, {format_float(args.max_length_ratio, 3)}]",
        f"- Min direction cosine: {format_float(args.min_direction_cosine, 3)}",
        f"- Original RMSE: {result.original_rmse_m:.4f} m",
        f"- Corrected RMSE: {result.corrected_rmse_m:.4f} m",
        f"- RMSE improvement: {format_float(result.rmse_improvement_percent, 1)}%",
        f"- DVL-covered steps: {result.covered_steps}/{result.steps}",
        f"- Prior-applied steps: {result.prior_steps}/{result.steps}",
        f"- Corrected TUM: `{result.corrected_tum}`",
        f"- Aligned visual TUM: `{result.aligned_visual_tum}`",
        f"- DVL prior TUM: `{result.dvl_prior_tum}`",
        f"- Step CSV: `{args.csv_out}`",
        "",
        "## Prior Reasons",
        "",
        "| Reason | Count |",
        "|--------|------:|",
    ]
    if reason_counts:
        for reason, count in sorted(reason_counts.items(), key=lambda item: (-item[1], item[0])):
            lines.append(f"| {reason} | {count} |")
    else:
        lines.append("| none | 0 |")
    lines.extend(["", "## Interpretation", ""])
    if result.corrected_rmse_m < result.original_rmse_m:
        lines.append("- The real DVL/IMU prior reduced this visual trajectory's aligned RMSE.")
    else:
        lines.append("- The real DVL/IMU prior did not reduce RMSE with these gates; tune scale/gates or use softer fusion.")
    lines.append("- If offsets or scale are tuned on this same sequence, treat this as a diagnostic rather than a paper-safe benchmark.")
    lines.append("")
    return "\n".join(lines)


def run_application(args) -> tuple[TankDvlPriorApplicationResult, list[prior_sim.PriorStep], list[dict]]:
    compare = load_compare_module()
    times, _reference_xyz, aligned_visual_xyz, reference_tum = matched_reference_and_aligned_visual(
        args.reference,
        args.visual,
    )
    dvl_records = read_dvl_records(args.bag, args.dvl_topic)
    imu_records = read_imu_yaw_records(args.bag, args.imu_topic) if args.dvl_yaw_mode == "imu_yaw" else None
    deltas = build_dvl_prior_deltas(
        times,
        dvl_records,
        reference_tum,
        args.dvl_yaw_mode,
        math.radians(args.dvl_frame_yaw_offset_deg),
        imu_records,
        math.radians(args.imu_yaw_offset_deg),
        args.prior_scale,
    )
    prior_xyz = positions_from_deltas(times, aligned_visual_xyz[0], deltas)
    corrected_xyz, sim_rows = prior_sim.simulate_prior(
        times,
        aligned_visual_xyz,
        prior_xyz,
        mode=args.mode,
        blend_alpha=args.blend_alpha,
        min_reference_step_m=args.min_prior_step_m,
        min_length_ratio=args.min_length_ratio,
        max_length_ratio=args.max_length_ratio,
        min_direction_cosine=args.min_direction_cosine,
    )
    prior_sim.write_tum(args.aligned_visual_out, times, aligned_visual_xyz)
    prior_sim.write_tum(args.dvl_prior_out, times, prior_xyz)
    prior_sim.write_tum(args.corrected_out, times, corrected_xyz)
    original_stats, _ = compare.compare(args.reference, args.aligned_visual_out, with_scale=False, no_align=False)
    corrected_stats, _ = compare.compare(args.reference, args.corrected_out, with_scale=False, no_align=False)
    prior_count = sum(1 for row in sim_rows if row.used_prior)
    original_rmse = float(original_stats["rmse"])
    corrected_rmse = float(corrected_stats["rmse"])
    improvement = 100.0 * (original_rmse - corrected_rmse) / original_rmse if original_rmse > 0.0 else math.nan
    result = TankDvlPriorApplicationResult(
        original_rmse_m=original_rmse,
        corrected_rmse_m=corrected_rmse,
        rmse_improvement_percent=improvement,
        samples=int(corrected_stats["count"]),
        matched_s=float(corrected_stats["matched_seconds"]),
        steps=len(sim_rows),
        covered_steps=sum(1 for delta in deltas if delta.covered),
        prior_steps=prior_count,
        corrected_tum=args.corrected_out,
        aligned_visual_tum=args.aligned_visual_out,
        dvl_prior_tum=args.dvl_prior_out,
    )
    quality_rows = prior_step_quality_rows(
        times,
        aligned_visual_xyz,
        prior_xyz,
        deltas,
        sim_rows,
        min_visual_step_m=args.min_visual_step_m,
        min_prior_step_m=args.min_prior_step_m,
        min_length_ratio=args.min_length_ratio,
        max_length_ratio=args.max_length_ratio,
        min_direction_cosine=args.min_direction_cosine,
    )
    return result, sim_rows, quality_rows


def parse_args(argv):
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--profile", type=Path)
    pre_args, _ = pre_parser.parse_known_args(argv)
    defaults = dict(PROFILE_DEFAULTS)
    profile = {}
    if pre_args.profile is not None:
        profile = tank_dvl_prior_profile.load_profile(pre_args.profile)
        defaults.update(tank_dvl_prior_profile.profile_arg_defaults(pre_args.profile))

    parser = argparse.ArgumentParser(
        description="Apply a Tank DVL/IMU motion prior to a visual trajectory."
    )
    parser.add_argument("--profile", type=Path, default=pre_args.profile)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--visual", required=True, type=Path)
    parser.add_argument("--dvl-topic", default=DEFAULT_DVL_TOPIC)
    parser.add_argument("--imu-topic", default=DEFAULT_IMU_TOPIC)
    parser.add_argument("--dvl-yaw-mode", choices=["body_raw", "gt_yaw", "imu_yaw"], default=defaults["dvl_yaw_mode"])
    parser.add_argument("--dvl-frame-yaw-offset-deg", type=float, default=defaults["dvl_frame_yaw_offset_deg"])
    parser.add_argument("--imu-yaw-offset-deg", type=float, default=defaults["imu_yaw_offset_deg"])
    parser.add_argument("--prior-scale", type=float, default=defaults["prior_scale"])
    parser.add_argument(
        "--mode",
        choices=prior_sim.APPLICATION_MODES,
        default=defaults["mode"],
    )
    parser.add_argument("--blend-alpha", type=float, default=defaults["blend_alpha"])
    parser.add_argument("--min-prior-step-m", type=float, default=defaults["min_prior_step_m"])
    parser.add_argument("--min-visual-step-m", type=float, default=defaults["min_visual_step_m"])
    parser.add_argument("--min-length-ratio", type=float, default=defaults["min_length_ratio"])
    parser.add_argument("--max-length-ratio", type=float, default=defaults["max_length_ratio"])
    parser.add_argument("--min-direction-cosine", type=float, default=defaults["min_direction_cosine"])
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_dvl_motion_prior_apply"))
    parser.add_argument("--summary-out", type=Path, default=None)
    parser.add_argument("--csv-out", type=Path, default=None)
    parser.add_argument("--corrected-out", type=Path, default=None)
    parser.add_argument("--aligned-visual-out", type=Path, default=None)
    parser.add_argument("--dvl-prior-out", type=Path, default=None)
    args = parser.parse_args(argv)
    args.profile_data = profile
    args.profile_label = (
        tank_dvl_prior_profile.profile_label(args.profile, profile)
        if args.profile is not None
        else ""
    )
    metadata = profile.get("metadata", {}) if isinstance(profile, dict) else {}
    args.profile_calibration_sequence = metadata.get("calibration_sequence", "") if isinstance(metadata, dict) else ""
    args.profile_validation_sequence = metadata.get("validation_sequence", "") if isinstance(metadata, dict) else ""
    return args


def fill_default_outputs(args) -> None:
    if args.summary_out is None:
        args.summary_out = args.out_dir / "tank_dvl_motion_prior_apply.md"
    if args.csv_out is None:
        args.csv_out = args.out_dir / "tank_dvl_motion_prior_steps.csv"
    if args.corrected_out is None:
        args.corrected_out = args.out_dir / "tank_dvl_motion_prior_corrected.tum"
    if args.aligned_visual_out is None:
        args.aligned_visual_out = args.out_dir / "aligned_visual_input.tum"
    if args.dvl_prior_out is None:
        args.dvl_prior_out = args.out_dir / "dvl_motion_prior.tum"


def validate_args(args) -> None:
    if not 0.0 <= args.blend_alpha <= 1.0:
        raise ValueError("--blend-alpha must be in [0, 1]")
    if args.prior_scale <= 0.0:
        raise ValueError("--prior-scale must be positive")
    if args.min_prior_step_m < 0.0:
        raise ValueError("--min-prior-step-m must be non-negative")
    if args.min_visual_step_m < 0.0:
        raise ValueError("--min-visual-step-m must be non-negative")
    if args.max_length_ratio < args.min_length_ratio:
        raise ValueError("--max-length-ratio must be >= --min-length-ratio")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    fill_default_outputs(args)
    validate_args(args)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    result, sim_rows, quality_rows = run_application(args)
    write_application_csv(args.csv_out, quality_rows)
    summary = format_markdown(args, result, sim_rows)
    args.summary_out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_out.write_text(summary, encoding="utf-8")
    print(summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
