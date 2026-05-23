#!/usr/bin/env python3
"""Sweep Tank DVL prior application gates without re-reading the bag each run."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

import apply_tank_dvl_motion_prior as dvl_apply
import promote_tank_dvl_sweep_profile
import simulate_visual_motion_prior as prior_sim
import tank_dvl_prior_profile


@dataclass(frozen=True)
class SweepMetadata:
    profile_label: str
    calibration_sequence: str
    profile_validation_sequence: str
    sequence: str
    same_sequence_allowed: bool
    profile_sequence_mismatch_allowed: bool


@dataclass(frozen=True)
class SweepRow:
    rank: int
    mode: str
    blend_alpha: float
    prior_scale: float
    min_length_ratio: float
    max_length_ratio: float
    min_direction_cosine: float
    corrected_rmse_m: float
    mean_error_m: float
    median_error_m: float
    max_error_m: float
    improvement_percent: float
    gap_to_baseline_x: float
    prior_steps: int
    steps: int
    dvl_covered_steps: int
    prior_match_accepted_steps: int
    mean_prior_match_confidence: float
    mean_applied_prior_confidence: float
    mean_effective_blend_alpha: float
    dominant_prior_reject_reason: str
    samples: int
    matched_s: float


@dataclass(frozen=True)
class CandidateOutput:
    row: SweepRow
    corrected_xyz: np.ndarray
    prior_xyz: np.ndarray
    sim_rows: list[prior_sim.PriorStep]
    quality_rows: list[dict]


def parse_float_list(text: str) -> list[float]:
    values = []
    for item in str(text).split(","):
        item = item.strip()
        if not item:
            continue
        values.append(float(item))
    if not values:
        raise ValueError("expected at least one numeric value")
    return values


def parse_mode_list(text: str) -> list[str]:
    modes = [item.strip() for item in str(text).split(",") if item.strip()]
    valid = set(prior_sim.APPLICATION_MODES)
    invalid = [mode for mode in modes if mode not in valid]
    if invalid:
        raise ValueError(f"unsupported mode(s): {', '.join(invalid)}")
    if not modes:
        raise ValueError("expected at least one mode")
    return modes


def validate_sequence_split(args, profile: dict) -> SweepMetadata:
    label = tank_dvl_prior_profile.profile_label(args.profile, profile)
    metadata = profile.get("metadata", {}) if isinstance(profile, dict) else {}
    if not isinstance(metadata, dict):
        metadata = {}
    calibration_sequence = str(metadata.get("calibration_sequence", "") or "")
    profile_validation_sequence = str(metadata.get("validation_sequence", "") or "")
    if calibration_sequence and args.sequence == calibration_sequence and not args.allow_same_sequence:
        raise ValueError(
            f"--sequence {args.sequence!r} matches profile calibration_sequence; "
            "use --allow-same-sequence only for diagnostics"
        )
    if (
        profile_validation_sequence
        and args.sequence != profile_validation_sequence
        and not args.allow_profile_sequence_mismatch
    ):
        raise ValueError(
            f"--sequence {args.sequence!r} does not match profile validation_sequence "
            f"{profile_validation_sequence!r}; use --allow-profile-sequence-mismatch only for diagnostics"
        )
    return SweepMetadata(
        profile_label=label,
        calibration_sequence=calibration_sequence,
        profile_validation_sequence=profile_validation_sequence,
        sequence=args.sequence,
        same_sequence_allowed=args.allow_same_sequence,
        profile_sequence_mismatch_allowed=args.allow_profile_sequence_mismatch,
    )


def aligned_error_stats(compare, times: np.ndarray, reference_xyz: np.ndarray, estimate_xyz: np.ndarray) -> dict:
    rotation, translation, scale = compare.umeyama_alignment(
        estimate_xyz,
        reference_xyz,
        with_scale=False,
    )
    aligned = compare.apply_transform(estimate_xyz, rotation, translation, scale)
    errors = np.linalg.norm(aligned - reference_xyz, axis=1)
    return {
        "samples": int(errors.shape[0]),
        "matched_s": float(times[-1] - times[0]) if times.shape[0] else math.nan,
        "mean": float(errors.mean()),
        "median": float(np.median(errors)),
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "max": float(errors.max()),
    }


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def finite_mean(values: list[float]) -> float:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return math.nan
    return sum(finite) / len(finite)


def dominant_reason(reasons: list[str]) -> str:
    counts: dict[str, int] = {}
    for reason in reasons:
        if not reason or reason == "accepted":
            continue
        counts[reason] = counts.get(reason, 0) + 1
    if not counts:
        return "none"
    return max(counts.items(), key=lambda item: (item[1], item[0]))[0]


def summarize_quality_rows(quality_rows: list[dict]) -> dict:
    applied_rows = [row for row in quality_rows if bool(row["used_prior"])]
    return {
        "dvl_covered_steps": sum(1 for row in quality_rows if bool(row["dvl_covered"])),
        "prior_match_accepted_steps": sum(
            1 for row in quality_rows if bool(row["prior_confidence_accepted"])
        ),
        "mean_prior_match_confidence": finite_mean([
            float(row["prior_match_confidence"]) for row in quality_rows
        ]),
        "mean_applied_prior_confidence": finite_mean([
            float(row["prior_confidence"]) for row in applied_rows
        ]),
        "mean_effective_blend_alpha": finite_mean([
            float(row["effective_blend_alpha"]) for row in quality_rows
        ]),
        "dominant_prior_reject_reason": dominant_reason([
            str(row["prior_reject_reason"]) for row in quality_rows
        ]),
    }


def attach_quality_summary(row: SweepRow, quality_rows: list[dict]) -> SweepRow:
    return replace(row, **summarize_quality_rows(quality_rows))


def make_sweep_row(
    *,
    rank: int,
    mode: str,
    blend_alpha: float,
    prior_scale: float,
    min_length_ratio: float,
    max_length_ratio: float,
    min_direction_cosine: float,
    corrected_stats: dict,
    original_rmse_m: float,
    baseline_rmse_m: float | None,
    prior_steps: int,
    steps: int,
) -> SweepRow:
    corrected_rmse = float(corrected_stats["rmse"])
    improvement = 100.0 * (original_rmse_m - corrected_rmse) / original_rmse_m if original_rmse_m > 0.0 else math.nan
    gap = corrected_rmse / baseline_rmse_m if baseline_rmse_m and baseline_rmse_m > 0.0 else math.nan
    return SweepRow(
        rank=rank,
        mode=mode,
        blend_alpha=blend_alpha,
        prior_scale=prior_scale,
        min_length_ratio=min_length_ratio,
        max_length_ratio=max_length_ratio,
        min_direction_cosine=min_direction_cosine,
        corrected_rmse_m=corrected_rmse,
        mean_error_m=float(corrected_stats["mean"]),
        median_error_m=float(corrected_stats["median"]),
        max_error_m=float(corrected_stats["max"]),
        improvement_percent=improvement,
        gap_to_baseline_x=gap,
        prior_steps=prior_steps,
        steps=steps,
        dvl_covered_steps=0,
        prior_match_accepted_steps=0,
        mean_prior_match_confidence=math.nan,
        mean_applied_prior_confidence=math.nan,
        mean_effective_blend_alpha=math.nan,
        dominant_prior_reject_reason="",
        samples=int(corrected_stats["samples"]),
        matched_s=float(corrected_stats["matched_s"]),
    )


def evaluate_candidate(
    compare,
    times: np.ndarray,
    reference_xyz: np.ndarray,
    visual_xyz: np.ndarray,
    prior_xyz: np.ndarray,
    *,
    mode: str,
    blend_alpha: float,
    prior_scale: float,
    min_prior_step_m: float,
    min_length_ratio: float,
    max_length_ratio: float,
    min_direction_cosine: float,
    original_rmse_m: float,
    baseline_rmse_m: float | None,
    rank: int = 0,
) -> tuple[SweepRow, np.ndarray, list[prior_sim.PriorStep]]:
    corrected_xyz, sim_rows = prior_sim.simulate_prior(
        times,
        visual_xyz,
        prior_xyz,
        mode=mode,
        blend_alpha=blend_alpha,
        min_reference_step_m=min_prior_step_m,
        min_length_ratio=min_length_ratio,
        max_length_ratio=max_length_ratio,
        min_direction_cosine=min_direction_cosine,
    )
    corrected_stats = aligned_error_stats(compare, times, reference_xyz, corrected_xyz)
    row = make_sweep_row(
        rank=rank,
        mode=mode,
        blend_alpha=blend_alpha,
        prior_scale=prior_scale,
        min_length_ratio=min_length_ratio,
        max_length_ratio=max_length_ratio,
        min_direction_cosine=min_direction_cosine,
        corrected_stats=corrected_stats,
        original_rmse_m=original_rmse_m,
        baseline_rmse_m=baseline_rmse_m,
        prior_steps=sum(1 for step in sim_rows if step.used_prior),
        steps=len(sim_rows),
    )
    return row, corrected_xyz, sim_rows


def rank_rows(rows: list[SweepRow]) -> list[SweepRow]:
    ranked = sorted(rows, key=lambda row: row.corrected_rmse_m)
    return [
        SweepRow(
            rank=index,
            mode=row.mode,
            blend_alpha=row.blend_alpha,
            prior_scale=row.prior_scale,
            min_length_ratio=row.min_length_ratio,
            max_length_ratio=row.max_length_ratio,
            min_direction_cosine=row.min_direction_cosine,
            corrected_rmse_m=row.corrected_rmse_m,
            mean_error_m=row.mean_error_m,
            median_error_m=row.median_error_m,
            max_error_m=row.max_error_m,
            improvement_percent=row.improvement_percent,
            gap_to_baseline_x=row.gap_to_baseline_x,
            prior_steps=row.prior_steps,
            steps=row.steps,
            dvl_covered_steps=row.dvl_covered_steps,
            prior_match_accepted_steps=row.prior_match_accepted_steps,
            mean_prior_match_confidence=row.mean_prior_match_confidence,
            mean_applied_prior_confidence=row.mean_applied_prior_confidence,
            mean_effective_blend_alpha=row.mean_effective_blend_alpha,
            dominant_prior_reject_reason=row.dominant_prior_reject_reason,
            samples=row.samples,
            matched_s=row.matched_s,
        )
        for index, row in enumerate(ranked, start=1)
    ]


def sweep_candidates(args, metadata: SweepMetadata) -> tuple[list[SweepRow], CandidateOutput]:
    compare = dvl_apply.load_compare_module()
    times, reference_xyz, aligned_visual_xyz, reference_tum = dvl_apply.matched_reference_and_aligned_visual(
        args.reference,
        args.visual,
    )
    original_stats = aligned_error_stats(compare, times, reference_xyz, aligned_visual_xyz)
    original_rmse = float(original_stats["rmse"])
    dvl_records = dvl_apply.read_dvl_records(args.bag, args.dvl_topic)
    imu_records = (
        dvl_apply.read_imu_yaw_records(args.bag, args.imu_topic)
        if args.dvl_yaw_mode == "imu_yaw"
        else None
    )

    modes = parse_mode_list(args.modes)
    blend_alphas = parse_float_list(args.blend_alphas)
    prior_scales = parse_float_list(args.prior_scales)
    min_length_ratios = parse_float_list(args.min_length_ratios)
    max_length_ratios = parse_float_list(args.max_length_ratios)
    min_direction_cosines = parse_float_list(args.min_direction_cosines)

    rows = []
    outputs: list[CandidateOutput] = []
    for prior_scale in prior_scales:
        deltas = dvl_apply.build_dvl_prior_deltas(
            times,
            dvl_records,
            reference_tum,
            args.dvl_yaw_mode,
            math.radians(args.dvl_frame_yaw_offset_deg),
            imu_records,
            math.radians(args.imu_yaw_offset_deg),
            prior_scale,
        )
        prior_xyz = dvl_apply.positions_from_deltas(times, aligned_visual_xyz[0], deltas)
        for mode in modes:
            alphas = (
                [blend_alphas[0]]
                if mode in {"replace-outliers", "confidence-replace-outliers"}
                else blend_alphas
            )
            for blend_alpha in alphas:
                for min_length_ratio in min_length_ratios:
                    for max_length_ratio in max_length_ratios:
                        if max_length_ratio < min_length_ratio:
                            continue
                        for min_direction_cosine in min_direction_cosines:
                            row, corrected_xyz, sim_rows = evaluate_candidate(
                                compare,
                                times,
                                reference_xyz,
                                aligned_visual_xyz,
                                prior_xyz,
                                mode=mode,
                                blend_alpha=blend_alpha,
                                prior_scale=prior_scale,
                                min_prior_step_m=args.min_prior_step_m,
                                min_length_ratio=min_length_ratio,
                                max_length_ratio=max_length_ratio,
                                min_direction_cosine=min_direction_cosine,
                                original_rmse_m=original_rmse,
                                baseline_rmse_m=args.baseline_rmse_m,
                            )
                            quality_rows = dvl_apply.prior_step_quality_rows(
                                times,
                                aligned_visual_xyz,
                                prior_xyz,
                                deltas,
                                sim_rows,
                                min_prior_step_m=args.min_prior_step_m,
                                min_length_ratio=min_length_ratio,
                                max_length_ratio=max_length_ratio,
                                min_direction_cosine=min_direction_cosine,
                            )
                            row = attach_quality_summary(row, quality_rows)
                            rows.append(row)
                            outputs.append(CandidateOutput(row, corrected_xyz, prior_xyz, sim_rows, quality_rows))

    ranked = rank_rows(rows)
    rank_by_key = {
        (
            row.mode,
            row.blend_alpha,
            row.prior_scale,
            row.min_length_ratio,
            row.max_length_ratio,
            row.min_direction_cosine,
        ): row
        for row in ranked
    }
    ranked_outputs = []
    for output in outputs:
        key = (
            output.row.mode,
            output.row.blend_alpha,
            output.row.prior_scale,
            output.row.min_length_ratio,
            output.row.max_length_ratio,
            output.row.min_direction_cosine,
        )
        ranked_outputs.append(CandidateOutput(
            rank_by_key[key],
            output.corrected_xyz,
            output.prior_xyz,
            output.sim_rows,
            output.quality_rows,
        ))
    best = min(ranked_outputs, key=lambda output: output.row.corrected_rmse_m)

    prior_sim.write_tum(args.best_corrected_out, times, best.corrected_xyz)
    prior_sim.write_tum(args.best_dvl_prior_out, times, best.prior_xyz)
    prior_sim.write_tum(args.aligned_visual_out, times, aligned_visual_xyz)
    dvl_apply.write_application_csv(args.best_step_csv_out, best.quality_rows)
    return ranked, best


def write_csv(path: Path, rows: list[SweepRow]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [field.name for field in SweepRow.__dataclass_fields__.values()]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(sweep_row_dict(row))


def sweep_row_dict(row: SweepRow) -> dict:
    return {
        field.name: getattr(row, field.name)
        for field in SweepRow.__dataclass_fields__.values()
    }


def write_best_profile(args, best: CandidateOutput) -> dict | None:
    if args.best_profile_out is None:
        return None
    promote_args = SimpleNamespace(
        base_profile=args.profile,
        sweep_csv=args.csv_out,
        rank=best.row.rank,
        name=args.best_profile_name,
        validation_sequence=args.best_profile_validation_sequence,
        note=args.best_profile_note,
    )
    profile = promote_tank_dvl_sweep_profile.promote_profile(
        args.profile_data,
        sweep_row_dict(best.row),
        promote_args,
    )
    tank_dvl_prior_profile.write_profile(args.best_profile_out, profile)
    return profile


def format_markdown(args, metadata: SweepMetadata, rows: list[SweepRow], best: CandidateOutput) -> str:
    shown = rows[: args.top_k]
    lines = [
        "# Tank DVL Prior Gate Sweep",
        "",
        f"- Sequence: `{metadata.sequence}`",
        f"- Profile: `{args.profile}`",
        f"- Profile label: `{metadata.profile_label}`",
        f"- Calibration sequence: `{metadata.calibration_sequence}`",
        f"- Profile validation sequence: `{metadata.profile_validation_sequence}`",
        f"- Same-sequence override: `{metadata.same_sequence_allowed}`",
        f"- Profile sequence mismatch override: `{metadata.profile_sequence_mismatch_allowed}`",
        f"- Candidates evaluated: {len(rows)}",
        f"- Best corrected RMSE: {best.row.corrected_rmse_m:.4f} m",
        f"- Best improvement: {format_float(best.row.improvement_percent, 1)}%",
        f"- Best DVL coverage: {best.row.dvl_covered_steps}/{best.row.steps}",
        f"- Best prior steps: {best.row.prior_steps}/{best.row.steps}",
        f"- Best accepted prior matches: {best.row.prior_match_accepted_steps}/{best.row.steps}",
        f"- Best mean prior match confidence: {format_float(best.row.mean_prior_match_confidence, 3)}",
        f"- Best mean applied prior confidence: {format_float(best.row.mean_applied_prior_confidence, 3)}",
        f"- Best mean effective alpha: {format_float(best.row.mean_effective_blend_alpha, 3)}",
        f"- Best dominant reject reason: `{best.row.dominant_prior_reject_reason}`",
        f"- Sweep CSV: `{args.csv_out}`",
        f"- Best corrected TUM: `{args.best_corrected_out}`",
        f"- Best DVL prior TUM: `{args.best_dvl_prior_out}`",
        f"- Best step CSV: `{args.best_step_csv_out}`",
    ]
    if args.best_profile_out is not None:
        lines.append(f"- Best promoted profile: `{args.best_profile_out}`")
    if args.baseline_rmse_m is not None:
        lines.append(f"- Best baseline gap: {format_float(best.row.gap_to_baseline_x, 2)}x")
    lines.extend([
        "",
        "## Top Candidates",
        "",
        (
            "| Rank | Mode | Alpha | Scale | Min ratio | Max ratio | Min cosine | RMSE m | "
            "Improvement | Gap x | Covered | Prior | Match ok | Match conf | Eff alpha | Dominant reject |"
        ),
        (
            "|-----:|------|------:|------:|----------:|----------:|-----------:|-------:|"
            "------------:|------:|--------:|------:|---------:|-----------:|----------:|----------------|"
        ),
    ])
    for row in shown:
        lines.append(
            f"| {row.rank} | {row.mode} | {format_float(row.blend_alpha, 3)} | "
            f"{format_float(row.prior_scale, 5)} | {format_float(row.min_length_ratio, 3)} | "
            f"{format_float(row.max_length_ratio, 3)} | {format_float(row.min_direction_cosine, 3)} | "
            f"{row.corrected_rmse_m:.4f} | {format_float(row.improvement_percent, 1)}% | "
            f"{format_float(row.gap_to_baseline_x, 2)} | {row.dvl_covered_steps}/{row.steps} | "
            f"{row.prior_steps}/{row.steps} | {row.prior_match_accepted_steps}/{row.steps} | "
            f"{format_float(row.mean_prior_match_confidence, 3)} | "
            f"{format_float(row.mean_effective_blend_alpha, 3)} | "
            f"{row.dominant_prior_reject_reason} |"
        )
    lines.extend(["", "## Interpretation", ""])
    if metadata.same_sequence_allowed or metadata.profile_sequence_mismatch_allowed:
        lines.append("- An override was used; treat this sweep as diagnostic tuning evidence, not a benchmark claim.")
    else:
        lines.append("- No sequence override was used; the best row is a held-out validation candidate.")
    if args.baseline_rmse_m is not None and best.row.gap_to_baseline_x < 1.0:
        lines.append("- The best row is below the supplied baseline RMSE; confirm on held-out data before claiming a win.")
    elif args.baseline_rmse_m is not None:
        lines.append("- The best row is still above the supplied baseline RMSE.")
    lines.append("")
    return "\n".join(lines)


def parse_args(argv):
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--profile", type=Path)
    pre_args, _ = pre_parser.parse_known_args(argv)
    defaults = dict(dvl_apply.PROFILE_DEFAULTS)
    profile = {}
    if pre_args.profile is not None:
        profile = tank_dvl_prior_profile.load_profile(pre_args.profile)
        defaults.update(tank_dvl_prior_profile.profile_arg_defaults(pre_args.profile))

    parser = argparse.ArgumentParser(
        description="Sweep Tank DVL prior application gates on one declared sequence."
    )
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--visual", required=True, type=Path)
    parser.add_argument("--dvl-topic", default=dvl_apply.DEFAULT_DVL_TOPIC)
    parser.add_argument("--imu-topic", default=dvl_apply.DEFAULT_IMU_TOPIC)
    parser.add_argument("--dvl-yaw-mode", choices=["body_raw", "gt_yaw", "imu_yaw"], default=defaults["dvl_yaw_mode"])
    parser.add_argument("--dvl-frame-yaw-offset-deg", type=float, default=defaults["dvl_frame_yaw_offset_deg"])
    parser.add_argument("--imu-yaw-offset-deg", type=float, default=defaults["imu_yaw_offset_deg"])
    parser.add_argument("--prior-scales", default=str(defaults["prior_scale"]))
    parser.add_argument("--modes", default=str(defaults["mode"]))
    parser.add_argument("--blend-alphas", default=str(defaults["blend_alpha"]))
    parser.add_argument("--min-prior-step-m", type=float, default=defaults["min_prior_step_m"])
    parser.add_argument("--min-length-ratios", default=str(defaults["min_length_ratio"]))
    parser.add_argument("--max-length-ratios", default=str(defaults["max_length_ratio"]))
    parser.add_argument("--min-direction-cosines", default=str(defaults["min_direction_cosine"]))
    parser.add_argument("--allow-same-sequence", action="store_true")
    parser.add_argument("--allow-profile-sequence-mismatch", action="store_true")
    parser.add_argument("--baseline-rmse-m", type=float)
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_dvl_prior_gate_sweep"))
    parser.add_argument("--summary-out", type=Path)
    parser.add_argument("--csv-out", type=Path)
    parser.add_argument("--best-corrected-out", type=Path)
    parser.add_argument("--best-dvl-prior-out", type=Path)
    parser.add_argument("--aligned-visual-out", type=Path)
    parser.add_argument("--best-step-csv-out", type=Path)
    parser.add_argument(
        "--best-profile-out",
        type=Path,
        help="Optional YAML profile promoted from the best sweep row.",
    )
    parser.add_argument("--best-profile-name", default="")
    parser.add_argument("--best-profile-validation-sequence", default="")
    parser.add_argument("--best-profile-note", default="")
    args = parser.parse_args(argv)
    args.profile_data = profile
    return args


def fill_default_outputs(args) -> None:
    if args.summary_out is None:
        args.summary_out = args.out_dir / "tank_dvl_prior_gate_sweep.md"
    if args.csv_out is None:
        args.csv_out = args.out_dir / "tank_dvl_prior_gate_sweep.csv"
    if args.best_corrected_out is None:
        args.best_corrected_out = args.out_dir / "best_corrected.tum"
    if args.best_dvl_prior_out is None:
        args.best_dvl_prior_out = args.out_dir / "best_dvl_prior.tum"
    if args.aligned_visual_out is None:
        args.aligned_visual_out = args.out_dir / "aligned_visual_input.tum"
    if args.best_step_csv_out is None:
        args.best_step_csv_out = args.out_dir / "best_steps.csv"


def validate_args(args) -> None:
    if args.min_prior_step_m < 0.0:
        raise ValueError("--min-prior-step-m must be non-negative")
    if args.baseline_rmse_m is not None and args.baseline_rmse_m <= 0.0:
        raise ValueError("--baseline-rmse-m must be positive")
    if args.top_k < 0:
        raise ValueError("--top-k must be non-negative")
    for value in parse_float_list(args.prior_scales):
        if value <= 0.0:
            raise ValueError("--prior-scales values must be positive")
    for value in parse_float_list(args.blend_alphas):
        if not 0.0 <= value <= 1.0:
            raise ValueError("--blend-alphas values must be in [0, 1]")
    for value in parse_float_list(args.min_direction_cosines):
        if not -1.0 <= value <= 1.0:
            raise ValueError("--min-direction-cosines values must be in [-1, 1]")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    fill_default_outputs(args)
    try:
        validate_args(args)
        metadata = validate_sequence_split(args, args.profile_data)
        args.out_dir.mkdir(parents=True, exist_ok=True)
        rows, best = sweep_candidates(args, metadata)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    write_csv(args.csv_out, rows)
    write_best_profile(args, best)
    summary = format_markdown(args, metadata, rows, best)
    args.summary_out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_out.write_text(summary, encoding="utf-8")
    print(summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
