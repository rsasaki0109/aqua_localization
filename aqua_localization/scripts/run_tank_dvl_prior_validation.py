#!/usr/bin/env python3
"""Validate a Tank DVL prior profile on a declared held-out sequence."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import apply_tank_dvl_motion_prior
import tank_dvl_prior_profile
import trajectory_benchmark_row


@dataclass(frozen=True)
class ValidationMetadata:
    profile_label: str
    calibration_sequence: str
    profile_validation_sequence: str
    sequence: str
    same_sequence_allowed: bool
    profile_sequence_mismatch_allowed: bool


@dataclass(frozen=True)
class PriorQualitySummary:
    steps: int
    dvl_covered_steps: int
    dvl_coverage_ratio: float
    prior_applied_steps: int
    prior_applied_ratio: float
    prior_match_accepted_steps: int
    prior_match_accepted_ratio: float
    mean_prior_match_confidence: float
    mean_applied_prior_confidence: float
    mean_effective_blend_alpha: float
    dominant_prior_reject_reason: str


def validate_sequence_split(args, profile: dict) -> ValidationMetadata:
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
    return ValidationMetadata(
        profile_label=label,
        calibration_sequence=calibration_sequence,
        profile_validation_sequence=profile_validation_sequence,
        sequence=args.sequence,
        same_sequence_allowed=args.allow_same_sequence,
        profile_sequence_mismatch_allowed=args.allow_profile_sequence_mismatch,
    )


def application_args(args):
    argv = [
        "--profile",
        str(args.profile),
        "--bag",
        str(args.bag),
        "--reference",
        str(args.reference),
        "--visual",
        str(args.visual),
        "--out-dir",
        str(args.out_dir / "application"),
    ]
    if args.csv_out is not None:
        argv.extend(["--csv-out", str(args.csv_out)])
    if args.corrected_out is not None:
        argv.extend(["--corrected-out", str(args.corrected_out)])
    if args.aligned_visual_out is not None:
        argv.extend(["--aligned-visual-out", str(args.aligned_visual_out)])
    if args.dvl_prior_out is not None:
        argv.extend(["--dvl-prior-out", str(args.dvl_prior_out)])
    app_args = apply_tank_dvl_motion_prior.parse_args(argv)
    apply_tank_dvl_motion_prior.fill_default_outputs(app_args)
    apply_tank_dvl_motion_prior.validate_args(app_args)
    return app_args


def finite_mean(values) -> float:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return math.nan
    return sum(finite) / len(finite)


def ratio(numerator: int, denominator: int) -> float:
    if denominator <= 0:
        return math.nan
    return numerator / denominator


def dominant_reject_reason(quality_rows: list[dict]) -> str:
    counts: dict[str, int] = {}
    for row in quality_rows:
        reason = str(row.get("prior_reject_reason", ""))
        if not reason or reason == "accepted":
            continue
        counts[reason] = counts.get(reason, 0) + 1
    if not counts:
        return "none"
    return max(counts.items(), key=lambda item: (item[1], item[0]))[0]


def summarize_prior_quality(quality_rows: list[dict], result) -> PriorQualitySummary:
    steps = int(getattr(result, "steps", len(quality_rows)))
    dvl_covered_steps = sum(1 for row in quality_rows if bool(row.get("dvl_covered")))
    prior_applied_steps = sum(1 for row in quality_rows if bool(row.get("used_prior")))
    accepted_steps = sum(
        1 for row in quality_rows if bool(row.get("prior_confidence_accepted"))
    )
    applied_rows = [row for row in quality_rows if bool(row.get("used_prior"))]
    return PriorQualitySummary(
        steps=steps,
        dvl_covered_steps=dvl_covered_steps,
        dvl_coverage_ratio=ratio(dvl_covered_steps, steps),
        prior_applied_steps=prior_applied_steps,
        prior_applied_ratio=ratio(prior_applied_steps, steps),
        prior_match_accepted_steps=accepted_steps,
        prior_match_accepted_ratio=ratio(accepted_steps, steps),
        mean_prior_match_confidence=finite_mean(
            row.get("prior_match_confidence", math.nan) for row in quality_rows
        ),
        mean_applied_prior_confidence=finite_mean(
            row.get("prior_confidence", math.nan) for row in applied_rows
        ),
        mean_effective_blend_alpha=finite_mean(
            row.get("effective_blend_alpha", math.nan) for row in quality_rows
        ),
        dominant_prior_reject_reason=dominant_reject_reason(quality_rows),
    )


def ratio_failure(label: str, value: float, minimum: float | None) -> str | None:
    if minimum is None:
        return None
    if not math.isfinite(value):
        return f"{label} n/a is below minimum {minimum:.3f}"
    if value < minimum:
        return f"{label} {value:.3f} is below minimum {minimum:.3f}"
    return None


def validation_status(
    args,
    result,
    quality: PriorQualitySummary | None = None,
) -> tuple[str, list[str]]:
    failures = []
    if args.max_corrected_rmse_m is not None and result.corrected_rmse_m > args.max_corrected_rmse_m:
        failures.append(
            f"corrected RMSE {result.corrected_rmse_m:.4f} m exceeds {args.max_corrected_rmse_m:.4f} m"
        )
    if (
        args.min_improvement_percent is not None
        and result.rmse_improvement_percent < args.min_improvement_percent
    ):
        failures.append(
            f"improvement {result.rmse_improvement_percent:.1f}% is below {args.min_improvement_percent:.1f}%"
        )
    if quality is not None:
        for failure in (
            ratio_failure(
                "DVL coverage ratio",
                quality.dvl_coverage_ratio,
                getattr(args, "min_dvl_coverage_ratio", None),
            ),
            ratio_failure(
                "prior-applied ratio",
                quality.prior_applied_ratio,
                getattr(args, "min_prior_applied_ratio", None),
            ),
            ratio_failure(
                "mean prior-match confidence",
                quality.mean_prior_match_confidence,
                getattr(args, "min_prior_match_confidence", None),
            ),
            ratio_failure(
                "mean applied-prior confidence",
                quality.mean_applied_prior_confidence,
                getattr(args, "min_applied_prior_confidence", None),
            ),
        ):
            if failure is not None:
                failures.append(failure)
    return ("FAIL" if failures else "PASS"), failures


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def format_markdown(
    args,
    metadata: ValidationMetadata,
    result,
    failures: list[str],
    app_args,
    quality: PriorQualitySummary | None = None,
) -> str:
    status = "FAIL" if failures else "PASS"
    lines = [
        "# Tank DVL Prior Held-Out Validation",
        "",
        f"- Status: `{status}`",
        f"- Sequence: `{metadata.sequence}`",
        f"- Profile: `{args.profile}`",
        f"- Profile label: `{metadata.profile_label}`",
        f"- Calibration sequence: `{metadata.calibration_sequence}`",
        f"- Profile validation sequence: `{metadata.profile_validation_sequence}`",
        f"- Same-sequence override: `{metadata.same_sequence_allowed}`",
        f"- Profile sequence mismatch override: `{metadata.profile_sequence_mismatch_allowed}`",
        f"- Bag: `{args.bag}`",
        f"- Reference: `{args.reference}`",
        f"- Visual estimate: `{args.visual}`",
        f"- Original RMSE: {result.original_rmse_m:.4f} m",
        f"- Corrected RMSE: {result.corrected_rmse_m:.4f} m",
        f"- RMSE improvement: {format_float(result.rmse_improvement_percent, 1)}%",
        f"- DVL-covered steps: {result.covered_steps}/{result.steps}",
        f"- Prior-applied steps: {result.prior_steps}/{result.steps}",
        f"- Corrected TUM: `{result.corrected_tum}`",
        f"- Aligned visual TUM: `{result.aligned_visual_tum}`",
        f"- DVL prior TUM: `{result.dvl_prior_tum}`",
        f"- Step CSV: `{app_args.csv_out}`",
        f"- Benchmark row: `{args.benchmark_row_out}`",
    ]
    if quality is not None:
        lines.extend([
            "",
            "## Prior Quality",
            "",
            f"- DVL coverage ratio: {format_float(quality.dvl_coverage_ratio, 3)} "
            f"({quality.dvl_covered_steps}/{quality.steps})",
            f"- Prior-applied ratio: {format_float(quality.prior_applied_ratio, 3)} "
            f"({quality.prior_applied_steps}/{quality.steps})",
            f"- Prior-match accepted ratio: "
            f"{format_float(quality.prior_match_accepted_ratio, 3)} "
            f"({quality.prior_match_accepted_steps}/{quality.steps})",
            f"- Mean prior-match confidence: "
            f"{format_float(quality.mean_prior_match_confidence, 3)}",
            f"- Mean applied-prior confidence: "
            f"{format_float(quality.mean_applied_prior_confidence, 3)}",
            f"- Mean effective blend alpha: "
            f"{format_float(quality.mean_effective_blend_alpha, 3)}",
            f"- Dominant prior reject reason: `{quality.dominant_prior_reject_reason}`",
        ])
    lines.extend(["", "## Gates", ""])
    lines.append(f"- Max corrected RMSE m: `{args.max_corrected_rmse_m}`")
    lines.append(f"- Min improvement percent: `{args.min_improvement_percent}`")
    lines.append(
        f"- Min DVL coverage ratio: `{getattr(args, 'min_dvl_coverage_ratio', None)}`"
    )
    lines.append(
        f"- Min prior-applied ratio: `{getattr(args, 'min_prior_applied_ratio', None)}`"
    )
    lines.append(
        f"- Min prior-match confidence: "
        f"`{getattr(args, 'min_prior_match_confidence', None)}`"
    )
    lines.append(
        f"- Min applied-prior confidence: "
        f"`{getattr(args, 'min_applied_prior_confidence', None)}`"
    )
    lines.extend(["", "## Failures", ""])
    if failures:
        for failure in failures:
            lines.append(f"- {failure}")
    else:
        lines.append("- none")
    lines.extend(["", "## Interpretation", ""])
    if failures:
        lines.append("- This profile did not pass the configured validation gates on the declared sequence.")
    elif metadata.same_sequence_allowed or metadata.profile_sequence_mismatch_allowed:
        lines.append("- Validation gates passed, but an override was used; treat this as a diagnostic smoke run.")
    else:
        lines.append("- Validation gates passed without sequence overrides; this row is a benchmark candidate.")
    lines.append("")
    return "\n".join(lines)


def benchmark_note(args, metadata: ValidationMetadata, result, failures: list[str]) -> str:
    if args.note:
        return args.note
    parts = [
        f"profile={metadata.profile_label}",
        f"prior={result.prior_steps}/{result.steps}",
        f"status={'FAIL' if failures else 'PASS'}",
    ]
    if metadata.same_sequence_allowed or metadata.profile_sequence_mismatch_allowed:
        parts.append("diagnostic override")
    else:
        parts.append("held-out validation")
    return "; ".join(parts)


def write_benchmark_row(args, metadata: ValidationMetadata, result, failures: list[str]) -> str:
    compare = trajectory_benchmark_row.load_compare_module()
    stats, _errors = compare.compare(args.reference, result.corrected_tum, with_scale=False, no_align=False)
    row_args = SimpleNamespace(
        dataset=args.dataset,
        sequence=metadata.sequence,
        system=args.system,
        note=benchmark_note(args, metadata, result, failures),
    )
    parts = []
    if not args.no_benchmark_row_header:
        parts.append(trajectory_benchmark_row.table_header())
    parts.append(trajectory_benchmark_row.format_row(row_args, stats))
    text = "\n".join(parts)
    trajectory_benchmark_row.write_output(args.benchmark_row_out, text, args.benchmark_row_append)
    return text


def run_validation(args):
    profile = tank_dvl_prior_profile.load_profile(args.profile)
    metadata = validate_sequence_split(args, profile)
    app_args = application_args(args)
    result, _sim_rows, quality_rows = apply_tank_dvl_motion_prior.run_application(app_args)
    apply_tank_dvl_motion_prior.write_application_csv(app_args.csv_out, quality_rows)
    quality = summarize_prior_quality(quality_rows, result)
    _status, failures = validation_status(args, result, quality)
    write_benchmark_row(args, metadata, result, failures)
    summary = format_markdown(args, metadata, result, failures, app_args, quality)
    args.summary_out.parent.mkdir(parents=True, exist_ok=True)
    args.summary_out.write_text(summary, encoding="utf-8")
    return result, failures, summary


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Run held-out validation for a Tank DVL prior profile."
    )
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--visual", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_dvl_prior_validation"))
    parser.add_argument("--summary-out", type=Path)
    parser.add_argument("--csv-out", type=Path)
    parser.add_argument("--corrected-out", type=Path)
    parser.add_argument("--aligned-visual-out", type=Path)
    parser.add_argument("--dvl-prior-out", type=Path)
    parser.add_argument("--benchmark-row-out", type=Path)
    parser.add_argument("--benchmark-row-append", action="store_true")
    parser.add_argument("--no-benchmark-row-header", action="store_true")
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--system", default="aqua_dvl_prior_visual")
    parser.add_argument("--note", default="")
    parser.add_argument("--allow-same-sequence", action="store_true")
    parser.add_argument("--allow-profile-sequence-mismatch", action="store_true")
    parser.add_argument("--max-corrected-rmse-m", type=float)
    parser.add_argument("--min-improvement-percent", type=float)
    parser.add_argument("--min-dvl-coverage-ratio", type=float)
    parser.add_argument("--min-prior-applied-ratio", type=float)
    parser.add_argument("--min-prior-match-confidence", type=float)
    parser.add_argument("--min-applied-prior-confidence", type=float)
    parser.add_argument("--fail-on-gate-failure", action="store_true")
    return parser.parse_args(argv)


def fill_default_outputs(args) -> None:
    if args.summary_out is None:
        args.summary_out = args.out_dir / "tank_dvl_prior_validation.md"
    if args.csv_out is None:
        args.csv_out = args.out_dir / "tank_dvl_prior_validation_steps.csv"
    if args.corrected_out is None:
        args.corrected_out = args.out_dir / "tank_dvl_prior_validation_corrected.tum"
    if args.aligned_visual_out is None:
        args.aligned_visual_out = args.out_dir / "aligned_visual_input.tum"
    if args.dvl_prior_out is None:
        args.dvl_prior_out = args.out_dir / "dvl_motion_prior.tum"
    if args.benchmark_row_out is None:
        args.benchmark_row_out = args.out_dir / "tank_dvl_prior_benchmark_row.md"


def validate_args(args) -> None:
    if args.max_corrected_rmse_m is not None and args.max_corrected_rmse_m <= 0.0:
        raise ValueError("--max-corrected-rmse-m must be positive")
    if args.min_improvement_percent is not None and args.min_improvement_percent < 0.0:
        raise ValueError("--min-improvement-percent must be non-negative")
    for name in (
        "min_dvl_coverage_ratio",
        "min_prior_applied_ratio",
        "min_prior_match_confidence",
        "min_applied_prior_confidence",
    ):
        value = getattr(args, name)
        if value is not None and not 0.0 <= value <= 1.0:
            raise ValueError(f"--{name.replace('_', '-')} must be in [0, 1]")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    fill_default_outputs(args)
    try:
        validate_args(args)
        args.out_dir.mkdir(parents=True, exist_ok=True)
        _result, failures, summary = run_validation(args)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(summary)
    return 1 if failures and args.fail_on_gate_failure else 0


if __name__ == "__main__":
    sys.exit(main())
