#!/usr/bin/env python3
"""Run Tank DVL prior validation plus gap and residual reports as one bundle."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import analyze_tank_dvl_prior_residuals as residual_analysis
import benchmark_gap_report
import run_tank_dvl_prior_validation as validation


@dataclass(frozen=True)
class BundlePaths:
    validation_summary: Path
    validation_steps_csv: Path
    corrected_tum: Path
    aligned_visual_tum: Path
    dvl_prior_tum: Path
    benchmark_row: Path
    gap_report: Path
    residual_report: Path
    residual_csv: Path
    bundle_summary: Path


@dataclass(frozen=True)
class BundleResult:
    validation_failures: list[str]
    gap_failures: list[str]
    validation_summary: str
    gap_summary: str
    residual_summary: str

    @property
    def ok(self) -> bool:
        return not self.validation_failures and not self.gap_failures


def default_benchmark_markdown() -> Path | None:
    candidate = Path(__file__).resolve().parents[2] / "docs" / "benchmarks" / "tank_aqua_slam.md"
    return candidate if candidate.exists() else None


def bundle_paths(out_dir: Path) -> BundlePaths:
    return BundlePaths(
        validation_summary=out_dir / "validation" / "tank_dvl_prior_validation.md",
        validation_steps_csv=out_dir / "validation" / "tank_dvl_prior_validation_steps.csv",
        corrected_tum=out_dir / "validation" / "tank_dvl_prior_validation_corrected.tum",
        aligned_visual_tum=out_dir / "validation" / "aligned_visual_input.tum",
        dvl_prior_tum=out_dir / "validation" / "dvl_motion_prior.tum",
        benchmark_row=out_dir / "validation" / "tank_dvl_prior_benchmark_row.md",
        gap_report=out_dir / "benchmark_gap_report.md",
        residual_report=out_dir / "tank_dvl_prior_residuals.md",
        residual_csv=out_dir / "tank_dvl_prior_residuals.csv",
        bundle_summary=out_dir / "tank_dvl_validation_bundle.md",
    )


def existing_input_paths(args) -> list[tuple[str, Path]]:
    paths = [
        ("profile", args.profile),
        ("bag", args.bag),
        ("reference", args.reference),
        ("visual", args.visual),
    ]
    if not args.skip_gap_report:
        for index, path in enumerate(args.benchmark_markdown or [], start=1):
            paths.append((f"benchmark_markdown[{index}]", path))
    return paths


def missing_inputs(args) -> list[str]:
    missing = []
    for label, path in existing_input_paths(args):
        if path is None:
            continue
        if not path.exists():
            missing.append(f"{label}: {path}")
    return missing


def make_validation_args(args, paths: BundlePaths):
    argv = [
        "--profile",
        str(args.profile),
        "--sequence",
        args.sequence,
        "--bag",
        str(args.bag),
        "--reference",
        str(args.reference),
        "--visual",
        str(args.visual),
        "--out-dir",
        str(paths.validation_summary.parent),
        "--summary-out",
        str(paths.validation_summary),
        "--csv-out",
        str(paths.validation_steps_csv),
        "--corrected-out",
        str(paths.corrected_tum),
        "--aligned-visual-out",
        str(paths.aligned_visual_tum),
        "--dvl-prior-out",
        str(paths.dvl_prior_tum),
        "--benchmark-row-out",
        str(paths.benchmark_row),
        "--dataset",
        args.dataset,
        "--system",
        args.target_system,
    ]
    if args.note:
        argv.extend(["--note", args.note])
    if args.allow_same_sequence:
        argv.append("--allow-same-sequence")
    if args.allow_profile_sequence_mismatch:
        argv.append("--allow-profile-sequence-mismatch")
    if args.max_corrected_rmse_m is not None:
        argv.extend(["--max-corrected-rmse-m", str(args.max_corrected_rmse_m)])
    if args.min_improvement_percent is not None:
        argv.extend(["--min-improvement-percent", str(args.min_improvement_percent)])
    return validation.parse_args(argv)


def run_validation(args, paths: BundlePaths):
    validation_args = make_validation_args(args, paths)
    validation.fill_default_outputs(validation_args)
    validation.validate_args(validation_args)
    validation_args.out_dir.mkdir(parents=True, exist_ok=True)
    result, failures, summary = validation.run_validation(validation_args)
    return validation_args, result, failures, summary


def run_gap_report(args, paths: BundlePaths) -> tuple[str, list[str]]:
    if args.skip_gap_report:
        return "Gap report skipped.\n", []
    rows = []
    for path in args.benchmark_markdown:
        rows.extend(benchmark_gap_report.parse_markdown_benchmark_rows(path.read_text(encoding="utf-8")))
    rows.extend(benchmark_gap_report.parse_markdown_benchmark_rows(paths.benchmark_row.read_text(encoding="utf-8")))
    gaps = benchmark_gap_report.compute_gaps(rows, args.target_system, args.baseline_system)
    report = benchmark_gap_report.format_report(gaps, args.target_system, args.baseline_system)
    paths.gap_report.parent.mkdir(parents=True, exist_ok=True)
    paths.gap_report.write_text(report + "\n", encoding="utf-8")
    failures = benchmark_gap_report.gate_failures(
        gaps,
        max_gap_x=args.max_gap_x,
        max_improvement_to_tie_percent=args.max_improvement_to_tie_percent,
    )
    return report, failures


def run_residual_report(args, paths: BundlePaths) -> str:
    if args.skip_residual_analysis:
        return "Residual analysis skipped.\n"
    residual_args = argparse.Namespace(
        reference=args.reference,
        visual_aligned=paths.aligned_visual_tum,
        corrected=paths.corrected_tum,
        step_csv=paths.validation_steps_csv,
        out=paths.residual_report,
        csv_out=paths.residual_csv,
        top_k=args.residual_top_k,
    )
    text, _rows = residual_analysis.run_analysis(residual_args)
    paths.residual_report.parent.mkdir(parents=True, exist_ok=True)
    paths.residual_report.write_text(text, encoding="utf-8")
    return text


def format_bundle_summary(args, paths: BundlePaths, result: BundleResult) -> str:
    status = "PASS" if result.ok else "FAIL"
    lines = [
        "# Tank DVL Validation Bundle",
        "",
        f"- Status: `{status}`",
        f"- Sequence: `{args.sequence}`",
        f"- Profile: `{args.profile}`",
        f"- Validation summary: `{paths.validation_summary}`",
        f"- Benchmark row: `{paths.benchmark_row}`",
        f"- Gap report: `{paths.gap_report}`",
        f"- Residual report: `{paths.residual_report}`",
        "",
        "## Failures",
        "",
    ]
    failures = result.validation_failures + result.gap_failures
    if failures:
        for failure in failures:
            lines.append(f"- {failure}")
    else:
        lines.append("- none")
    lines.extend(["", "## Interpretation", ""])
    if result.ok:
        lines.append("- Validation and configured benchmark gates passed.")
    else:
        lines.append("- One or more validation or benchmark gates failed; inspect the linked reports.")
    lines.append("")
    return "\n".join(lines)


def run_bundle(args) -> tuple[BundlePaths, BundleResult, str]:
    paths = bundle_paths(args.out_dir)
    _validation_args, _app_result, validation_failures, validation_summary = run_validation(args, paths)
    gap_summary, gap_failures = run_gap_report(args, paths)
    residual_summary = run_residual_report(args, paths)
    result = BundleResult(
        validation_failures=validation_failures if args.fail_on_gate_failure else [],
        gap_failures=gap_failures,
        validation_summary=validation_summary,
        gap_summary=gap_summary,
        residual_summary=residual_summary,
    )
    summary = format_bundle_summary(args, paths, result)
    paths.bundle_summary.parent.mkdir(parents=True, exist_ok=True)
    paths.bundle_summary.write_text(summary, encoding="utf-8")
    return paths, result, summary


def parse_args(argv):
    default_markdown = default_benchmark_markdown()
    parser = argparse.ArgumentParser(
        description="Run Tank DVL prior validation, benchmark gap, and residual reports."
    )
    parser.add_argument("--profile", required=True, type=Path)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--visual", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_dvl_validation_bundle"))
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--target-system", default="aqua_dvl_prior_visual")
    parser.add_argument("--baseline-system", default="AQUA-SLAM")
    parser.add_argument("--benchmark-markdown", action="append", type=Path, default=[])
    parser.add_argument("--skip-gap-report", action="store_true")
    parser.add_argument("--skip-residual-analysis", action="store_true")
    parser.add_argument("--allow-same-sequence", action="store_true")
    parser.add_argument("--allow-profile-sequence-mismatch", action="store_true")
    parser.add_argument("--max-corrected-rmse-m", type=float)
    parser.add_argument("--min-improvement-percent", type=float)
    parser.add_argument("--fail-on-gate-failure", action="store_true")
    parser.add_argument("--max-gap-x", type=float)
    parser.add_argument("--max-improvement-to-tie-percent", type=float)
    parser.add_argument("--residual-top-k", type=int, default=10)
    parser.add_argument("--note", default="")
    args = parser.parse_args(argv)
    if not args.benchmark_markdown and default_markdown is not None:
        args.benchmark_markdown = [default_markdown]
    return args


def validate_args(args) -> None:
    if args.max_corrected_rmse_m is not None and args.max_corrected_rmse_m <= 0.0:
        raise ValueError("--max-corrected-rmse-m must be positive")
    if args.min_improvement_percent is not None and args.min_improvement_percent < 0.0:
        raise ValueError("--min-improvement-percent must be non-negative")
    if args.max_gap_x is not None and args.max_gap_x <= 0.0:
        raise ValueError("--max-gap-x must be positive")
    if args.max_improvement_to_tie_percent is not None and args.max_improvement_to_tie_percent < 0.0:
        raise ValueError("--max-improvement-to-tie-percent must be non-negative")
    if args.residual_top_k < 0:
        raise ValueError("--residual-top-k must be non-negative")
    missing = missing_inputs(args)
    if missing:
        raise ValueError("missing required input(s): " + "; ".join(missing))


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        validate_args(args)
        _paths, result, summary = run_bundle(args)
    except (FileNotFoundError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(summary)
    return 0 if result.ok else 1


if __name__ == "__main__":
    sys.exit(main())
