#!/usr/bin/env python3
"""Check readiness for a held-out Tank AQUA-SLAM baseline comparison."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import benchmark_gap_report
import compare_trajectories
import ingest_tank_aqua_slam_baseline
import ros1_odometry_csv_to_tum


DEFAULT_SEQUENCE = "Medium"
DEFAULT_DATASET = "Tank Dataset"
DEFAULT_BASELINE_SYSTEM = "AQUA-SLAM"
DEFAULT_TARGET_SYSTEM = "aqua_dvl_prior_visual"
DEFAULT_ALIGNMENT = "SE(3)"
DEFAULT_PROFILE = Path("/tmp/aqua_tank_dvl_prior_confidence_sweep_short_diag/best_profile.yaml")


@dataclass(frozen=True)
class TrajectorySummary:
    path: Path
    exists: bool
    valid: bool
    count: int | None
    duration_s: float | None
    detail: str


@dataclass(frozen=True)
class MarkdownSourceSummary:
    path: Path
    exists: bool
    valid: bool
    matching_rows: tuple[benchmark_gap_report.BenchmarkRow, ...]
    detail: str


@dataclass(frozen=True)
class ReadinessReport:
    sequence: str
    dataset: str
    reference: TrajectorySummary
    csv: TrajectorySummary
    tum: TrajectorySummary
    benchmark_sources: tuple[MarkdownSourceSummary, ...]
    profile_exists: bool
    bag_exists: bool
    visual: TrajectorySummary
    args: argparse.Namespace

    @property
    def source_ready(self) -> bool:
        return self.csv.valid or self.tum.valid

    @property
    def ingest_ready(self) -> bool:
        return self.reference.valid and self.source_ready

    @property
    def baseline_row_ready(self) -> bool:
        return any(source.matching_rows for source in self.benchmark_sources)

    @property
    def validation_ready(self) -> bool:
        return (
            self.reference.valid
            and self.baseline_row_ready
            and self.profile_exists
            and self.bag_exists
            and self.visual.valid
        )


def repo_benchmark_markdown() -> Path:
    return Path(__file__).resolve().parents[2] / "docs" / "benchmarks" / "tank_aqua_slam.md"


def sequence_stem(sequence: str) -> str:
    return ingest_tank_aqua_slam_baseline.sanitize_name(sequence)


def sequence_slug(sequence: str) -> str:
    return sequence_stem(sequence).lower()


def default_baseline_dir(sequence: str) -> Path:
    return Path(f"/tmp/aqua_slam_{sequence_slug(sequence)}_baseline")


def default_reference(sequence: str) -> Path:
    return Path(f"/tmp/tank_{sequence_slug(sequence)}_gt.tum")


def default_csv(sequence: str) -> Path:
    return Path(f"/tmp/aqua_slam_{sequence_slug(sequence)}_orb_odom.csv")


def default_visual(sequence: str) -> Path:
    return Path(f"/tmp/tank_{sequence_slug(sequence)}_visual_frontend.tum")


def default_bag(sequence: str) -> Path:
    return Path(f"/tmp/tank_{sequence_slug(sequence)}_ros2_visual")


def baseline_paths(args) -> ingest_tank_aqua_slam_baseline.BaselinePaths:
    out_dir = args.baseline_dir or default_baseline_dir(args.sequence)
    return ingest_tank_aqua_slam_baseline.default_paths(out_dir, args.sequence)


def resolve_defaults(args) -> None:
    paths = baseline_paths(args)
    if args.reference is None:
        args.reference = default_reference(args.sequence)
    if args.csv is None:
        args.csv = default_csv(args.sequence)
    if args.tum is None:
        args.tum = paths.estimate_tum
    if args.baseline_row is None:
        args.baseline_row = paths.benchmark_row
    if args.visual is None:
        args.visual = default_visual(args.sequence)
    if args.bag is None:
        args.bag = default_bag(args.sequence)
    if not args.benchmark_markdown:
        candidate = repo_benchmark_markdown()
        if candidate.exists():
            args.benchmark_markdown = [candidate]


def summarize_tum(path: Path) -> TrajectorySummary:
    if not path.exists():
        return TrajectorySummary(path, False, False, None, None, "missing")
    try:
        rows = compare_trajectories.load_tum(path)
    except Exception as exc:
        return TrajectorySummary(path, True, False, None, None, f"invalid TUM: {exc}")
    duration = float(rows[-1, 0] - rows[0, 0]) if rows.shape[0] > 1 else 0.0
    return TrajectorySummary(path, True, True, int(rows.shape[0]), duration, "ok")


def summarize_csv(path: Path, time_unit: str) -> TrajectorySummary:
    if not path.exists():
        return TrajectorySummary(path, False, False, None, None, "missing")
    try:
        rows = ros1_odometry_csv_to_tum.convert_rows(path, time_unit=time_unit)
    except Exception as exc:
        return TrajectorySummary(path, True, False, None, None, f"invalid CSV: {exc}")
    if not rows:
        return TrajectorySummary(path, True, False, 0, 0.0, "empty CSV")
    duration = float(rows[-1][0] - rows[0][0]) if len(rows) > 1 else 0.0
    return TrajectorySummary(path, True, True, len(rows), duration, "ok")


def matching_baseline_rows(args, rows) -> tuple[benchmark_gap_report.BenchmarkRow, ...]:
    matches = []
    for row in rows:
        if row.dataset != args.dataset:
            continue
        if row.sequence != args.sequence:
            continue
        if row.alignment != args.alignment:
            continue
        if not benchmark_gap_report.matching_system(row, args.baseline_system):
            continue
        matches.append(row)
    return tuple(matches)


def summarize_markdown_source(path: Path, args) -> MarkdownSourceSummary:
    if not path.exists():
        return MarkdownSourceSummary(path, False, False, (), "missing")
    try:
        rows = benchmark_gap_report.parse_markdown_benchmark_rows(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return MarkdownSourceSummary(path, True, False, (), f"invalid Markdown: {exc}")
    matches = matching_baseline_rows(args, rows)
    detail = f"{len(matches)} matching {args.baseline_system} row(s)" if matches else "no matching baseline row"
    return MarkdownSourceSummary(path, True, True, matches, detail)


def build_report(args) -> ReadinessReport:
    resolve_defaults(args)
    markdown_paths = [*args.benchmark_markdown, args.baseline_row]
    deduped_markdown_paths = list(dict.fromkeys(markdown_paths))
    return ReadinessReport(
        sequence=args.sequence,
        dataset=args.dataset,
        reference=summarize_tum(args.reference),
        csv=summarize_csv(args.csv, args.time_unit),
        tum=summarize_tum(args.tum),
        benchmark_sources=tuple(
            summarize_markdown_source(path, args) for path in deduped_markdown_paths
        ),
        profile_exists=args.profile.exists(),
        bag_exists=args.bag.exists(),
        visual=summarize_tum(args.visual),
        args=args,
    )


def pass_fail(value: bool) -> str:
    return "PASS" if value else "FAIL"


def format_duration(value: float | None) -> str:
    return "TBD" if value is None else f"{value:.2f}"


def format_count(value: int | None) -> str:
    return "TBD" if value is None else str(value)


def source_label(report: ReadinessReport) -> str:
    if report.csv.valid:
        return f"CSV `{report.csv.path}`"
    if report.tum.valid:
        return f"TUM `{report.tum.path}`"
    return "none"


def format_trajectory_row(label: str, summary: TrajectorySummary) -> str:
    status = pass_fail(summary.valid)
    return (
        f"| {label} | `{summary.path}` | {status} | "
        f"{format_count(summary.count)} | {format_duration(summary.duration_s)} | {summary.detail} |"
    )


def format_path_row(label: str, path: Path, exists: bool) -> str:
    return f"| {label} | `{path}` | {pass_fail(exists)} | {'exists' if exists else 'missing'} |"


def ingest_source_command_arg(report: ReadinessReport) -> str:
    if report.tum.valid and not report.csv.valid:
        return f"  --tum {report.args.tum} \\"
    return f"  --csv {report.args.csv} \\"


def format_report(report: ReadinessReport) -> str:
    args = report.args
    lines = [
        "# Tank AQUA-SLAM Baseline Readiness Report",
        "",
        f"- Dataset: `{report.dataset}`",
        f"- Sequence: `{report.sequence}`",
        f"- Baseline system: `{args.baseline_system}`",
        f"- Target system: `{args.target_system}`",
        f"- Alignment: `{args.alignment}`",
        "",
        "## Stage Verdicts",
        "",
        "| Stage | Status | Detail |",
        "|-------|--------|--------|",
        (
            f"| Baseline ingest inputs | {pass_fail(report.ingest_ready)} | "
            f"reference={pass_fail(report.reference.valid)}, source={source_label(report)} |"
        ),
        (
            f"| Baseline row for gap checks | {pass_fail(report.baseline_row_ready)} | "
            f"{sum(len(source.matching_rows) for source in report.benchmark_sources)} matching row(s) |"
        ),
        (
            f"| Held-out validation bundle | {pass_fail(report.validation_ready)} | "
            f"profile={pass_fail(report.profile_exists)}, bag={pass_fail(report.bag_exists)}, "
            f"visual={pass_fail(report.visual.valid)}, gap row={pass_fail(report.baseline_row_ready)} |"
        ),
        "",
        "## Trajectory Inputs",
        "",
        "| Input | Path | Status | Samples | Duration s | Detail |",
        "|-------|------|--------|--------:|-----------:|--------|",
        format_trajectory_row("Reference TUM", report.reference),
        format_trajectory_row("AQUA-SLAM CSV", report.csv),
        format_trajectory_row("AQUA-SLAM TUM", report.tum),
        format_trajectory_row("Visual frontend TUM", report.visual),
        "",
        "## Benchmark Sources",
        "",
        "| Source | Status | Matching rows | Detail |",
        "|--------|--------|--------------:|--------|",
    ]
    for source in report.benchmark_sources:
        lines.append(
            f"| `{source.path}` | {pass_fail(source.valid and bool(source.matching_rows))} | "
            f"{len(source.matching_rows)} | {source.detail} |"
        )

    lines.extend(
        [
            "",
            "## Validation Inputs",
            "",
            "| Input | Path | Status | Detail |",
            "|-------|------|--------|--------|",
            format_path_row("DVL prior profile", args.profile, report.profile_exists),
            format_path_row("ROS 2 validation bag", args.bag, report.bag_exists),
            "",
            "## Next Commands",
            "",
        ]
    )
    if not report.ingest_ready:
        lines.extend([
            "Record or expose the AQUA-SLAM trajectory, then ingest it:",
            "",
            "```bash",
            f"rostopic echo -p {args.source_topic} > {args.csv}",
            "",
            "ros2 run aqua_localization ingest_tank_aqua_slam_baseline.py \\",
            ingest_source_command_arg(report),
            f"  --reference {args.reference} \\",
            f"  --sequence {args.sequence} \\",
            f"  --config {args.config} \\",
            f"  --out-dir {baseline_paths(args).estimate_tum.parent}",
            "```",
        ])
    elif not report.baseline_row_ready:
        lines.extend([
            "Generate the missing AQUA-SLAM benchmark row:",
            "",
            "```bash",
            "ros2 run aqua_localization ingest_tank_aqua_slam_baseline.py \\",
            ingest_source_command_arg(report),
            f"  --reference {args.reference} \\",
            f"  --sequence {args.sequence} \\",
            f"  --config {args.config} \\",
            f"  --out-dir {baseline_paths(args).estimate_tum.parent}",
            "```",
        ])
    else:
        lines.extend([
            "Run the held-out validation bundle against the matching AQUA-SLAM row:",
            "",
            "```bash",
            "ros2 run aqua_localization run_tank_dvl_validation_bundle.py \\",
            f"  --profile {args.profile} \\",
            f"  --sequence {args.sequence} \\",
            f"  --bag {args.bag} \\",
            f"  --reference {args.reference} \\",
            f"  --visual {args.visual} \\",
            f"  --benchmark-markdown {repo_benchmark_markdown()} \\",
            f"  --benchmark-markdown {args.baseline_row} \\",
            "  --max-gap-x 1.0 \\",
            "  --fail-on-gate-failure \\",
            f"  --out-dir /tmp/aqua_tank_dvl_prior_{sequence_slug(args.sequence)}_validation_bundle",
            "```",
        ])
    return "\n".join(lines) + "\n"


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Check whether a Tank held-out AQUA-SLAM baseline row is ready for gap validation."
    )
    parser.add_argument("--sequence", default=DEFAULT_SEQUENCE)
    parser.add_argument("--dataset", default=DEFAULT_DATASET)
    parser.add_argument("--alignment", default=DEFAULT_ALIGNMENT)
    parser.add_argument("--baseline-system", default=DEFAULT_BASELINE_SYSTEM)
    parser.add_argument("--target-system", default=DEFAULT_TARGET_SYSTEM)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--tum", type=Path)
    parser.add_argument("--baseline-dir", type=Path)
    parser.add_argument("--baseline-row", type=Path)
    parser.add_argument("--benchmark-markdown", action="append", type=Path, default=[])
    parser.add_argument(
        "--profile",
        type=Path,
        default=DEFAULT_PROFILE,
    )
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--visual", type=Path)
    parser.add_argument("--time-unit", choices=("auto", "seconds", "nanoseconds"), default="auto")
    parser.add_argument("--source-topic", default=ingest_tank_aqua_slam_baseline.DEFAULT_SOURCE)
    parser.add_argument("--config", default="underwater_orbslam3_blue_gx5_medium.yaml")
    parser.add_argument("--out", type=Path, help="Optional Markdown report path.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    report = build_report(args)
    text = format_report(report)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0 if report.validation_ready else 1


if __name__ == "__main__":
    sys.exit(main())
