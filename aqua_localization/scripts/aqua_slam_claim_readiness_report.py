#!/usr/bin/env python3
"""Render the current AQUA-SLAM claim readiness and held-out blockers."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import shlex
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import aqua_slam_head_to_head_report as head_to_head
import check_tank_aqua_slam_baseline_ready as baseline_ready
import locate_tank_heldout_inputs as locator
import verify_tank_medium_heldout_ready as heldout_ready


@dataclass(frozen=True)
class ClaimReadiness:
    comparisons: list[head_to_head.Comparison]
    reasons_by_case: dict[tuple[str, str, str], list[str]]
    best_numeric: head_to_head.Comparison | None
    best_claimable_win: head_to_head.Comparison | None
    heldout: heldout_ready.VerifyReport
    head_args: argparse.Namespace
    args: argparse.Namespace

    @property
    def claimable(self) -> bool:
        return self.best_claimable_win is not None


def shell_join(command: tuple[str, ...]) -> str:
    return shlex.join([str(part) for part in command])


def command_block(command: tuple[str, ...]) -> list[str]:
    if not command:
        return []
    return ["```bash", shell_join(command), "```"]


def default_markdown() -> Path:
    return baseline_ready.repo_benchmark_markdown()


def load_rows(paths: list[Path]) -> list[head_to_head.MetricRow]:
    rows: list[head_to_head.MetricRow] = []
    for path in paths:
        rows.extend(head_to_head.parse_metric_rows(path.read_text(encoding="utf-8")))
    return rows


def build_head_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        min_baseline_samples=args.min_baseline_samples,
        min_baseline_matched_s=args.min_baseline_matched_s,
        min_target_samples=args.min_target_samples,
        min_target_matched_s=args.min_target_matched_s,
        require_held_out_note=args.require_held_out_note,
        fail_without_claimable_win=True,
        fail_on_diagnostic_win=args.fail_on_diagnostic_win,
    )


def optional_path(argv: list[str], flag: str, value: Path | None) -> None:
    if value is not None:
        argv.extend([flag, str(value)])


def build_heldout_args(args: argparse.Namespace) -> argparse.Namespace:
    argv = [
        "--sequence",
        args.sequence,
        "--dataset",
        args.dataset,
        "--alignment",
        args.alignment,
        "--baseline-system",
        args.baseline_system,
        "--target-system",
        args.target_system,
        "--profile",
        str(args.profile),
        "--min-baseline-samples",
        str(args.min_baseline_samples),
        "--min-baseline-matched-s",
        str(args.min_baseline_matched_s),
        "--min-target-samples",
        str(args.min_target_samples),
        "--min-target-matched-s",
        str(args.min_target_matched_s),
        "--max-gap-x",
        str(args.max_gap_x),
        "--locator-max-depth",
        str(args.locator_max_depth),
        "--out-dir",
        str(args.heldout_out_dir),
    ]
    optional_path(argv, "--reference", args.reference)
    optional_path(argv, "--csv", args.csv)
    optional_path(argv, "--tum", args.tum)
    optional_path(argv, "--baseline-dir", args.baseline_dir)
    optional_path(argv, "--baseline-row", args.baseline_row)
    optional_path(argv, "--bag", args.bag)
    optional_path(argv, "--visual", args.visual)
    optional_path(argv, "--validation-out-dir", args.validation_out_dir)
    argv.extend(["--reference-topic", args.reference_topic])
    for path in args.markdown:
        argv.extend(["--benchmark-markdown", str(path)])
    for root in args.locator_root:
        argv.extend(["--locator-root", str(root)])
    return heldout_ready.parse_args(argv)


def build_readiness(args: argparse.Namespace) -> ClaimReadiness:
    rows = load_rows(args.markdown)
    comparisons = head_to_head.collect_comparisons(
        rows,
        baseline_system=args.baseline_system,
        target_prefixes=args.target_prefix,
    )
    head_args = build_head_args(args)
    reasons_by_case = {
        comparison.case_key: head_to_head.comparison_reasons(comparison, head_args)
        for comparison in comparisons
    }
    wins = head_to_head.claimable_wins(comparisons, reasons_by_case)
    best_claimable = min(wins, key=lambda item: item.gap_x) if wins else None
    heldout_args = build_heldout_args(args)
    return ClaimReadiness(
        comparisons=comparisons,
        reasons_by_case=reasons_by_case,
        best_numeric=head_to_head.best_numeric_comparison(comparisons),
        best_claimable_win=best_claimable,
        heldout=heldout_ready.build_verify_report(heldout_args),
        head_args=head_args,
        args=args,
    )


def pass_fail(value: bool) -> str:
    return "PASS" if value else "BLOCKED"


def format_optional_path(path: Path | None) -> str:
    return f"`{path}`" if path is not None else "none"


def claim_gate_command(args: argparse.Namespace) -> tuple[str, ...]:
    command = [
        "ros2",
        "run",
        "aqua_localization",
        "aqua_slam_head_to_head_report.py",
        *[str(path) for path in args.markdown],
        "--baseline-system",
        args.baseline_system,
        "--fail-without-claimable-win",
    ]
    for prefix in args.target_prefix:
        command.extend(["--target-prefix", prefix])
    if not args.require_held_out_note:
        command.append("--no-require-held-out-note")
    return tuple(command)


def heldout_link_bootstrap_command(args: argparse.Namespace) -> tuple[str, ...]:
    command = [
        "ros2",
        "run",
        "aqua_localization",
        "verify_tank_medium_heldout_ready.py",
        "--sequence",
        args.sequence,
        "--profile",
        str(args.profile),
        "--locator-max-depth",
        str(args.locator_max_depth),
        "--apply-located-links",
        "--out",
        str(args.heldout_out_dir / "heldout_verify.md"),
    ]
    for root in args.locator_root:
        command.extend(["--locator-root", str(root)])
    return tuple(command)


def comparison_summary(comparison: head_to_head.Comparison | None) -> list[str]:
    if comparison is None or comparison.target is None or comparison.baseline is None:
        return ["| TBD | TBD | TBD | TBD | TBD |"]
    return [
        "| "
        + " | ".join(
            [
                head_to_head.format_cell(comparison.target.system),
                head_to_head.format_cell(comparison.sequence),
                head_to_head.format_float(comparison.target.rmse_m),
                head_to_head.format_float(comparison.baseline.rmse_m),
                head_to_head.format_ratio(comparison.gap_x),
            ]
        )
        + " |"
    ]


def blockers_for_best_numeric(state: ClaimReadiness) -> list[str]:
    best = state.best_numeric
    if best is None:
        return ["no same-case measured AQUA-SLAM/current comparison exists"]
    return state.reasons_by_case.get(best.case_key, [])


def format_required_inputs(state: ClaimReadiness) -> list[str]:
    report = state.heldout.readiness_report
    args = report.args
    usable_rows = sum(len(source.matching_rows) for source in report.benchmark_sources)
    rejected_rows = sum(len(source.rejected_rows) for source in report.benchmark_sources)
    return [
        "| Input | Path | Status | Detail |",
        "|-------|------|--------|--------|",
        (
            f"| Reference TUM | `{args.reference}` | {baseline_ready.pass_fail(report.reference.valid)} | "
            f"{report.reference.detail} |"
        ),
        (
            f"| ROS 2 bag | `{args.bag}` | {baseline_ready.pass_fail(report.bag_exists)} | "
            f"{'exists' if report.bag_exists else 'missing'} |"
        ),
        (
            f"| Rank-1 profile | `{args.profile}` | {baseline_ready.pass_fail(report.profile_exists)} | "
            f"{'exists' if report.profile_exists else 'missing'} |"
        ),
        (
            f"| AQUA-SLAM source | `{args.csv}` / `{args.tum}` | "
            f"{baseline_ready.pass_fail(report.source_ready)} | {baseline_ready.source_label(report)} |"
        ),
        (
            f"| AQUA-SLAM baseline row | `{args.baseline_row}` | "
            f"{baseline_ready.pass_fail(report.baseline_row_ready)} | "
            f"usable={usable_rows}, rejected={rejected_rows} |"
        ),
        (
            f"| Visual TUM | `{args.visual}` | {baseline_ready.pass_fail(report.visual.valid)} | "
            f"{report.visual.detail} |"
        ),
    ]


def format_candidate_counts(state: ClaimReadiness) -> list[str]:
    report = state.heldout.locate_report
    lines = [
        "| Role | Count | First candidate |",
        "|------|------:|-----------------|",
    ]
    for role, label in (
        ("reference_tum", "Reference TUM"),
        ("ros2_bag", "ROS 2 bag"),
        ("ros1_bag", "ROS 1 bag"),
        ("visual_tum", "Visual TUM"),
        ("aqua_slam_csv", "AQUA-SLAM CSV"),
        ("aqua_slam_tum", "AQUA-SLAM TUM"),
        ("baseline_row", "AQUA-SLAM baseline row"),
    ):
        first = locator.first_path(report, role)
        lines.append(f"| {label} | {len(report.by_role(role))} | {format_optional_path(first)} |")
    return lines


def format_report(state: ClaimReadiness) -> str:
    args = state.args
    status = "CLAIMABLE_WIN" if state.claimable else "BLOCKED"
    lines = [
        "# AQUA-SLAM Claim Readiness",
        "",
        f"- Status: `{status}`",
        f"- Benchmark sources: {', '.join(f'`{path}`' for path in args.markdown)}",
        f"- Baseline: `{args.baseline_system}`",
        f"- Target prefixes: {', '.join(f'`{prefix}`' for prefix in args.target_prefix)}",
        f"- Held-out sequence: `{state.heldout.readiness_report.sequence}`",
        f"- Official Tank Dataset download page: {locator.OFFICIAL_DOWNLOAD_URL}",
        "",
        "## Current Claim",
        "",
        "| Best current system | Sequence | Current RMSE m | AQUA-SLAM RMSE m | Gap |",
        "|---------------------|----------|---------------:|-----------------:|----:|",
        *comparison_summary(state.best_numeric),
        "",
    ]
    if state.best_claimable_win is not None:
        lines.append(
            f"- Best claimable win: `{state.best_claimable_win.target.system}` on "
            f"`{state.best_claimable_win.sequence}` at "
            f"{head_to_head.format_ratio(state.best_claimable_win.gap_x)}."
        )
    else:
        blockers = blockers_for_best_numeric(state)
        lines.append("- No claimable AQUA-SLAM win is available yet.")
        lines.append(
            "- Best numeric blockers: "
            + ("; ".join(blockers) if blockers else "none")
            + "."
        )

    lines.extend(
        [
            "",
            "## Claim Gate",
            "",
            *command_block(claim_gate_command(args)),
            "",
            "## Held-Out Readiness",
            "",
            f"- Status: `{pass_fail(state.heldout.ready)}`",
            f"- Next action: `{state.heldout.next_action.title}`",
            "",
            *format_required_inputs(state),
            "",
            "## Candidate Link Bootstrap",
            "",
            *command_block(heldout_link_bootstrap_command(args)),
            "",
            "## Located Candidates",
            "",
            *format_candidate_counts(state),
            "",
            "## Next Action Command",
            "",
            f"{state.heldout.next_action.title}: {state.heldout.next_action.detail}",
            "",
            *command_block(state.heldout.next_action.command),
            "",
        ]
    )
    if state.heldout.ready:
        heldout_args = build_heldout_args(args)
        lines.extend(
            [
                "## Held-Out Validation Command",
                "",
                *command_block(heldout_ready.validation_command(state.heldout.readiness_report, heldout_args)),
                "",
            ]
        )
    lines.extend(
        [
            "## Practical Order",
            "",
            "1. Satisfy the first failing held-out input above.",
            "2. Run the Medium held-out validation bundle.",
            "3. Regenerate the head-to-head report and rerun the claim gate.",
            "4. Only publish \"beats AQUA-SLAM\" wording after this report reaches `CLAIMABLE_WIN`.",
        ]
    )
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("markdown", nargs="*", type=Path, help="Benchmark Markdown sources.")
    parser.add_argument("--dataset", default=baseline_ready.DEFAULT_DATASET)
    parser.add_argument("--sequence", default=baseline_ready.DEFAULT_SEQUENCE)
    parser.add_argument("--alignment", default=baseline_ready.DEFAULT_ALIGNMENT)
    parser.add_argument("--baseline-system", default=baseline_ready.DEFAULT_BASELINE_SYSTEM)
    parser.add_argument("--target-system", default=baseline_ready.DEFAULT_TARGET_SYSTEM)
    parser.add_argument("--target-prefix", action="append", default=["aqua_"])
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--tum", type=Path)
    parser.add_argument("--baseline-dir", type=Path)
    parser.add_argument("--baseline-row", type=Path)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--visual", type=Path)
    parser.add_argument("--reference-topic", default="/apriltag_slam/GT")
    parser.add_argument("--profile", type=Path, default=baseline_ready.DEFAULT_PROFILE)
    parser.add_argument("--min-baseline-samples", type=int, default=baseline_ready.DEFAULT_MIN_BASELINE_SAMPLES)
    parser.add_argument("--min-baseline-matched-s", type=float, default=baseline_ready.DEFAULT_MIN_BASELINE_MATCHED_S)
    parser.add_argument("--min-target-samples", type=int, default=10)
    parser.add_argument("--min-target-matched-s", type=float, default=10.0)
    parser.add_argument("--max-gap-x", type=float, default=1.0)
    parser.add_argument("--locator-root", action="append", type=Path, default=[])
    parser.add_argument("--locator-max-depth", type=int, default=7)
    parser.add_argument("--heldout-out-dir", type=Path, default=heldout_ready.DEFAULT_OUT_DIR)
    parser.add_argument("--validation-out-dir", type=Path)
    parser.add_argument(
        "--no-require-held-out-note",
        dest="require_held_out_note",
        action="store_false",
    )
    parser.set_defaults(require_held_out_note=True)
    parser.add_argument("--fail-on-diagnostic-win", action="store_true")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    if not args.markdown:
        args.markdown = [default_markdown()]
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        state = build_readiness(args)
    except (OSError, ValueError) as exc:
        print(f"failed to build AQUA-SLAM claim readiness: {exc}", file=sys.stderr)
        return 2
    text = format_report(state)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"wrote AQUA-SLAM claim readiness report to {args.out}")
    else:
        print(text)
    return 0 if state.claimable else 1


if __name__ == "__main__":
    sys.exit(main())
