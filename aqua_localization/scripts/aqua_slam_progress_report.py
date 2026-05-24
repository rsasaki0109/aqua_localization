#!/usr/bin/env python3
"""Report aqua_localization progress against the AQUA-SLAM Tank baseline."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_gap_report as gap_report


@dataclass(frozen=True)
class ProgressRow:
    case_key: tuple[str, str, str]
    target: gap_report.BenchmarkRow
    baseline: gap_report.BenchmarkRow
    anchor: gap_report.BenchmarkRow | None

    @property
    def gap_x(self) -> float:
        if self.target.rmse_m is None or self.baseline.rmse_m in {None, 0.0}:
            return math.nan
        return self.target.rmse_m / self.baseline.rmse_m

    @property
    def improvement_to_tie_percent(self) -> float:
        if self.target.rmse_m is None or self.baseline.rmse_m is None:
            return math.nan
        if self.target.rmse_m == 0.0:
            return 0.0
        return max(0.0, (1.0 - self.baseline.rmse_m / self.target.rmse_m) * 100.0)

    @property
    def anchor_improvement_percent(self) -> float | None:
        if self.anchor is None or self.anchor.rmse_m in {None, 0.0}:
            return None
        if self.target.rmse_m is None:
            return None
        return (1.0 - self.target.rmse_m / self.anchor.rmse_m) * 100.0

    @property
    def diagnostic(self) -> bool:
        return diagnostic_note(self.target.note)


def matching_target(row: gap_report.BenchmarkRow, prefixes: list[str]) -> bool:
    system = row.system.lower()
    return any(system.startswith(prefix.lower()) for prefix in prefixes)


def diagnostic_note(note: str) -> bool:
    text = note.lower()
    return any(
        marker in text
        for marker in (
            "diagnostic",
            "same-sequence",
            "same sequence",
            "override",
            "validate on held-out",
        )
    )


def best_baselines(
    rows: list[gap_report.BenchmarkRow],
    baseline_system: str,
) -> dict[tuple[str, str, str], gap_report.BenchmarkRow]:
    return gap_report.best_by_case(rows, baseline_system)


def best_anchors(
    rows: list[gap_report.BenchmarkRow],
    anchor_system: str,
) -> dict[tuple[str, str, str], gap_report.BenchmarkRow]:
    return gap_report.best_by_case(rows, anchor_system)


def collect_progress_rows(
    rows: list[gap_report.BenchmarkRow],
    *,
    baseline_system: str,
    anchor_system: str,
    target_prefixes: list[str],
) -> list[ProgressRow]:
    unique = gap_report.dedupe_rows(rows)
    baselines = best_baselines(unique, baseline_system)
    anchors = best_anchors(unique, anchor_system)
    best_targets: dict[tuple[tuple[str, str, str], str], gap_report.BenchmarkRow] = {}
    for row in unique:
        if row.rmse_m is None or not matching_target(row, target_prefixes):
            continue
        if gap_report.matching_system(row, baseline_system):
            continue
        key = (row.dataset, row.sequence, row.alignment)
        if key not in baselines:
            continue
        target_key = (key, row.system)
        previous = best_targets.get(target_key)
        if previous is None or row.rmse_m < previous.rmse_m:
            best_targets[target_key] = row

    progress: list[ProgressRow] = []
    for (key, _system), row in best_targets.items():
        progress.append(
            ProgressRow(
                case_key=key,
                target=row,
                baseline=baselines[key],
                anchor=anchors.get(key),
            )
        )
    return sorted(progress, key=lambda item: (item.case_key, item.gap_x, item.target.system))


def format_float(value: float | None, precision: int = 4) -> str:
    if value is None or math.isnan(value):
        return "TBD"
    if math.isinf(value):
        return "inf"
    return f"{value:.{precision}f}"


def format_percent(value: float | None, precision: int = 1) -> str:
    if value is None or math.isnan(value):
        return "TBD"
    return f"{value:.{precision}f}%"


def format_cell(value: str) -> str:
    return value.replace("`", "").replace("|", "\\|").replace("\n", " ").strip()


def best_progress(progress: list[ProgressRow]) -> ProgressRow | None:
    if not progress:
        return None
    return min(progress, key=lambda row: row.gap_x)


def best_claimable_progress(progress: list[ProgressRow]) -> ProgressRow | None:
    claimable = [row for row in progress if not row.diagnostic]
    if not claimable:
        return None
    return best_progress(claimable)


def format_report(
    progress: list[ProgressRow],
    *,
    source_paths: list[Path],
    baseline_system: str,
    anchor_system: str,
    target_prefixes: list[str],
) -> str:
    lines = [
        "# AQUA-SLAM Progress Report",
        "",
        f"- Sources: {', '.join(f'`{path}`' for path in source_paths)}",
        f"- Baseline system: `{baseline_system}`",
        f"- Anchor system: `{anchor_system}`",
        f"- Target prefixes: {', '.join(f'`{prefix}`' for prefix in target_prefixes)}",
        "",
    ]
    if not progress:
        lines.extend(
            [
                "No matching measured target rows were found.",
                "",
                "Add measured Tank rows for both AQUA-SLAM and aqua_localization first.",
            ]
        )
        return "\n".join(lines)

    lines.extend(
        [
            "| Dataset | Sequence | Alignment | System | RMSE m | Gap to AQUA-SLAM | Improvement to tie | Improvement vs anchor | Samples | Evidence status | Note |",
            "|---------|----------|-----------|--------|-------:|-----------------:|-------------------:|----------------------:|--------:|--------------|------|",
        ]
    )
    for row in progress:
        lines.append(
            "| "
            + " | ".join(
                [
                    row.target.dataset,
                    row.target.sequence,
                    row.target.alignment,
                    format_cell(row.target.system),
                    format_float(row.target.rmse_m),
                    format_float(row.gap_x, precision=2) + "x",
                    format_percent(row.improvement_to_tie_percent),
                    format_percent(row.anchor_improvement_percent),
                    str(row.target.samples) if row.target.samples is not None else "TBD",
                    "diagnostic" if row.diagnostic else "non-diagnostic",
                    format_cell(row.target.note),
                ]
            )
            + " |"
        )

    best = best_progress(progress)
    best_claimable = best_claimable_progress(progress)
    assert best is not None
    lines.extend(
        [
            "",
            "## Readout",
            "",
            (
                f"- Best current row: `{best.target.system}` on "
                f"`{best.target.sequence}` at {format_float(best.target.rmse_m)} m RMSE."
            ),
            (
                f"- Gap to `{baseline_system}` there: "
                f"{format_float(best.gap_x, precision=2)}x."
            ),
            (
                f"- Remaining RMSE reduction to tie: "
                f"{format_percent(best.improvement_to_tie_percent)}."
            ),
        ]
    )
    if best.diagnostic:
        lines.append("- Best current row is diagnostic; do not use it as a superiority claim.")
    if best_claimable is None:
        lines.append("- No non-diagnostic target row is available yet.")
    else:
        lines.append(
            f"- Best non-diagnostic row: `{best_claimable.target.system}` on "
            f"`{best_claimable.target.sequence}` at {format_float(best_claimable.target.rmse_m)} m RMSE "
            f"({format_float(best_claimable.gap_x, precision=2)}x `{baseline_system}`)."
        )
    if best.anchor_improvement_percent is not None:
        lines.append(
            f"- Improvement versus `{anchor_system}` anchor: "
            f"{format_percent(best.anchor_improvement_percent)}."
        )
    lines.extend(
        [
            "",
            "This report is a progress meter, not a superiority claim. A win requires "
            "a target row below the AQUA-SLAM RMSE on the same dataset, sequence, "
            "alignment, and reference trajectory.",
            "",
        ]
    )
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "markdown",
        nargs="+",
        type=Path,
        help="Markdown files containing Tank benchmark tables.",
    )
    parser.add_argument("--baseline-system", default="AQUA-SLAM")
    parser.add_argument("--anchor-system", default="aqua_localization")
    parser.add_argument(
        "--target-prefix",
        action="append",
        default=["aqua_"],
        help="System-name prefix to include. May be repeated.",
    )
    parser.add_argument("--out", type=Path, help="Optional Markdown output path.")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rows: list[gap_report.BenchmarkRow] = []
    try:
        for path in args.markdown:
            rows.extend(
                gap_report.parse_markdown_benchmark_rows(
                    path.read_text(encoding="utf-8")
                )
            )
    except OSError as exc:
        print(f"failed to read benchmark markdown: {exc}", file=sys.stderr)
        return 2

    progress = collect_progress_rows(
        rows,
        baseline_system=args.baseline_system,
        anchor_system=args.anchor_system,
        target_prefixes=args.target_prefix,
    )
    text = format_report(
        progress,
        source_paths=args.markdown,
        baseline_system=args.baseline_system,
        anchor_system=args.anchor_system,
        target_prefixes=args.target_prefix,
    )
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"wrote AQUA-SLAM progress report to {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
