#!/usr/bin/env python3
"""Render a claim-aware AQUA-SLAM head-to-head report from benchmark tables."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Callable, Iterable

sys.path.insert(0, str(Path(__file__).resolve().parent))
import benchmark_gap_report as markdown


REQUIRED_COLUMNS = {"Dataset", "Sequence", "System", "Alignment"}
P95_COLUMNS = ("P95 m", "95% m", "ATE P95 m", "P95")
DIAGNOSTIC_MARKERS = (
    "diagnostic",
    "same-sequence",
    "same sequence",
    "override",
    "validate on held-out",
    "not paper-safe",
    "smoke",
)
HELD_OUT_MARKERS = (
    "held-out",
    "held out",
    "validation sequence",
    "test split",
)


@dataclass(frozen=True)
class MetricRow:
    dataset: str
    sequence: str
    system: str
    alignment: str
    samples: int | None
    matched_seconds: float | None
    mean_m: float | None
    median_m: float | None
    rmse_m: float | None
    p95_m: float | None
    max_m: float | None
    note: str

    @property
    def measured(self) -> bool:
        return self.rmse_m is not None and self.alignment.upper() != "TBD"

    @property
    def case_key(self) -> tuple[str, str, str]:
        return (self.dataset, self.sequence, self.alignment)


@dataclass(frozen=True)
class Comparison:
    case_key: tuple[str, str, str]
    target: MetricRow | None
    baseline: MetricRow | None
    pending_targets: tuple[MetricRow, ...] = ()
    pending_baselines: tuple[MetricRow, ...] = ()

    @property
    def dataset(self) -> str:
        return self.case_key[0]

    @property
    def sequence(self) -> str:
        return self.case_key[1]

    @property
    def alignment(self) -> str:
        return self.case_key[2]

    @property
    def gap_x(self) -> float:
        if self.target is None or self.baseline is None:
            return math.nan
        if self.target.rmse_m is None or self.baseline.rmse_m in {None, 0.0}:
            return math.nan
        return self.target.rmse_m / self.baseline.rmse_m

    @property
    def improvement_to_tie_percent(self) -> float:
        if self.target is None or self.baseline is None:
            return math.nan
        if self.target.rmse_m is None or self.baseline.rmse_m is None:
            return math.nan
        if self.target.rmse_m == 0.0:
            return 0.0
        return max(0.0, (1.0 - self.baseline.rmse_m / self.target.rmse_m) * 100.0)


def format_cell(value: str) -> str:
    return value.replace("`", "").replace("|", "\\|").replace("\n", " ").strip()


def format_float(value: float | None, precision: int = 4) -> str:
    if value is None or math.isnan(value):
        return "TBD"
    if math.isinf(value):
        return "inf"
    return f"{value:.{precision}f}"


def format_ratio(value: float | None) -> str:
    if value is None or math.isnan(value):
        return "TBD"
    if math.isinf(value):
        return "inf"
    return f"{value:.2f}x"


def format_percent(value: float | None) -> str:
    if value is None or math.isnan(value):
        return "TBD"
    return f"{value:.1f}%"


def table_cell(cells: list[str], index: dict[str, int], column: str) -> str:
    if column not in index:
        return ""
    column_index = index[column]
    if column_index >= len(cells):
        return ""
    return cells[column_index]


def optional_float(cells: list[str], index: dict[str, int], *columns: str) -> float | None:
    for column in columns:
        value = markdown.parse_float(table_cell(cells, index, column))
        if value is not None:
            return value
    return None


def optional_int(cells: list[str], index: dict[str, int], column: str) -> int | None:
    return markdown.parse_int(table_cell(cells, index, column))


def rows_from_table(header: list[str], rows: list[list[str]]) -> list[MetricRow]:
    normalized_header = [markdown.normalize_text(cell) for cell in header]
    if not REQUIRED_COLUMNS.issubset(set(normalized_header)):
        return []

    index = {name: normalized_header.index(name) for name in normalized_header}
    parsed: list[MetricRow] = []
    for cells in rows:
        if len(cells) < len(normalized_header):
            continue
        parsed.append(
            MetricRow(
                dataset=markdown.normalize_text(table_cell(cells, index, "Dataset")),
                sequence=markdown.normalize_text(table_cell(cells, index, "Sequence")),
                system=markdown.normalize_text(table_cell(cells, index, "System")),
                alignment=markdown.normalize_text(table_cell(cells, index, "Alignment")),
                samples=optional_int(cells, index, "Samples"),
                matched_seconds=optional_float(cells, index, "Matched s"),
                mean_m=optional_float(cells, index, "Mean m"),
                median_m=optional_float(cells, index, "Median m"),
                rmse_m=optional_float(cells, index, "RMSE m"),
                p95_m=optional_float(cells, index, *P95_COLUMNS),
                max_m=optional_float(cells, index, "Max m"),
                note=markdown.normalize_text(table_cell(cells, index, "Note")),
            )
        )
    return parsed


def parse_metric_rows(text: str) -> list[MetricRow]:
    lines = text.splitlines()
    parsed: list[MetricRow] = []
    i = 0
    while i < len(lines) - 1:
        header = markdown.split_markdown_row(lines[i])
        separator = markdown.split_markdown_row(lines[i + 1])
        if header and markdown.is_separator_row(separator):
            table_rows = []
            i += 2
            while i < len(lines):
                cells = markdown.split_markdown_row(lines[i])
                if not cells or markdown.is_separator_row(cells):
                    break
                table_rows.append(cells)
                i += 1
            parsed.extend(rows_from_table(header, table_rows))
        else:
            i += 1
    return parsed


def dedupe_rows(rows: Iterable[MetricRow]) -> list[MetricRow]:
    seen = set()
    unique: list[MetricRow] = []
    for row in rows:
        key = (
            row.dataset,
            row.sequence,
            row.system,
            row.alignment,
            row.samples,
            row.matched_seconds,
            row.mean_m,
            row.median_m,
            row.rmse_m,
            row.p95_m,
            row.max_m,
            row.note,
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(row)
    return unique


def matching_system(row: MetricRow, system: str) -> bool:
    return row.system.lower() == system.strip("`").lower()


def matching_target(row: MetricRow, target_prefixes: list[str]) -> bool:
    system = row.system.lower()
    return any(system.startswith(prefix.lower()) for prefix in target_prefixes)


def best_measured_by_case(
    rows: Iterable[MetricRow],
    predicate: Callable[[MetricRow], bool],
) -> dict[tuple[str, str, str], MetricRow]:
    best: dict[tuple[str, str, str], MetricRow] = {}
    for row in rows:
        if not row.measured or row.rmse_m is None or not predicate(row):
            continue
        previous = best.get(row.case_key)
        if previous is None or row.rmse_m < previous.rmse_m:
            best[row.case_key] = row
    return best


def pending_by_case(
    rows: Iterable[MetricRow],
    predicate: Callable[[MetricRow], bool],
) -> dict[tuple[str, str, str], tuple[MetricRow, ...]]:
    pending: dict[tuple[str, str, str], list[MetricRow]] = {}
    for row in rows:
        if row.measured or not predicate(row):
            continue
        pending.setdefault(row.case_key, []).append(row)
    return {key: tuple(value) for key, value in pending.items()}


def collect_comparisons(
    rows: list[MetricRow],
    *,
    baseline_system: str,
    target_prefixes: list[str],
) -> list[Comparison]:
    unique = dedupe_rows(rows)
    baseline_predicate = lambda row: matching_system(row, baseline_system)
    target_predicate = lambda row: matching_target(row, target_prefixes) and not matching_system(
        row, baseline_system
    )
    baselines = best_measured_by_case(unique, baseline_predicate)
    targets = best_measured_by_case(unique, target_predicate)
    pending_baselines = pending_by_case(unique, baseline_predicate)
    pending_targets = pending_by_case(unique, target_predicate)
    keys = set(baselines) | set(targets) | set(pending_baselines) | set(pending_targets)
    comparisons = [
        Comparison(
            case_key=key,
            target=targets.get(key),
            baseline=baselines.get(key),
            pending_targets=pending_targets.get(key, ()),
            pending_baselines=pending_baselines.get(key, ()),
        )
        for key in keys
    ]
    return sorted(
        comparisons,
        key=lambda item: (
            item.dataset,
            item.sequence,
            item.alignment,
            item.gap_x if not math.isnan(item.gap_x) else math.inf,
        ),
    )


def diagnostic_note(note: str) -> bool:
    text = note.lower()
    return any(marker in text for marker in DIAGNOSTIC_MARKERS)


def held_out_established(row: MetricRow) -> bool:
    text = row.note.lower()
    if diagnostic_note(row.note):
        return False
    return any(marker in text for marker in HELD_OUT_MARKERS)


def coverage_reasons(
    label: str,
    row: MetricRow,
    *,
    min_samples: int,
    min_matched_s: float,
) -> list[str]:
    reasons = []
    if row.samples is None:
        reasons.append(f"{label} sample count missing")
    elif row.samples < min_samples:
        reasons.append(f"{label} row too short: {row.samples} samples < {min_samples}")
    if row.matched_seconds is None:
        reasons.append(f"{label} matched duration missing")
    elif row.matched_seconds < min_matched_s:
        reasons.append(
            f"{label} matched duration too short: "
            f"{row.matched_seconds:.2f}s < {min_matched_s:.2f}s"
        )
    return reasons


def comparison_reasons(comparison: Comparison, args: argparse.Namespace) -> list[str]:
    reasons: list[str] = []
    if comparison.baseline is None:
        reasons.append("missing AQUA-SLAM measured row")
    if comparison.target is None:
        reasons.append("missing current measured row")
    if comparison.baseline is not None:
        reasons.extend(
            coverage_reasons(
                "baseline",
                comparison.baseline,
                min_samples=args.min_baseline_samples,
                min_matched_s=args.min_baseline_matched_s,
            )
        )
    if comparison.target is not None:
        reasons.extend(
            coverage_reasons(
                "target",
                comparison.target,
                min_samples=args.min_target_samples,
                min_matched_s=args.min_target_matched_s,
            )
        )
        if diagnostic_note(comparison.target.note):
            reasons.append("current row is diagnostic")
        if args.require_held_out_note and not held_out_established(comparison.target):
            reasons.append("held-out validation not established")
    return reasons


def verdict(comparison: Comparison, reasons: list[str]) -> str:
    if comparison.target is None or comparison.baseline is None:
        return "blocked"
    if comparison.target.rmse_m is None or comparison.baseline.rmse_m is None:
        return "blocked"
    if comparison.gap_x < 1.0:
        if not reasons:
            return "claimable win"
        if diagnostic_note(comparison.target.note):
            return "diagnostic win"
        return "win, evidence blocked"
    if comparison.gap_x == 1.0:
        return "tie, evidence blocked" if reasons else "claimable tie"
    return "behind"


def measured_comparisons(comparisons: list[Comparison]) -> list[Comparison]:
    return [
        item
        for item in comparisons
        if item.target is not None
        and item.baseline is not None
        and item.target.rmse_m is not None
        and item.baseline.rmse_m is not None
    ]


def best_numeric_comparison(comparisons: list[Comparison]) -> Comparison | None:
    measured = measured_comparisons(comparisons)
    if not measured:
        return None
    return min(measured, key=lambda item: item.gap_x)


def best_claimable_comparison(
    comparisons: list[Comparison],
    reasons_by_case: dict[tuple[str, str, str], list[str]],
) -> Comparison | None:
    claimable = [
        item
        for item in measured_comparisons(comparisons)
        if item.gap_x <= 1.0 and not reasons_by_case[item.case_key]
    ]
    if not claimable:
        return None
    return min(claimable, key=lambda item: item.gap_x)


def claimable_wins(
    comparisons: list[Comparison],
    reasons_by_case: dict[tuple[str, str, str], list[str]],
) -> list[Comparison]:
    return [
        item
        for item in measured_comparisons(comparisons)
        if item.gap_x < 1.0 and not reasons_by_case[item.case_key]
    ]


def unclaimable_wins(
    comparisons: list[Comparison],
    reasons_by_case: dict[tuple[str, str, str], list[str]],
) -> list[Comparison]:
    return [
        item
        for item in measured_comparisons(comparisons)
        if item.gap_x < 1.0 and reasons_by_case[item.case_key]
    ]


def diagnostic_wins(
    comparisons: list[Comparison],
    reasons_by_case: dict[tuple[str, str, str], list[str]],
) -> list[Comparison]:
    return [
        item
        for item in unclaimable_wins(comparisons, reasons_by_case)
        if item.target is not None and diagnostic_note(item.target.note)
    ]


def metric_pair(current: float | None, baseline: float | None, precision: int = 4) -> str:
    return f"{format_float(current, precision)} / {format_float(baseline, precision)}"


def metric_gap(current: float | None, baseline: float | None) -> str:
    if current is None or baseline in {None, 0.0}:
        return "TBD"
    return format_ratio(current / baseline)


def format_pending_rows(comparison: Comparison) -> str:
    rows = list(comparison.pending_targets) + list(comparison.pending_baselines)
    if not rows:
        return ""
    return "; ".join(f"{row.system}: {row.note or 'pending'}" for row in rows)


def format_report(
    comparisons: list[Comparison],
    *,
    source_paths: list[Path],
    baseline_system: str,
    target_prefixes: list[str],
    args: argparse.Namespace,
) -> str:
    reasons_by_case = {
        comparison.case_key: comparison_reasons(comparison, args)
        for comparison in comparisons
    }
    best_numeric = best_numeric_comparison(comparisons)
    best_claimable = best_claimable_comparison(comparisons, reasons_by_case)
    lines = [
        "# AQUA-SLAM Head-to-Head Diagnosis",
        "",
        f"- Sources: {', '.join(f'`{path}`' for path in source_paths)}",
        f"- Baseline system: `{baseline_system}`",
        f"- Target prefixes: {', '.join(f'`{prefix}`' for prefix in target_prefixes)}",
        (
            "- Claim gates: "
            f"baseline >= {args.min_baseline_samples} samples / "
            f"{args.min_baseline_matched_s:.1f}s, target >= "
            f"{args.min_target_samples} samples / {args.min_target_matched_s:.1f}s"
        ),
        f"- Held-out note required: `{str(args.require_held_out_note).lower()}`",
        "",
    ]
    if not comparisons:
        lines.extend(
            [
                "No benchmark rows with dataset, sequence, system, and alignment columns were found.",
                "",
                "Add Tank benchmark rows for AQUA-SLAM and at least one `aqua_*` system first.",
            ]
        )
        return "\n".join(lines)

    lines.extend(
        [
            "## Claim Table",
            "",
            "| Dataset | Sequence | Alignment | Current system | Verdict | Current RMSE m | AQUA-SLAM RMSE m | Gap | Improvement to tie | Evidence blockers |",
            "|---------|----------|-----------|----------------|---------|---------------:|-----------------:|----:|-------------------:|-------------------|",
        ]
    )
    for comparison in comparisons:
        reasons = reasons_by_case[comparison.case_key]
        lines.append(
            "| "
            + " | ".join(
                [
                    format_cell(comparison.dataset),
                    format_cell(comparison.sequence),
                    format_cell(comparison.alignment),
                    format_cell(comparison.target.system if comparison.target else "TBD"),
                    verdict(comparison, reasons),
                    format_float(comparison.target.rmse_m if comparison.target else None),
                    format_float(comparison.baseline.rmse_m if comparison.baseline else None),
                    format_ratio(comparison.gap_x),
                    format_percent(comparison.improvement_to_tie_percent),
                    format_cell("; ".join(reasons) if reasons else "none"),
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Metric Detail",
            "",
            "| Dataset | Sequence | Current / AQUA-SLAM mean m | median m | P95 m | max m | samples | matched s | Pending evidence |",
            "|---------|----------|----------------------------:|---------:|------:|------:|--------:|----------:|------------------|",
        ]
    )
    for comparison in comparisons:
        target = comparison.target
        baseline = comparison.baseline
        lines.append(
            "| "
            + " | ".join(
                [
                    format_cell(comparison.dataset),
                    format_cell(comparison.sequence),
                    metric_pair(target.mean_m if target else None, baseline.mean_m if baseline else None),
                    metric_pair(target.median_m if target else None, baseline.median_m if baseline else None),
                    metric_pair(target.p95_m if target else None, baseline.p95_m if baseline else None),
                    metric_pair(target.max_m if target else None, baseline.max_m if baseline else None),
                    metric_pair(
                        float(target.samples) if target and target.samples is not None else None,
                        float(baseline.samples) if baseline and baseline.samples is not None else None,
                        precision=0,
                    ),
                    metric_pair(
                        target.matched_seconds if target else None,
                        baseline.matched_seconds if baseline else None,
                        precision=2,
                    ),
                    format_cell(format_pending_rows(comparison) or "none"),
                ]
            )
            + " |"
        )

    lines.extend(["", "## Readout", ""])
    if best_numeric is None or best_numeric.target is None or best_numeric.baseline is None:
        lines.append("- No measured same-case AQUA-SLAM/current comparison exists yet.")
    else:
        blockers = reasons_by_case[best_numeric.case_key]
        lines.append(
            f"- Best numeric row: `{best_numeric.target.system}` on "
            f"`{best_numeric.sequence}` at {format_float(best_numeric.target.rmse_m)} m RMSE."
        )
        lines.append(
            f"- Gap to `{baseline_system}` there: {format_ratio(best_numeric.gap_x)} "
            f"({format_percent(best_numeric.improvement_to_tie_percent)} RMSE reduction to tie)."
        )
        if blockers:
            lines.append(
                "- It is not a superiority claim because: "
                + "; ".join(blockers[:4])
                + ("." if len(blockers) <= 4 else "; ...")
            )
        elif best_numeric.gap_x <= 1.0:
            lines.append("- This row passes the configured evidence gates.")
    if best_claimable is None:
        lines.append("- No claimable win is available under the configured gates.")
    else:
        lines.append(
            f"- Best claimable win: `{best_claimable.target.system}` on "
            f"`{best_claimable.sequence}` at {format_ratio(best_claimable.gap_x)}."
        )

    lines.extend(
        [
            "",
            "## GitHub-Safe Summary",
            "",
            "| Status | Best current | Current RMSE m | AQUA-SLAM RMSE m | Gap | Claim |",
            "|--------|--------------|---------------:|-----------------:|----:|-------|",
        ]
    )
    if best_numeric is None or best_numeric.target is None or best_numeric.baseline is None:
        lines.append("| blocked | TBD | TBD | TBD | TBD | Need same-case measured rows first. |")
    else:
        blockers = reasons_by_case[best_numeric.case_key]
        claim = (
            "claimable"
            if best_numeric.gap_x <= 1.0 and not blockers
            else "not claimable yet"
        )
        lines.append(
            "| "
            + " | ".join(
                [
                    verdict(best_numeric, blockers),
                    format_cell(f"{best_numeric.target.system} / {best_numeric.sequence}"),
                    format_float(best_numeric.target.rmse_m),
                    format_float(best_numeric.baseline.rmse_m),
                    format_ratio(best_numeric.gap_x),
                    claim,
                ]
            )
            + " |"
        )

    lines.extend(
        [
            "",
            "## Practical PR Order",
            "",
            "1. Produce a held-out Tank Medium AQUA-SLAM row and matching current row with enough samples.",
            "2. Add P95 to benchmark rows when raw error vectors are available.",
            "3. Only after the held-out table exists, tune the visual/DVL frontend against the largest remaining metric gap.",
        ]
    )
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "markdown",
        nargs="+",
        type=Path,
        help="Markdown files containing benchmark tables.",
    )
    parser.add_argument("--baseline-system", default="AQUA-SLAM")
    parser.add_argument(
        "--target-prefix",
        action="append",
        default=["aqua_"],
        help="System-name prefix to compare against AQUA-SLAM. May be repeated.",
    )
    parser.add_argument("--min-baseline-samples", type=int, default=10)
    parser.add_argument("--min-baseline-matched-s", type=float, default=10.0)
    parser.add_argument("--min-target-samples", type=int, default=10)
    parser.add_argument("--min-target-matched-s", type=float, default=10.0)
    parser.add_argument(
        "--no-require-held-out-note",
        dest="require_held_out_note",
        action="store_false",
        help="Do not block claims when the target note lacks explicit held-out wording.",
    )
    parser.set_defaults(require_held_out_note=True)
    parser.add_argument(
        "--fail-without-claimable-win",
        action="store_true",
        help=(
            "Return exit code 1 unless at least one target beats AQUA-SLAM "
            "and passes every configured evidence gate."
        ),
    )
    parser.add_argument(
        "--fail-on-diagnostic-win",
        action="store_true",
        help="Return exit code 1 when a numeric win is diagnostic.",
    )
    parser.add_argument("--out", type=Path, help="Optional Markdown output path.")
    return parser.parse_args(argv)


def gate_failure_messages(
    comparisons: list[Comparison],
    args: argparse.Namespace,
) -> list[str]:
    reasons_by_case = {
        comparison.case_key: comparison_reasons(comparison, args)
        for comparison in comparisons
    }
    failures: list[str] = []
    wins = claimable_wins(comparisons, reasons_by_case)
    blocked_wins = unclaimable_wins(comparisons, reasons_by_case)
    diag_wins = diagnostic_wins(comparisons, reasons_by_case)

    if args.fail_without_claimable_win and not wins:
        if blocked_wins:
            best = min(blocked_wins, key=lambda item: item.gap_x)
            assert best.target is not None
            blockers = "; ".join(reasons_by_case[best.case_key])
            failures.append(
                "no claimable AQUA-SLAM win; best numeric win is "
                f"{best.target.system} on {best.sequence} at {format_ratio(best.gap_x)} "
                f"but blocked by: {blockers}"
            )
        else:
            failures.append("no claimable AQUA-SLAM win; no measured target beats the baseline")

    if args.fail_on_diagnostic_win and diag_wins:
        best = min(diag_wins, key=lambda item: item.gap_x)
        assert best.target is not None
        failures.append(
            "diagnostic AQUA-SLAM win present: "
            f"{best.target.system} on {best.sequence} at {format_ratio(best.gap_x)}"
        )
    return failures


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rows: list[MetricRow] = []
    try:
        for path in args.markdown:
            rows.extend(parse_metric_rows(path.read_text(encoding="utf-8")))
    except OSError as exc:
        print(f"failed to read benchmark markdown: {exc}", file=sys.stderr)
        return 2

    comparisons = collect_comparisons(
        rows,
        baseline_system=args.baseline_system,
        target_prefixes=args.target_prefix,
    )
    text = format_report(
        comparisons,
        source_paths=args.markdown,
        baseline_system=args.baseline_system,
        target_prefixes=args.target_prefix,
        args=args,
    )
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"wrote AQUA-SLAM head-to-head report to {args.out}")
    else:
        print(text)

    failures = gate_failure_messages(comparisons, args)
    for failure in failures:
        print(f"AQUA-SLAM claim gate failed: {failure}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
