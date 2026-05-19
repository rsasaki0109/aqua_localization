#!/usr/bin/env python3
"""Summarize RMSE gaps from Markdown benchmark tables.

The report is intentionally table-driven: it reads the same Markdown rows used
in docs and README snippets, then computes how far a target system is from a
baseline on the same dataset, sequence, and alignment.
"""

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REQUIRED_COLUMNS = {
    "Dataset",
    "Sequence",
    "System",
    "Alignment",
    "Samples",
    "Matched s",
    "RMSE m",
}


@dataclass(frozen=True)
class BenchmarkRow:
    dataset: str
    sequence: str
    system: str
    alignment: str
    samples: int | None
    matched_seconds: float | None
    rmse_m: float | None
    note: str


@dataclass(frozen=True)
class GapRow:
    dataset: str
    sequence: str
    alignment: str
    target: BenchmarkRow
    baseline: BenchmarkRow

    @property
    def ratio(self) -> float:
        if self.target.rmse_m is None or self.baseline.rmse_m is None:
            return math.nan
        if self.baseline.rmse_m == 0.0:
            return math.inf
        return self.target.rmse_m / self.baseline.rmse_m

    @property
    def improvement_to_tie_percent(self) -> float:
        if self.target.rmse_m is None or self.baseline.rmse_m is None:
            return math.nan
        if self.target.rmse_m == 0.0:
            return 0.0
        return max(0.0, (1.0 - self.baseline.rmse_m / self.target.rmse_m) * 100.0)


def split_markdown_row(line: str) -> list[str]:
    stripped = line.strip()
    if not stripped.startswith("|") or not stripped.endswith("|"):
        return []
    return [cell.strip() for cell in stripped.strip("|").split("|")]


def is_separator_row(cells: Iterable[str]) -> bool:
    cells = list(cells)
    if not cells:
        return False
    return all(re.fullmatch(r":?-{3,}:?", cell.replace(" ", "")) for cell in cells)


def normalize_text(value: str) -> str:
    return value.strip().strip("`").replace("\\|", "|")


def parse_float(value: str) -> float | None:
    text = normalize_text(value)
    if text.upper() in {"", "TBD", "N/A", "NA", "NONE"}:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def parse_int(value: str) -> int | None:
    number = parse_float(value)
    if number is None:
        return None
    return int(number)


def rows_from_table(header: list[str], rows: list[list[str]]) -> list[BenchmarkRow]:
    if not REQUIRED_COLUMNS.issubset(set(header)):
        return []

    index = {name: header.index(name) for name in header}
    parsed = []
    for cells in rows:
        if len(cells) < len(header):
            continue
        rmse = parse_float(cells[index["RMSE m"]])
        if rmse is None:
            continue
        note = cells[index["Note"]] if "Note" in index else ""
        parsed.append(
            BenchmarkRow(
                dataset=normalize_text(cells[index["Dataset"]]),
                sequence=normalize_text(cells[index["Sequence"]]),
                system=normalize_text(cells[index["System"]]),
                alignment=normalize_text(cells[index["Alignment"]]),
                samples=parse_int(cells[index["Samples"]]),
                matched_seconds=parse_float(cells[index["Matched s"]]),
                rmse_m=rmse,
                note=normalize_text(note),
            )
        )
    return parsed


def parse_markdown_benchmark_rows(text: str) -> list[BenchmarkRow]:
    lines = text.splitlines()
    parsed: list[BenchmarkRow] = []
    i = 0
    while i < len(lines) - 1:
        header = split_markdown_row(lines[i])
        separator = split_markdown_row(lines[i + 1])
        if header and is_separator_row(separator):
            table_rows = []
            i += 2
            while i < len(lines):
                cells = split_markdown_row(lines[i])
                if not cells or is_separator_row(cells):
                    break
                table_rows.append(cells)
                i += 1
            parsed.extend(rows_from_table(header, table_rows))
        else:
            i += 1
    return parsed


def dedupe_rows(rows: Iterable[BenchmarkRow]) -> list[BenchmarkRow]:
    seen = set()
    unique = []
    for row in rows:
        key = (
            row.dataset,
            row.sequence,
            row.system,
            row.alignment,
            row.samples,
            row.matched_seconds,
            row.rmse_m,
            row.note,
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(row)
    return unique


def matching_system(row: BenchmarkRow, system: str) -> bool:
    return row.system.lower() == system.strip("`").lower()


def best_by_case(rows: Iterable[BenchmarkRow], system: str) -> dict[tuple[str, str, str], BenchmarkRow]:
    best: dict[tuple[str, str, str], BenchmarkRow] = {}
    for row in rows:
        if not matching_system(row, system) or row.rmse_m is None:
            continue
        key = (row.dataset, row.sequence, row.alignment)
        if key not in best or row.rmse_m < best[key].rmse_m:
            best[key] = row
    return best


def compute_gaps(rows: Iterable[BenchmarkRow], target_system: str, baseline_system: str) -> list[GapRow]:
    unique = dedupe_rows(rows)
    targets = best_by_case(unique, target_system)
    baselines = best_by_case(unique, baseline_system)
    gaps = []
    for key in sorted(set(targets).intersection(baselines)):
        dataset, sequence, alignment = key
        gaps.append(
            GapRow(
                dataset=dataset,
                sequence=sequence,
                alignment=alignment,
                target=targets[key],
                baseline=baselines[key],
            )
        )
    return gaps


def format_float(value: float, precision: int = 4) -> str:
    if math.isnan(value):
        return "TBD"
    if math.isinf(value):
        return "inf"
    return f"{value:.{precision}f}"


def format_report(gaps: list[GapRow], target_system: str, baseline_system: str) -> str:
    lines = [
        "# Benchmark Gap Report",
        "",
        f"Target system: `{target_system}`",
        f"Baseline system: `{baseline_system}`",
        "",
    ]
    if not gaps:
        lines.extend(
            [
                "No matching dataset/sequence/alignment rows were found.",
                "",
                "Add rows generated by `trajectory_benchmark_row.py` for both systems first.",
            ]
        )
        return "\n".join(lines)

    lines.extend(
        [
            "| Dataset | Sequence | Alignment | Target RMSE m | Baseline RMSE m | Gap x | Improvement to tie | Target samples | Baseline samples |",
            "|---------|----------|-----------|--------------:|----------------:|------:|-------------------:|---------------:|-----------------:|",
        ]
    )
    for gap in gaps:
        lines.append(
            "| "
            + " | ".join(
                [
                    gap.dataset,
                    gap.sequence,
                    gap.alignment,
                    format_float(gap.target.rmse_m),
                    format_float(gap.baseline.rmse_m),
                    format_float(gap.ratio, precision=2),
                    f"{format_float(gap.improvement_to_tie_percent, precision=1)}%",
                    str(gap.target.samples) if gap.target.samples is not None else "TBD",
                    str(gap.baseline.samples) if gap.baseline.samples is not None else "TBD",
                ]
            )
            + " |"
        )

    best_gap = min(gaps, key=lambda gap: gap.ratio)
    lines.extend(
        [
            "",
            "## Readout",
            "",
            (
                f"- Closest case: `{best_gap.dataset}` `{best_gap.sequence}` "
                f"{best_gap.alignment}, {format_float(best_gap.ratio, precision=2)}x "
                f"the baseline RMSE."
            ),
            (
                f"- Required RMSE reduction to tie there: "
                f"{format_float(best_gap.improvement_to_tie_percent, precision=1)}%."
            ),
        ]
    )
    return "\n".join(lines)


def gate_failures(
    gaps: list[GapRow],
    max_gap_x: float | None,
    max_improvement_to_tie_percent: float | None,
) -> list[str]:
    if max_gap_x is None and max_improvement_to_tie_percent is None:
        return []
    if not gaps:
        return ["no matching benchmark gaps were found"]

    failures = []
    for gap in gaps:
        case = f"{gap.dataset} {gap.sequence} {gap.alignment}"
        if max_gap_x is not None and gap.ratio > max_gap_x:
            failures.append(
                f"{case}: gap {gap.ratio:.4f}x exceeds --max-gap-x {max_gap_x:.4f}"
            )
        if (
            max_improvement_to_tie_percent is not None
            and gap.improvement_to_tie_percent > max_improvement_to_tie_percent
        ):
            failures.append(
                f"{case}: improvement-to-tie {gap.improvement_to_tie_percent:.4f}% "
                f"exceeds --max-improvement-to-tie-percent "
                f"{max_improvement_to_tie_percent:.4f}%"
            )
    return failures


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Create a Markdown RMSE gap report from benchmark tables."
    )
    parser.add_argument(
        "markdown",
        nargs="+",
        type=Path,
        help="Markdown files containing trajectory benchmark tables.",
    )
    parser.add_argument("--target-system", required=True, help="System to improve.")
    parser.add_argument("--baseline-system", required=True, help="Baseline system to beat.")
    parser.add_argument("--out", type=Path, default=None, help="Optional Markdown output path.")
    parser.add_argument(
        "--max-gap-x",
        type=float,
        default=None,
        help="Fail with exit code 2 if any target/baseline RMSE ratio exceeds this value.",
    )
    parser.add_argument(
        "--max-improvement-to-tie-percent",
        type=float,
        default=None,
        help="Fail with exit code 2 if any case needs more than this RMSE reduction to tie.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rows = []
    for path in args.markdown:
        rows.extend(parse_markdown_benchmark_rows(path.read_text(encoding="utf-8")))
    gaps = compute_gaps(rows, args.target_system, args.baseline_system)
    report = format_report(gaps, args.target_system, args.baseline_system)

    if args.out is None:
        print(report)
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report + "\n", encoding="utf-8")
    failures = gate_failures(gaps, args.max_gap_x, args.max_improvement_to_tie_percent)
    if failures:
        print("benchmark gap gate failed:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
