#!/usr/bin/env python3
"""Check MBES loop visual-audit decision worksheets."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
import sys


VALID_DECISIONS = {"keep", "reject", "unclear"}
VALID_ACTIONS = {
    "keep",
    "tighten translation",
    "tighten rotation",
    "descriptor reject",
    "manual reject",
}


@dataclass(frozen=True)
class DecisionRow:
    line_number: int
    rank: str
    priority: str
    candidate_current: str
    decision: str
    reviewer_note: str
    action: str


@dataclass(frozen=True)
class RowIssue:
    line_number: int
    rank: str
    candidate_current: str
    issue: str


def split_markdown_row(line: str) -> list[str]:
    text = line.strip()
    if not text.startswith("|") or not text.endswith("|"):
        return []
    return [cell.strip() for cell in text.strip("|").split("|")]


def is_separator_row(cells: list[str]) -> bool:
    if not cells:
        return False
    return all(cell and set(cell) <= {"-", ":", " "} for cell in cells)


def is_todo(value: str) -> bool:
    text = value.strip()
    return not text or text.lower() == "todo" or text.lower().startswith("todo:")


def parse_decision_rows(text: str) -> list[DecisionRow]:
    lines = text.splitlines()
    for index, line in enumerate(lines):
        header = split_markdown_row(line)
        if not header:
            continue
        required = {"Decision", "Reviewer note", "Action"}
        if not required.issubset(set(header)):
            continue

        rows: list[DecisionRow] = []
        for row_index, row_line in enumerate(lines[index + 1 :], start=index + 2):
            cells = split_markdown_row(row_line)
            if not cells:
                if rows:
                    break
                continue
            if is_separator_row(cells):
                continue
            if len(cells) != len(header):
                raise ValueError(
                    f"line {row_index}: expected {len(header)} table cells, got {len(cells)}"
                )
            values = dict(zip(header, cells, strict=True))
            rows.append(
                DecisionRow(
                    line_number=row_index,
                    rank=values.get("Rank", ""),
                    priority=values.get("Priority", ""),
                    candidate_current=values.get("Candidate -> Current", ""),
                    decision=values["Decision"],
                    reviewer_note=values["Reviewer note"],
                    action=values["Action"],
                )
            )
        return rows
    raise ValueError("no audit table with Decision, Reviewer note, and Action columns")


def read_decision_rows(path: Path) -> list[DecisionRow]:
    return parse_decision_rows(path.read_text(encoding="utf-8"))


def validate_rows(rows: list[DecisionRow]) -> list[RowIssue]:
    issues: list[RowIssue] = []
    for row in rows:
        decision = row.decision.strip().lower()
        action = row.action.strip().lower()
        if is_todo(row.decision):
            issues.append(row_issue(row, "decision TODO"))
        elif decision not in VALID_DECISIONS:
            issues.append(row_issue(row, f"invalid decision `{row.decision}`"))
        if is_todo(row.reviewer_note):
            issues.append(row_issue(row, "reviewer note TODO"))
        if is_todo(row.action):
            issues.append(row_issue(row, "action TODO"))
        elif action not in VALID_ACTIONS:
            issues.append(row_issue(row, f"invalid action `{row.action}`"))
    return issues


def row_issue(row: DecisionRow, issue: str) -> RowIssue:
    return RowIssue(
        line_number=row.line_number,
        rank=row.rank,
        candidate_current=row.candidate_current,
        issue=issue,
    )


def normalized_counter(values: list[str]) -> Counter[str]:
    counter: Counter[str] = Counter()
    for value in values:
        key = "TODO" if is_todo(value) else value.strip().lower()
        counter[key or "blank"] += 1
    return counter


def format_count_table(title: str, counts: Counter[str]) -> list[str]:
    lines = [
        f"## {title}",
        "",
        "| Value | Count |",
        "|-------|------:|",
    ]
    for value, count in counts.most_common():
        lines.append(f"| {value} | {count} |")
    return lines


def format_issue_table(issues: list[RowIssue]) -> list[str]:
    lines = [
        "## Blocking Rows",
        "",
        "| Line | Rank | Candidate -> Current | Issue |",
        "|-----:|-----:|----------------------|-------|",
    ]
    if not issues:
        lines.append("| n/a | n/a | n/a | none |")
        return lines
    for issue in issues:
        lines.append(
            f"| {issue.line_number} | {issue.rank or 'n/a'} | "
            f"{issue.candidate_current or 'n/a'} | {issue.issue} |"
        )
    return lines


def format_summary(rows: list[DecisionRow], issues: list[RowIssue], source: Path) -> str:
    issue_rows = {issue.line_number for issue in issues}
    incomplete_rows = {
        issue.line_number
        for issue in issues
        if issue.issue in {"decision TODO", "reviewer note TODO", "action TODO"}
    }
    invalid_rows = {
        issue.line_number
        for issue in issues
        if issue.issue.startswith("invalid ")
    }
    lines = [
        "# MBES Loop Audit Decision Summary",
        "",
        f"- Source report: `{source}`",
        f"- Rows: {len(rows)}",
        f"- Complete rows: {len(rows) - len(issue_rows)}",
        f"- Incomplete rows: {len(incomplete_rows)}",
        f"- Invalid rows: {len(invalid_rows)}",
        "",
        *format_count_table(
            "Decision Counts",
            normalized_counter([row.decision for row in rows]),
        ),
        "",
        *format_count_table(
            "Action Counts",
            normalized_counter([row.action for row in rows]),
        ),
        "",
        *format_issue_table(issues),
        "",
        "## Allowed Values",
        "",
        f"- Decision: {', '.join(f'`{value}`' for value in sorted(VALID_DECISIONS))}",
        f"- Action: {', '.join(f'`{value}`' for value in sorted(VALID_ACTIONS))}",
        "",
    ]
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path, help="Audit Markdown report")
    parser.add_argument("--out", type=Path, help="Optional Markdown summary output path")
    parser.add_argument(
        "--require-complete",
        action="store_true",
        help="Return nonzero when any row is TODO, blank, or invalid",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        rows = read_decision_rows(args.report)
        issues = validate_rows(rows)
    except (OSError, ValueError) as exc:
        print(f"failed to check MBES loop audit decisions: {exc}", file=sys.stderr)
        return 2

    text = format_summary(rows, issues, args.report)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"wrote decision summary to {args.out}")
    else:
        print(text)

    if args.require_complete and issues:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
