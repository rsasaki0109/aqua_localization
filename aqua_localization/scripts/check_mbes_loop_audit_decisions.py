#!/usr/bin/env python3
"""Check MBES loop visual-audit decision worksheets."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import math
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
    keyframe_gap: int | None
    fitness_score: float
    correction_translation_m: float
    correction_rotation_rad: float
    descriptor_centroid_distance_m: float
    descriptor_extent_ratio: float
    descriptor_point_count_ratio: float
    flags: tuple[str, ...]
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


def parse_int(value: str) -> int | None:
    try:
        return int(value.strip())
    except (AttributeError, ValueError):
        return None


def parse_float(value: str) -> float:
    try:
        return float(value.strip())
    except (AttributeError, ValueError):
        return math.nan


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    if value == 0.0:
        return "0"
    return f"{value:.{precision}f}".rstrip("0").rstrip(".")


def format_optional(value: float | None) -> str:
    return format_float(value if value is not None else math.nan)


def parse_descriptor(value: str) -> tuple[float, float, float]:
    parts = [parse_float(part) for part in value.split("/")]
    while len(parts) < 3:
        parts.append(math.nan)
    return tuple(parts[:3])


def parse_flags(value: str) -> tuple[str, ...]:
    text = value.strip()
    if not text or text.lower() == "none":
        return ()
    return tuple(flag.strip() for flag in text.split(",") if flag.strip())


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
            descriptor = parse_descriptor(values.get("Descriptor c/e/r", ""))
            rows.append(
                DecisionRow(
                    line_number=row_index,
                    rank=values.get("Rank", ""),
                    priority=values.get("Priority", ""),
                    candidate_current=values.get("Candidate -> Current", ""),
                    keyframe_gap=parse_int(values.get("Gap", "")),
                    fitness_score=parse_float(values.get("Fitness", "")),
                    correction_translation_m=parse_float(
                        values.get("Correction m", "")
                    ),
                    correction_rotation_rad=parse_float(
                        values.get("Rotation rad", "")
                    ),
                    descriptor_centroid_distance_m=descriptor[0],
                    descriptor_extent_ratio=descriptor[1],
                    descriptor_point_count_ratio=descriptor[2],
                    flags=parse_flags(values.get("Flags", "")),
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


def row_is_actionable(row: DecisionRow) -> bool:
    return (
        not is_todo(row.decision)
        and row.decision.strip().lower() in VALID_DECISIONS
        and not is_todo(row.reviewer_note)
        and not is_todo(row.action)
        and row.action.strip().lower() in VALID_ACTIONS
    )


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


def finite_values(rows: list[DecisionRow], attr: str) -> list[float]:
    values = [getattr(row, attr) for row in rows]
    return [value for value in values if math.isfinite(value)]


def max_or_none(values: list[float]) -> float | None:
    return max(values) if values else None


def min_or_none(values: list[float]) -> float | None:
    return min(values) if values else None


def describe_threshold_candidate(
    label: str,
    rows: list[DecisionRow],
    action: str,
    attr: str,
) -> str:
    rejected = [
        row
        for row in rows
        if row.decision.strip().lower() == "reject"
        and row.action.strip().lower() == action
    ]
    kept = [row for row in rows if row.decision.strip().lower() == "keep"]
    rejected_values = finite_values(rejected, attr)
    kept_values = finite_values(kept, attr)
    if not rejected_values:
        return f"- {label}: no `{action}` rejected rows yet."

    lowest_rejected = min_or_none(rejected_values)
    highest_kept = max_or_none(kept_values)
    if highest_kept is not None and lowest_rejected is not None:
        relation = (
            "clear gap"
            if highest_kept < lowest_rejected
            else "overlaps kept rows; inspect before tightening"
        )
        return (
            f"- {label}: lowest rejected `{action}` value "
            f"{format_float(lowest_rejected)}, highest kept value "
            f"{format_float(highest_kept)} ({relation})."
        )
    return (
        f"- {label}: lowest rejected `{action}` value "
        f"{format_float(lowest_rejected if lowest_rejected is not None else math.nan)}; "
        "no kept rows to bound the threshold."
    )


def describe_descriptor_rejects(rows: list[DecisionRow]) -> list[str]:
    rejected = [
        row
        for row in rows
        if row.decision.strip().lower() == "reject"
        and row.action.strip().lower() == "descriptor reject"
    ]
    if not rejected:
        return ["- Descriptor gate: no `descriptor reject` rows yet."]

    centroid = finite_values(rejected, "descriptor_centroid_distance_m")
    extent = finite_values(rejected, "descriptor_extent_ratio")
    point_ratio = finite_values(rejected, "descriptor_point_count_ratio")
    return [
        "- Descriptor gate rejected-row ranges: "
        f"centroid {format_optional(min_or_none(centroid))}-"
        f"{format_optional(max_or_none(centroid))} m, "
        f"extent {format_optional(min_or_none(extent))}-"
        f"{format_optional(max_or_none(extent))}, "
        f"point ratio {format_optional(min_or_none(point_ratio))}-"
        f"{format_optional(max_or_none(point_ratio))}.",
        "- Descriptor threshold changes need at least one kept row near the "
        "same area before promoting the new gate.",
    ]


def format_gate_recommendations(rows: list[DecisionRow]) -> list[str]:
    actionable = [row for row in rows if row_is_actionable(row)]
    action_counts = normalized_counter([row.action for row in actionable])
    decision_counts = normalized_counter([row.decision for row in actionable])
    lines = [
        "## Gate Recommendations",
        "",
        f"- Actionable reviewed rows: {len(actionable)} / {len(rows)}",
        f"- Reviewed keep rows: {decision_counts.get('keep', 0)}",
        f"- Reviewed reject rows: {decision_counts.get('reject', 0)}",
        f"- Reviewed unclear rows: {decision_counts.get('unclear', 0)}",
        "",
    ]
    if len(actionable) < len(rows):
        lines.append(
            "- Recommendation status: blocked until every high-priority row has "
            "a valid decision, reviewer note, and action."
        )
    elif action_counts.get("keep", 0) == len(actionable):
        lines.append("- Recommendation status: no rejected rows; keep current gates.")
    else:
        lines.append(
            "- Recommendation status: candidate gate changes below are audit hints, "
            "not accuracy claims."
        )
    lines.extend(
        [
            "",
            describe_threshold_candidate(
                "Translation gate",
                actionable,
                "tighten translation",
                "correction_translation_m",
            ),
            describe_threshold_candidate(
                "Rotation gate",
                actionable,
                "tighten rotation",
                "correction_rotation_rad",
            ),
            *describe_descriptor_rejects(actionable),
        ]
    )
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
        *format_gate_recommendations(rows),
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
