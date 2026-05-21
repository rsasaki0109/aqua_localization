#!/usr/bin/env python3
"""Render a real-bag benchmark manifest into a compact Markdown run sheet."""

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


REQUIRED_CASE_FIELDS = {
    "id",
    "dataset",
    "sequence",
    "status",
    "comparison_group",
    "target_system",
    "baselines",
    "inputs",
    "reference",
    "metrics",
    "artifacts",
    "next_step",
    "fairness_notes",
}


@dataclass(frozen=True)
class EvaluationCase:
    case_id: str
    dataset: str
    sequence: str
    status: str
    comparison_group: str
    target_system: str
    baselines: tuple[str, ...]
    inputs: tuple[str, ...]
    reference: str
    metrics: tuple[str, ...]
    artifacts: tuple[str, ...]
    command: str
    next_step: str
    fairness_notes: tuple[str, ...]


def as_string_list(value, field: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"{field} must be a list of strings")
    return tuple(item for item in value if item.strip())


def load_manifest(path: Path) -> list[EvaluationCase]:
    with path.open(encoding="utf-8") as fp:
        data = json.load(fp)

    raw_cases = data.get("cases") if isinstance(data, dict) else None
    if not isinstance(raw_cases, list):
        raise ValueError("manifest must contain a top-level 'cases' list")

    cases = []
    seen_ids = set()
    for index, raw in enumerate(raw_cases):
        if not isinstance(raw, dict):
            raise ValueError(f"case {index} must be an object")
        missing = sorted(REQUIRED_CASE_FIELDS.difference(raw))
        if missing:
            raise ValueError(f"case {raw.get('id', index)} missing fields: {', '.join(missing)}")
        case_id = str(raw["id"])
        if case_id in seen_ids:
            raise ValueError(f"duplicate case id: {case_id}")
        seen_ids.add(case_id)
        cases.append(
            EvaluationCase(
                case_id=case_id,
                dataset=str(raw["dataset"]),
                sequence=str(raw["sequence"]),
                status=str(raw["status"]),
                comparison_group=str(raw["comparison_group"]),
                target_system=str(raw["target_system"]),
                baselines=as_string_list(raw["baselines"], "baselines"),
                inputs=as_string_list(raw["inputs"], "inputs"),
                reference=str(raw["reference"]),
                metrics=as_string_list(raw["metrics"], "metrics"),
                artifacts=as_string_list(raw["artifacts"], "artifacts"),
                command=str(raw.get("command", "")),
                next_step=str(raw["next_step"]),
                fairness_notes=as_string_list(raw["fairness_notes"], "fairness_notes"),
            )
        )
    return cases


def escape_cell(value: str) -> str:
    return value.replace("|", "\\|").replace("\n", " ")


def comma_join(values: Iterable[str]) -> str:
    return ", ".join(values) if values else "TBD"


def filter_cases(cases: Iterable[EvaluationCase], statuses: set[str]) -> list[EvaluationCase]:
    if not statuses:
        return list(cases)
    lowered = {status.lower() for status in statuses}
    return [case for case in cases if case.status.lower() in lowered]


def format_summary_table(cases: list[EvaluationCase]) -> list[str]:
    lines = [
        "| Case | Status | Target | Baselines | Metrics | Next step |",
        "|------|--------|--------|-----------|---------|-----------|",
    ]
    for case in cases:
        lines.append(
            "| "
            + " | ".join(
                escape_cell(cell)
                for cell in [
                    f"{case.dataset} `{case.sequence}`",
                    case.status,
                    case.target_system,
                    comma_join(case.baselines),
                    comma_join(case.metrics),
                    case.next_step,
                ]
            )
            + " |"
        )
    return lines


def format_case_detail(case: EvaluationCase) -> list[str]:
    lines = [
        f"### {case.dataset} `{case.sequence}`",
        "",
        f"- Status: `{case.status}`",
        f"- Comparison group: {case.comparison_group}",
        f"- Target system: `{case.target_system}`",
        f"- Baselines: {comma_join(f'`{baseline}`' for baseline in case.baselines)}",
        f"- Inputs: {comma_join(case.inputs)}",
        f"- Reference: {case.reference}",
        f"- Artifacts: {comma_join(f'`{artifact}`' for artifact in case.artifacts)}",
    ]
    if case.command.strip():
        lines.extend(["", "```bash", case.command, "```"])
    else:
        lines.extend(["", "_No replay command is pinned yet._"])

    lines.extend(["", "Fairness notes:"])
    lines.extend(f"- {note}" for note in case.fairness_notes)
    return lines


def format_report(cases: list[EvaluationCase], source: Path | None = None) -> str:
    lines = [
        "# Real-Bag Evaluation Run Sheet",
        "",
        "This report is generated from the benchmark manifest. Use it to keep",
        "real-data comparisons reproducible before turning results into README or",
        "paper claims.",
        "",
    ]
    if source is not None:
        lines.extend([f"Manifest: `{source}`", ""])

    if not cases:
        lines.extend(["No cases matched the selected filters.", ""])
        return "\n".join(lines)

    lines.extend(format_summary_table(cases))
    lines.extend(["", "## Case Details", ""])
    for index, case in enumerate(cases):
        if index:
            lines.append("")
        lines.extend(format_case_detail(case))
    lines.append("")
    return "\n".join(lines)


def readiness_failures(cases: Iterable[EvaluationCase]) -> list[str]:
    failures = []
    for case in cases:
        if case.status.lower() in {"measured", "ready"} and not case.command.strip():
            failures.append(f"{case.case_id}: measured/ready case has no command")
        if not case.baselines:
            failures.append(f"{case.case_id}: no baseline listed")
        if not case.metrics:
            failures.append(f"{case.case_id}: no metric listed")
    return failures


def doc_artifact_failures(cases: Iterable[EvaluationCase], repo_root: Path) -> list[str]:
    failures = []
    for case in cases:
        for artifact in case.artifacts:
            if not artifact.startswith("docs/"):
                continue
            path = repo_root / artifact
            if not path.exists():
                failures.append(f"{case.case_id}: missing documented artifact {artifact}")
    return failures


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Render a real-bag benchmark manifest into Markdown."
    )
    parser.add_argument("manifest", type=Path, help="JSON benchmark manifest.")
    parser.add_argument("--out", type=Path, help="Optional Markdown output path.")
    parser.add_argument(
        "--status",
        action="append",
        default=[],
        help="Only include cases with this status. Can be repeated.",
    )
    parser.add_argument(
        "--check-ready",
        action="store_true",
        help="Fail if measured/ready cases are missing commands or cases lack metrics/baselines.",
    )
    parser.add_argument(
        "--check-doc-artifacts",
        action="store_true",
        help="Fail if artifact entries that look like docs/... paths do not exist.",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=Path.cwd(),
        help="Repository root used by --check-doc-artifacts. Defaults to the current directory.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        cases = filter_cases(load_manifest(args.manifest), set(args.status))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"benchmark manifest error: {exc}", file=sys.stderr)
        return 2

    if args.check_ready:
        failures = readiness_failures(cases)
        if failures:
            for failure in failures:
                print(f"benchmark manifest check failed: {failure}", file=sys.stderr)
            return 2

    if args.check_doc_artifacts:
        failures = doc_artifact_failures(cases, args.repo_root)
        if failures:
            for failure in failures:
                print(f"benchmark manifest check failed: {failure}", file=sys.stderr)
            return 2

    report = format_report(cases, source=args.manifest)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
    else:
        print(report, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
