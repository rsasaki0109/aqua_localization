"""Tests for MBES loop audit decision worksheet checks."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "check_mbes_loop_audit_decisions.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "check_mbes_loop_audit_decisions", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def audit_report(rows: list[str]) -> str:
    return "\n".join(
        [
            "# MBES Loop Candidate Visual Audit",
            "",
            "| Rank | Priority | Candidate -> Current | Gap | Decision | Reviewer note | Action |",
            "|-----:|----------|----------------------|----:|----------|---------------|--------|",
            *rows,
            "",
        ]
    )


def test_parse_and_validate_complete_decision_rows():
    module = load_module()
    rows = module.parse_decision_rows(
        audit_report(
            [
                "| 1 | high | 10 -> 70 | 60 | keep | Plausible revisit | keep |",
                "| 2 | high | 42 -> 64 | 22 | reject | Adjacent duplicate | manual reject |",
                "| 3 | medium | 30 -> 95 | 65 | unclear | Needs bag replay | tighten rotation |",
            ]
        )
    )
    issues = module.validate_rows(rows)

    assert len(rows) == 3
    assert issues == []


def test_validate_rows_reports_todo_and_invalid_values():
    module = load_module()
    rows = module.parse_decision_rows(
        audit_report(
            [
                "| 1 | high | 10 -> 70 | 60 | TODO | TODO: inspect | TODO |",
                "| 2 | high | 42 -> 64 | 22 | maybe | checked | tune |",
            ]
        )
    )
    issues = module.validate_rows(rows)

    assert [issue.issue for issue in issues] == [
        "decision TODO",
        "reviewer note TODO",
        "action TODO",
        "invalid decision `maybe`",
        "invalid action `tune`",
    ]


def test_format_summary_counts_decisions_and_actions(tmp_path):
    module = load_module()
    report = tmp_path / "audit.md"
    report.write_text(
        audit_report(
            [
                "| 1 | high | 10 -> 70 | 60 | keep | Plausible revisit | keep |",
                "| 2 | high | 42 -> 64 | 22 | TODO | TODO: inspect | TODO |",
            ]
        ),
        encoding="utf-8",
    )
    rows = module.read_decision_rows(report)
    summary = module.format_summary(rows, module.validate_rows(rows), report)

    assert "Rows: 2" in summary
    assert "Complete rows: 1" in summary
    assert "Incomplete rows: 1" in summary
    assert "| keep | 1 |" in summary
    assert "| TODO | 1 |" in summary
    assert "decision TODO" in summary


def test_cli_require_complete_fails_on_todo(tmp_path):
    report = tmp_path / "audit.md"
    report.write_text(
        audit_report(["| 1 | high | 10 -> 70 | 60 | TODO | TODO: inspect | TODO |"]),
        encoding="utf-8",
    )

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--report",
            str(report),
            "--require-complete",
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 1
    assert "Incomplete rows: 1" in proc.stdout


def test_cli_writes_summary_for_complete_report(tmp_path):
    report = tmp_path / "audit.md"
    out = tmp_path / "summary.md"
    report.write_text(
        audit_report(["| 1 | high | 10 -> 70 | 60 | keep | Good revisit | keep |"]),
        encoding="utf-8",
    )

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--report",
            str(report),
            "--out",
            str(out),
            "--require-complete",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "wrote decision summary" in proc.stdout
    assert "Complete rows: 1" in out.read_text(encoding="utf-8")
