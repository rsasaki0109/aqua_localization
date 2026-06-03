"""Tests for diagnose_mbes_selected_loop_coverage.py."""

import importlib.util
from pathlib import Path
import sys


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "diagnose_mbes_selected_loop_coverage.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "diagnose_mbes_selected_loop_coverage", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, text: str):
    path.write_text(text.strip() + "\n", encoding="utf-8")


def test_format_report_passes_when_exact_pairs_are_accepted(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    write_csv(
        selected_path,
        """
timestamp,current_id,candidate_id,status
10.0,100,4,accepted
11.0,110,5,accepted
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad
10.0,100,4,1,accepted,0.1,0.5,0.05
11.0,110,5,1,accepted,0.2,0.6,0.06
""",
    )

    selected = module.read_selected(selected_path)
    status_rows = module.read_status(status_path)
    rows = module.coverage_rows(selected, status_rows, timestamp_window_s=0.5)
    text = module.format_report(
        selected_path, status_path, selected, status_rows, rows, 0.5, max_rows=20
    )

    assert "- Allowlisted loops: 2" in text
    assert "- Exact pairs observed: 2" in text
    assert "- Exact pairs accepted: 2" in text
    assert "PASS: every selected loop was observed and accepted by exact ID." in text


def test_format_report_flags_timestamp_candidate_drift(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    write_csv(
        selected_path,
        """
timestamp,current_id,candidate_id,status
10.0,100,4,accepted
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad
10.2,101,7,0,loop selection rejected,0.1,0.5,0.05
""",
    )

    selected = module.read_selected(selected_path)
    status_rows = module.read_status(status_path)
    rows = module.coverage_rows(selected, status_rows, timestamp_window_s=0.5)
    text = module.format_report(
        selected_path, status_path, selected, status_rows, rows, 0.5, max_rows=20
    )

    assert "- Exact pairs missing: 1" in text
    assert "- Allowlist rows with accepted-looking candidate in time window: 1" in text
    assert "DRIFT: replay produced accepted-looking candidates near selected source times" in text
    assert "| 1 | 10.000 | 4 -> 100 | missing | 0.200 | 7 -> 101 | loop selection rejected | 1 | 1 |" in text
    assert "## Accepted-Looking Rows Rejected By Selection" in text


def test_main_writes_report(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    out = tmp_path / "coverage.md"
    write_csv(
        selected_path,
        """
timestamp,current_id,candidate_id
10.0,100,4
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_id,candidate_id,accepted,status
12.0,120,4294967295,0,no candidate submaps
""",
    )

    rc = module.main(
        [
            "--selected",
            str(selected_path),
            "--status",
            str(status_path),
            "--out",
            str(out),
            "--timestamp-window-s",
            "0.5",
        ]
    )

    assert rc == 0
    text = out.read_text(encoding="utf-8")
    assert "MISS: selected loop source times did not overlap replay candidate generation." in text


def test_empty_selected_csv_is_reported_as_empty(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    write_csv(
        selected_path,
        """
timestamp,current_id,candidate_id
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_id,candidate_id,accepted,status
10.0,100,4,0,loop selection rejected
""",
    )

    selected = module.read_selected(selected_path)
    status_rows = module.read_status(status_path)
    rows = module.coverage_rows(selected, status_rows, timestamp_window_s=1.0)
    text = module.format_report(
        selected_path, status_path, selected, status_rows, rows, 1.0, max_rows=20
    )

    assert "- Allowlisted loops: 0" in text
    assert "EMPTY: the selected loop CSV has no data rows." in text
