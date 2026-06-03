"""Tests for remap_mbes_selected_loop_allowlist.py."""

import csv
import importlib.util
from pathlib import Path
import sys


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "remap_mbes_selected_loop_allowlist.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "remap_mbes_selected_loop_allowlist", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, text: str):
    path.write_text(text.strip() + "\n", encoding="utf-8")


def read_rows(path: Path):
    with path.open(newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


def test_remap_picks_best_accepted_looking_target(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    out = tmp_path / "remapped.csv"
    write_csv(
        selected_path,
        """
timestamp,current_id,candidate_id,fitness_score,correction_translation_m,correction_rotation_rad,status
10.0,100,4,0.10,1.0,0.10,accepted
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad
9.9,120,8,0,loop selection rejected,0.11,1.1,0.11
10.1,121,9,0,registration did not converge,0.10,1.0,0.10
10.2,122,10,0,loop selection rejected,2.00,5.0,0.90
""",
    )

    selected = module.read_selected(selected_path)
    status_rows = module.read_status(status_path)
    matches, unmatched = module.match_selected_loops(
        selected, status_rows, timestamp_window_s=0.5
    )
    module.write_remapped_csv(out, matches)

    assert len(matches) == 1
    assert unmatched == []
    rows = read_rows(out)
    assert rows[0]["current_id"] == "120"
    assert rows[0]["candidate_id"] == "8"
    assert rows[0]["source_current_id"] == "100"
    assert rows[0]["source_candidate_id"] == "4"


def test_remap_reports_unmatched_rows(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    out = tmp_path / "remapped.csv"
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
10.0,101,4294967295,0,no candidate submaps
""",
    )

    selected = module.read_selected(selected_path)
    status_rows = module.read_status(status_path)
    matches, unmatched = module.match_selected_loops(
        selected, status_rows, timestamp_window_s=1.0
    )
    module.write_remapped_csv(out, matches)
    text = module.format_report(
        selected_path,
        status_path,
        out,
        selected,
        status_rows,
        matches,
        unmatched,
        timestamp_window_s=1.0,
        max_rows=20,
    )

    assert matches == []
    assert len(unmatched) == 1
    assert "MISS: no selected loops could be remapped" in text
    assert read_rows(out) == []


def test_main_writes_csv_and_report(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    out = tmp_path / "remapped.csv"
    report = tmp_path / "remap.md"
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
timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad
10.2,120,8,0,loop selection rejected,0.1,1.0,0.1
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
            "--report-out",
            str(report),
            "--timestamp-window-s",
            "0.5",
        ]
    )

    assert rc == 0
    assert read_rows(out)[0]["current_id"] == "120"
    assert "- Remapped loops: 1" in report.read_text(encoding="utf-8")
