"""Tests for MBES loop candidate visual-audit report generation."""

import csv
import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "audit_mbes_loop_candidates.py"


def load_module():
    spec = importlib.util.spec_from_file_location("audit_mbes_loop_candidates", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_status_csv(path: Path):
    fields = [
        "timestamp",
        "frame_id",
        "current_id",
        "candidate_id",
        "accepted",
        "converged",
        "fitness_score",
        "correction_translation_m",
        "correction_rotation_rad",
        "descriptor_centroid_distance_m",
        "descriptor_extent_ratio",
        "descriptor_point_count_ratio",
        "status",
    ]
    rows = [
        {
            "timestamp": "1.0",
            "frame_id": "map",
            "current_id": "70",
            "candidate_id": "10",
            "accepted": "1",
            "converged": "1",
            "fitness_score": "0.10",
            "correction_translation_m": "0.20",
            "correction_rotation_rad": "0.01",
            "descriptor_centroid_distance_m": "0.5",
            "descriptor_extent_ratio": "1.2",
            "descriptor_point_count_ratio": "0.9",
            "status": "accepted",
        },
        {
            "timestamp": "2.0",
            "frame_id": "map",
            "current_id": "64",
            "candidate_id": "42",
            "accepted": "1",
            "converged": "1",
            "fitness_score": "1.80",
            "correction_translation_m": "4.70",
            "correction_rotation_rad": "0.42",
            "descriptor_centroid_distance_m": "1.0",
            "descriptor_extent_ratio": "6.0",
            "descriptor_point_count_ratio": "0.45",
            "status": "accepted",
        },
        {
            "timestamp": "3.0",
            "frame_id": "map",
            "current_id": "80",
            "candidate_id": str(2**32 - 1),
            "accepted": "0",
            "converged": "0",
            "fitness_score": "nan",
            "correction_translation_m": "nan",
            "correction_rotation_rad": "nan",
            "descriptor_centroid_distance_m": "nan",
            "descriptor_extent_ratio": "nan",
            "descriptor_point_count_ratio": "nan",
            "status": "no candidate submaps",
        },
        {
            "timestamp": "4.0",
            "frame_id": "map",
            "current_id": "81",
            "candidate_id": "20",
            "accepted": "0",
            "converged": "0",
            "fitness_score": "2.5",
            "correction_translation_m": "1.0",
            "correction_rotation_rad": "0.1",
            "descriptor_centroid_distance_m": "0.2",
            "descriptor_extent_ratio": "1.0",
            "descriptor_point_count_ratio": "0.8",
            "status": "fitness score exceeds gate",
        },
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_accepted_audit_rows_prioritize_near_gate_candidate(tmp_path):
    module = load_module()
    path = tmp_path / "status.csv"
    write_status_csv(path)
    rows = module.read_loop_status_csv(path)

    class Args:
        max_fitness = 2.0
        max_translation_m = 5.0
        max_rotation_rad = 0.5
        min_keyframe_separation = 20
        descriptor_extent_warn = 5.0
        descriptor_point_ratio_warn = 0.5

    audit_rows = module.accepted_audit_rows(rows, Args)

    assert len(audit_rows) == 2
    assert audit_rows[0].row.current_id == 64
    assert audit_rows[0].priority == "high"
    assert "fitness near gate" in audit_rows[0].flags
    assert "translation near gate" in audit_rows[0].flags
    assert "large extent ratio" in audit_rows[0].flags
    assert audit_rows[1].priority == "low"


def test_report_contains_summary_tables(tmp_path):
    module = load_module()
    path = tmp_path / "status.csv"
    write_status_csv(path)

    args = module.parse_args(["--csv", str(path), "--max-accepted", "2"])
    text = module.format_report(module.read_loop_status_csv(path), args)

    assert "# MBES Loop Candidate Visual Audit" in text
    assert "Accepted loops: 2" in text
    assert "Rejected candidates: 1" in text
    assert "No-candidate statuses: 1" in text
    assert "42 -> 64" in text
    assert "TODO: inspect accepted marker geometry" in text
    assert "fitness score exceeds gate" in text


def test_cli_writes_audit_report(tmp_path):
    csv_path = tmp_path / "status.csv"
    out = tmp_path / "audit.md"
    write_status_csv(csv_path)

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--csv",
            str(csv_path),
            "--out",
            str(out),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "wrote audit report" in proc.stdout
    text = out.read_text(encoding="utf-8")
    assert "MBES Loop Candidate Visual Audit" in text
    assert "42 -> 64" in text
