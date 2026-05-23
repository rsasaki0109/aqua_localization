"""Tests for analyze_tank_dvl_prior_residuals.py."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "analyze_tank_dvl_prior_residuals.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("analyze_tank_dvl_prior_residuals", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def tum_row(stamp_s: float, xyz):
    return [stamp_s, xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0, 1.0]


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{value:.9f}" for value in row) + "\n")


def write_step_csv(path: Path, rows):
    fieldnames = [
        "start_stamp_s",
        "end_stamp_s",
        "offset_s",
        "dt_s",
        "visual_step_m",
        "prior_step_m",
        "corrected_step_m",
        "visual_prior_length_ratio",
        "visual_prior_direction_cosine",
        "visual_prior_heading_error_deg",
        "dvl_covered",
        "dvl_samples",
        "used_prior",
        "reason",
    ]
    with path.open("w", encoding="utf-8") as fp:
        fp.write(",".join(fieldnames) + "\n")
        for row in rows:
            fp.write(",".join(str(row.get(field, "")) for field in fieldnames) + "\n")


def test_build_residual_rows_groups_prior_usage():
    module = load_module()
    times = np.array([0.0, 1.0, 2.0])
    reference = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
    ])
    visual = np.array([
        [0.0, 0.0, 0.0],
        [1.2, 0.0, 0.0],
        [2.4, 0.0, 0.0],
    ])
    corrected = np.array([
        [0.0, 0.0, 0.0],
        [1.05, 0.0, 0.0],
        [2.5, 0.0, 0.0],
    ])
    step_rows = [
        {
            "visual_step_m": "1.2",
            "prior_step_m": "1.0",
            "corrected_step_m": "1.05",
            "used_prior": "true",
            "dvl_covered": "true",
            "reason": "replace-outlier",
        },
        {
            "visual_step_m": "1.2",
            "prior_step_m": "1.0",
            "corrected_step_m": "1.45",
            "used_prior": "false",
            "dvl_covered": "true",
            "reason": "visual-ok",
        },
    ]

    rows = module.build_residual_rows(
        times,
        visual,
        corrected,
        reference,
        np.array([True, True, True]),
        step_rows,
    )

    assert len(rows) == 2
    assert rows[0].improvement_m == pytest.approx(0.15)
    assert rows[0].used_prior is True
    assert rows[1].improvement_m == pytest.approx(-0.1)
    assert rows[1].reason == "visual-ok"


def test_format_markdown_reports_reason_groups(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        reference=tmp_path / "ref.tum",
        visual_aligned=tmp_path / "visual.tum",
        corrected=tmp_path / "corrected.tum",
        step_csv=tmp_path / "steps.csv",
        csv_out=tmp_path / "residuals.csv",
        top_k=2,
    )
    rows = [
        module.ResidualRow(
            index=1,
            stamp_s=1.0,
            offset_s=1.0,
            visual_error_m=0.2,
            corrected_error_m=0.05,
            improvement_m=0.15,
            visual_step_m=1.2,
            prior_step_m=1.0,
            corrected_step_m=1.05,
            used_prior=True,
            dvl_covered=True,
            reason="replace-outlier",
            direction_cosine=0.9,
            length_ratio=1.2,
            heading_error_deg=5.0,
        ),
        module.ResidualRow(
            index=2,
            stamp_s=2.0,
            offset_s=2.0,
            visual_error_m=0.4,
            corrected_error_m=0.5,
            improvement_m=-0.1,
            visual_step_m=1.2,
            prior_step_m=1.0,
            corrected_step_m=1.45,
            used_prior=False,
            dvl_covered=True,
            reason="visual-ok",
            direction_cosine=0.8,
            length_ratio=1.2,
            heading_error_deg=10.0,
        ),
    ]

    text = module.format_markdown(args, rows)

    assert "# Tank DVL Prior Residual Analysis" in text
    assert "replace-outlier" in text
    assert "visual-ok" in text
    assert "Largest Regressions" in text


def test_format_markdown_labels_smallest_improvements_without_regressions(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        reference=tmp_path / "ref.tum",
        visual_aligned=tmp_path / "visual.tum",
        corrected=tmp_path / "corrected.tum",
        step_csv=tmp_path / "steps.csv",
        csv_out=None,
        top_k=1,
    )
    rows = [
        module.ResidualRow(
            index=1,
            stamp_s=1.0,
            offset_s=1.0,
            visual_error_m=0.2,
            corrected_error_m=0.1,
            improvement_m=0.1,
            visual_step_m=1.0,
            prior_step_m=1.0,
            corrected_step_m=1.0,
            used_prior=True,
            dvl_covered=True,
            reason="replace-outlier",
            direction_cosine=1.0,
            length_ratio=1.0,
            heading_error_deg=0.0,
        )
    ]

    text = module.format_markdown(args, rows)

    assert "Smallest Improvements" in text


def test_run_analysis_writes_csv_and_markdown(tmp_path):
    module = load_module()
    reference = tmp_path / "reference.tum"
    visual = tmp_path / "visual.tum"
    corrected = tmp_path / "corrected.tum"
    step_csv = tmp_path / "steps.csv"
    out = tmp_path / "analysis.md"
    csv_out = tmp_path / "residuals.csv"
    write_tum(reference, [tum_row(float(i), [float(i), 0.0, 0.0]) for i in range(4)])
    write_tum(visual, [tum_row(float(i), [float(i), 0.1 * float(i % 2), 0.0]) for i in range(4)])
    write_tum(corrected, [tum_row(float(i), [float(i), 0.02 * float(i % 2), 0.0]) for i in range(4)])
    write_step_csv(step_csv, [
        {"visual_step_m": 1.0, "prior_step_m": 1.0, "corrected_step_m": 1.0, "used_prior": "true", "dvl_covered": "true", "reason": "replace-outlier"},
        {"visual_step_m": 1.0, "prior_step_m": 1.0, "corrected_step_m": 1.0, "used_prior": "false", "dvl_covered": "true", "reason": "visual-ok"},
        {"visual_step_m": 1.0, "prior_step_m": 1.0, "corrected_step_m": 1.0, "used_prior": "true", "dvl_covered": "true", "reason": "replace-outlier"},
    ])

    rc = module.main([
        "--reference",
        str(reference),
        "--visual-aligned",
        str(visual),
        "--corrected",
        str(corrected),
        "--step-csv",
        str(step_csv),
        "--out",
        str(out),
        "--csv-out",
        str(csv_out),
    ])

    assert rc == 0
    assert "Tank DVL Prior Residual Analysis" in out.read_text(encoding="utf-8")
    assert csv_out.read_text(encoding="utf-8").startswith("index,stamp_s,offset_s")
