"""Tests for analyze_visual_step_errors.py."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "analyze_visual_step_errors.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("analyze_visual_step_errors", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{value:.9f}" for value in row) + "\n")


def tum_row(stamp_s: float, xyz):
    return [stamp_s, xyz[0], xyz[1], xyz[2], 0.0, 0.0, 0.0, 1.0]


def test_build_step_errors_reports_length_and_direction():
    module = load_module()
    times = np.array([0.0, 1.0, 2.0])
    reference = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
    ])
    visual = np.array([
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
    ])

    steps = module.build_step_errors(times, visual, reference, min_reference_step_m=0.0)

    assert len(steps) == 2
    assert steps[0].length_ratio == pytest.approx(2.0)
    assert steps[0].direction_cosine == pytest.approx(1.0)
    assert steps[1].length_ratio == pytest.approx(1.0)
    assert steps[1].direction_cosine == pytest.approx(0.0)
    assert steps[1].heading_error_deg == pytest.approx(90.0)


def test_matched_aligned_positions_removes_global_offset(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    rows_ref = [tum_row(float(i), [float(i), 0.0, 0.0]) for i in range(5)]
    rows_est = [tum_row(float(i), [float(i) + 10.0, 5.0, 0.0]) for i in range(5)]
    write_tum(ref, rows_ref)
    write_tum(est, rows_est)

    _, visual_xyz, ref_xyz = module.matched_aligned_positions(ref, est)

    np.testing.assert_allclose(visual_xyz, ref_xyz, atol=1.0e-9)


def test_run_analysis_writes_csv_and_markdown(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    out = tmp_path / "steps.md"
    csv = tmp_path / "steps.csv"
    write_tum(ref, [tum_row(float(i), [0.1 * float(i), 0.0, 0.0]) for i in range(20)])
    write_tum(est, [tum_row(float(i), [0.12 * float(i), 0.0, 0.0]) for i in range(20)])

    rc = module.main([
        str(ref),
        str(est),
        "--out",
        str(out),
        "--csv",
        str(csv),
        "--top-k",
        "3",
    ])

    assert rc == 0
    assert "# Visual Step Error Analysis" in out.read_text(encoding="utf-8")
    csv_text = csv.read_text(encoding="utf-8")
    assert csv_text.startswith("start_stamp_s,end_stamp_s")
    assert "length_ratio" in csv_text


def test_angle_difference_wraps_to_shortest_angle():
    module = load_module()

    diff = module.angle_difference_deg(math.radians(179.0), math.radians(-179.0))

    assert diff == pytest.approx(-2.0)
