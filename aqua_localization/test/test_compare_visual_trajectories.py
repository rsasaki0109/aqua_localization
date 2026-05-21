"""Tests for compare_visual_trajectories.py."""

import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "compare_visual_trajectories.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("compare_visual_trajectories", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{value:.9f}" for value in row) + "\n")


def quat_from_yaw(yaw_rad: float):
    return [0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad)]


def tum_row(stamp_s: float, xyz, yaw_rad: float = 0.0):
    return [stamp_s, xyz[0], xyz[1], xyz[2], *quat_from_yaw(yaw_rad)]


def test_path_length_and_ratio_are_reported_after_alignment(tmp_path):
    module = load_module()
    baseline_rows = [
        tum_row(10.0, [0.0, 0.0, 0.0]),
        tum_row(11.0, [1.0, 0.0, 0.0]),
        tum_row(12.0, [2.0, 0.0, 0.0]),
        tum_row(13.0, [3.0, 0.0, 0.0]),
    ]
    target_rows = [
        tum_row(10.0, [5.0, 1.0, 0.0]),
        tum_row(11.0, [7.0, 1.0, 0.0]),
        tum_row(12.0, [9.0, 1.0, 0.0]),
        tum_row(13.0, [11.0, 1.0, 0.0]),
    ]
    baseline = tmp_path / "baseline.tum"
    target = tmp_path / "target.tum"
    write_tum(baseline, baseline_rows)
    write_tum(target, target_rows)

    summary, rows = module.compare_visual_trajectories(
        baseline,
        target,
        with_scale=True,
        no_align=False,
        drift_threshold_m=0.05,
        drift_consecutive_samples=2,
    )

    assert summary.samples == 4
    assert summary.error_rmse_m == pytest.approx(0.0, abs=1.0e-9)
    assert summary.alignment_scale == pytest.approx(0.5, abs=1.0e-9)
    assert summary.baseline_path_length_m == pytest.approx(3.0, abs=1.0e-9)
    assert summary.target_raw_path_length_m == pytest.approx(6.0, abs=1.0e-9)
    assert summary.target_aligned_path_length_m == pytest.approx(3.0, abs=1.0e-9)
    assert summary.raw_path_length_ratio == pytest.approx(2.0, abs=1.0e-9)
    assert rows[0]["offset_s"] == pytest.approx(0.0)
    assert rows[-1]["offset_s"] == pytest.approx(3.0)


def test_find_drift_start_requires_consecutive_samples():
    module = load_module()
    times = np.array([10.0, 11.0, 12.0, 13.0, 14.0])
    errors = np.array([0.01, 0.06, 0.03, 0.07, 0.08])

    stamp, offset, error = module.find_drift_start(
        times,
        errors,
        threshold_m=0.05,
        consecutive_samples=2,
    )

    assert stamp == pytest.approx(13.0)
    assert offset == pytest.approx(3.0)
    assert error == pytest.approx(0.07)


def test_yaw_drift_uses_unwrapped_relative_change(tmp_path):
    module = load_module()
    baseline_rows = [
        tum_row(0.0, [0.0, 0.0, 0.0], 0.0),
        tum_row(1.0, [1.0, 0.0, 0.0], 0.0),
        tum_row(2.0, [2.0, 0.0, 0.0], 0.0),
    ]
    target_rows = [
        tum_row(0.0, [0.0, 0.0, 0.0], math.radians(10.0)),
        tum_row(1.0, [1.0, 0.0, 0.0], math.radians(20.0)),
        tum_row(2.0, [2.0, 0.0, 0.0], math.radians(30.0)),
    ]
    baseline = tmp_path / "baseline.tum"
    target = tmp_path / "target.tum"
    write_tum(baseline, baseline_rows)
    write_tum(target, target_rows)

    summary, _ = module.compare_visual_trajectories(
        baseline,
        target,
        with_scale=False,
        no_align=False,
        drift_threshold_m=0.05,
        drift_consecutive_samples=2,
    )

    assert summary.yaw_drift_range_deg == pytest.approx(20.0, abs=1.0e-6)
    assert summary.yaw_drift_final_deg == pytest.approx(20.0, abs=1.0e-6)


def test_write_error_csv(tmp_path):
    module = load_module()
    out = tmp_path / "errors.csv"
    module.write_error_csv(out, [{"stamp_s": 1.0, "offset_s": 0.0, "baseline_x_m": 0.0,
                                  "baseline_y_m": 0.0, "baseline_z_m": 0.0,
                                  "target_raw_x_m": 1.0, "target_raw_y_m": 0.0,
                                  "target_raw_z_m": 0.0, "target_aligned_x_m": 0.0,
                                  "target_aligned_y_m": 0.0, "target_aligned_z_m": 0.0,
                                  "error_m": 0.0}])

    text = out.read_text(encoding="utf-8")
    assert text.startswith("stamp_s,offset_s,baseline_x_m")
    assert "1.0,0.0" in text
