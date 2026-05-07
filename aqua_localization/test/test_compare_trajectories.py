"""Unit tests for compare_trajectories.py — pure numpy, no ROS context."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "compare_trajectories.py"


def load_module():
    spec = importlib.util.spec_from_file_location("compare_trajectories", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def reference_circle(n=200, radius=10.0, t0=100.0, hz=10.0):
    rows = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        rows.append(
            [
                t0 + i / hz,
                radius * math.cos(theta),
                radius * math.sin(theta),
                0.0,
                0.0, 0.0, 0.0, 1.0,
            ]
        )
    return rows


def test_load_tum_sorts_by_timestamp(tmp_path):
    module = load_module()
    rows = [
        [3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [2.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
    path = tmp_path / "t.tum"
    write_tum(path, rows)
    arr = module.load_tum(path)
    assert arr.shape == (3, 8)
    np.testing.assert_array_equal(arr[:, 0], [1.0, 2.0, 3.0])


def test_interpolate_positions_inside_range(tmp_path):
    module = load_module()
    rows = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [1.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0],
        [2.0, 2.0, 4.0, 6.0, 0.0, 0.0, 0.0, 1.0],
    ]
    path = tmp_path / "t.tum"
    write_tum(path, rows)
    traj = module.load_tum(path)

    out = module.interpolate_positions(traj, np.array([0.5, 1.5]))
    np.testing.assert_allclose(out, np.array([[0.5, 1.0, 1.5], [1.5, 3.0, 4.5]]))


def test_interpolate_positions_marks_out_of_range_as_nan(tmp_path):
    module = load_module()
    rows = [
        [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [11.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
    path = tmp_path / "t.tum"
    write_tum(path, rows)
    traj = module.load_tum(path)
    out = module.interpolate_positions(traj, np.array([5.0, 10.5, 12.0]))
    assert np.isnan(out[0]).all()
    assert not np.isnan(out[1]).any()
    assert np.isnan(out[2]).all()


def test_umeyama_recovers_known_rigid_transform():
    module = load_module()
    src = np.array(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 1.0]]
    )
    angle = math.radians(35.0)
    R_true = np.array(
        [
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle),  math.cos(angle), 0.0],
            [0.0,              0.0,             1.0],
        ]
    )
    t_true = np.array([2.5, -1.0, 0.5])
    dst = (R_true @ src.T).T + t_true

    R, t, scale = module.umeyama_alignment(src, dst, with_scale=False)
    np.testing.assert_allclose(R, R_true, atol=1e-10)
    np.testing.assert_allclose(t, t_true, atol=1e-10)
    assert scale == pytest.approx(1.0, abs=1e-10)


def test_umeyama_recovers_known_similarity():
    module = load_module()
    rng = np.random.default_rng(7)
    src = rng.normal(size=(50, 3))
    angle = math.radians(20.0)
    R_true = np.array(
        [
            [1.0, 0.0,             0.0],
            [0.0, math.cos(angle), -math.sin(angle)],
            [0.0, math.sin(angle),  math.cos(angle)],
        ]
    )
    s_true = 1.7
    t_true = np.array([0.5, 0.25, -1.0])
    dst = s_true * (R_true @ src.T).T + t_true

    R, t, scale = module.umeyama_alignment(src, dst, with_scale=True)
    np.testing.assert_allclose(R, R_true, atol=1e-9)
    np.testing.assert_allclose(t, t_true, atol=1e-9)
    assert scale == pytest.approx(s_true, abs=1e-9)


def test_compare_perfect_overlap_yields_zero_ape(tmp_path):
    module = load_module()
    rows = reference_circle(n=120, radius=5.0, t0=10.0, hz=20.0)
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, rows)
    write_tum(est, rows)

    stats, errors = module.compare(ref, est, with_scale=False, no_align=False)
    assert stats["count"] == len(rows)
    assert stats["rmse"] == pytest.approx(0.0, abs=1e-9)
    assert errors.max() == pytest.approx(0.0, abs=1e-9)


def test_compare_translated_estimate_is_aligned_to_zero(tmp_path):
    module = load_module()
    rows_ref = reference_circle(n=80, radius=4.0, t0=0.0, hz=10.0)
    offset = np.array([100.0, -50.0, 7.5])
    rows_est = []
    for r in rows_ref:
        rows_est.append(
            [r[0], r[1] + offset[0], r[2] + offset[1], r[3] + offset[2], 0.0, 0.0, 0.0, 1.0]
        )
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, rows_ref)
    write_tum(est, rows_est)

    stats, _ = module.compare(ref, est, with_scale=False, no_align=False)
    assert stats["rmse"] == pytest.approx(0.0, abs=1e-9)
    np.testing.assert_allclose(stats["alignment"]["translation"], -offset, atol=1e-9)


def test_compare_no_align_keeps_offset_in_error(tmp_path):
    module = load_module()
    rows_ref = reference_circle(n=40, radius=2.0, t0=0.0, hz=5.0)
    offset = np.array([3.0, -4.0, 0.0])
    rows_est = []
    for r in rows_ref:
        rows_est.append(
            [r[0], r[1] + offset[0], r[2] + offset[1], r[3] + offset[2], 0.0, 0.0, 0.0, 1.0]
        )
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, rows_ref)
    write_tum(est, rows_est)

    stats, errors = module.compare(ref, est, with_scale=False, no_align=True)
    expected = float(np.linalg.norm(offset))
    assert stats["mean"] == pytest.approx(expected, abs=1e-9)
    assert stats["alignment"]["applied"] is False


def test_compare_raises_when_no_overlap(tmp_path):
    module = load_module()
    rows_ref = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
    rows_est = [[10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                [11.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, rows_ref)
    write_tum(est, rows_est)

    with pytest.raises(ValueError):
        module.compare(ref, est, with_scale=False, no_align=False)
