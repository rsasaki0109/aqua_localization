"""Tests for plot_trajectories.py — alignment helper plus a tmp PNG render."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "plot_trajectories.py"


def load_module():
    pytest.importorskip("matplotlib")
    spec = importlib.util.spec_from_file_location("plot_trajectories", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def reference_circle(n=120, radius=4.0, t0=10.0, hz=10.0):
    rows = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n
        rows.append([t0 + i / hz,
                     radius * math.cos(theta),
                     radius * math.sin(theta),
                     -0.5,
                     0.0, 0.0, 0.0, 1.0])
    return rows


def test_align_estimate_returns_overlapping_arrays(tmp_path):
    module = load_module()
    rows_ref = reference_circle()
    rows_est = []
    offset = np.array([10.0, -5.0, 0.0])
    for r in rows_ref:
        rows_est.append([r[0], r[1] + offset[0], r[2] + offset[1], r[3] + offset[2],
                         0.0, 0.0, 0.0, 1.0])
    ref_path = tmp_path / "ref.tum"
    est_path = tmp_path / "est.tum"
    write_tum(ref_path, rows_ref)
    write_tum(est_path, rows_est)

    compare = module.load_compare_module()
    reference = compare.load_tum(ref_path)
    estimate = compare.load_tum(est_path)
    timestamps, aligned, ref_xyz = module.align_estimate_to_reference(
        reference, estimate, with_scale=False, no_align=False)

    assert timestamps.shape[0] == len(rows_ref)
    assert aligned.shape == ref_xyz.shape
    # After rigid alignment of a translated copy, the residual should be ~0.
    residual = float(np.linalg.norm(aligned - ref_xyz, axis=1).mean())
    assert residual == pytest.approx(0.0, abs=1e-6)


def test_render_figure_writes_png(tmp_path):
    module = load_module()
    rows_ref = reference_circle(n=60)
    rows_est = [[r[0], r[1] + 0.01, r[2] - 0.02, r[3], 0.0, 0.0, 0.0, 1.0] for r in rows_ref]
    ref_path = tmp_path / "ref.tum"
    est_path = tmp_path / "est.tum"
    write_tum(ref_path, rows_ref)
    write_tum(est_path, rows_est)

    compare = module.load_compare_module()
    reference = compare.load_tum(ref_path)
    estimate = compare.load_tum(est_path)
    timestamps, aligned, ref_xyz = module.align_estimate_to_reference(
        reference, estimate, with_scale=False, no_align=False)

    out_png = tmp_path / "out.png"
    module.render_figure(
        timestamps, aligned, ref_xyz, "test", out_png,
        dpi=80, width_in=6.0, height_in=3.0,
    )
    assert out_png.is_file()
    assert out_png.stat().st_size > 1000


def test_align_raises_when_no_overlap(tmp_path):
    module = load_module()
    rows_ref = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
    rows_est = [[10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                [11.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
    ref_path = tmp_path / "ref.tum"
    est_path = tmp_path / "est.tum"
    write_tum(ref_path, rows_ref)
    write_tum(est_path, rows_est)

    compare = module.load_compare_module()
    reference = compare.load_tum(ref_path)
    estimate = compare.load_tum(est_path)
    with pytest.raises(ValueError):
        module.align_estimate_to_reference(reference, estimate, with_scale=False,
                                           no_align=False)
