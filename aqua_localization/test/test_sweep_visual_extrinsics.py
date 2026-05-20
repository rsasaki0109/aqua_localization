"""Tests for sweep_visual_extrinsics.py."""

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "sweep_visual_extrinsics.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("sweep_visual_extrinsics", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def test_transform_camera_to_base_recovers_known_lever_arm():
    module = load_module()
    candidate = module.ExtrinsicCandidate(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    camera_rows = np.asarray(
        [
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)],
        ],
        dtype=np.float64,
    )

    base_rows = module.transform_camera_to_base(camera_rows, candidate)

    np.testing.assert_allclose(base_rows[0, 1:4], [0.0, 0.0, 0.0], atol=1.0e-9)
    np.testing.assert_allclose(base_rows[1, 1:4], [2.0, -1.0, 0.0], atol=1.0e-9)


def test_build_candidates_uses_cartesian_product():
    module = load_module()
    args = SimpleNamespace(
        x_m="0,1",
        y_m="0",
        z_m="0",
        roll_deg="0",
        pitch_deg="0",
        yaw_deg="-5,5",
    )

    candidates = module.build_candidates(args)

    assert len(candidates) == 4
    assert candidates[0].x_m == 0.0
    assert candidates[-1].x_m == 1.0
    assert candidates[-1].yaw_rad == pytest.approx(math.radians(5.0))


def test_run_sweep_selects_known_extrinsic(tmp_path):
    module = load_module()
    reference = tmp_path / "base_ref.tum"
    estimate = tmp_path / "camera_est.tum"
    base_rows = []
    camera_rows = []
    base_from_camera = module.transform_from_xyz_rpy(0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
    for i in range(20):
        yaw = 0.05 * float(i)
        world_from_base = module.transform_from_xyz_rpy(
            0.2 * float(i),
            math.sin(0.1 * float(i)),
            0.0,
            0.0,
            0.0,
            yaw,
        )
        world_from_camera = world_from_base @ base_from_camera
        base_rows.append(module.tum_row_from_transform(float(i), world_from_base))
        camera_rows.append(module.tum_row_from_transform(float(i), world_from_camera))
    write_tum(reference, base_rows)
    write_tum(estimate, camera_rows)
    args = SimpleNamespace(
        reference=reference,
        estimate=estimate,
        out_dir=tmp_path / "out",
        x_m="0.0,0.5",
        y_m="0.0",
        z_m="0.0",
        roll_deg="0.0",
        pitch_deg="0.0",
        yaw_deg="0.0",
        scale=False,
        no_align=False,
    )

    results = module.run_sweep(args)
    best = min(results, key=lambda result: result.rmse_m)

    assert best.candidate.x_m == pytest.approx(0.5)
    assert best.rmse_m == pytest.approx(0.0, abs=1.0e-9)
    assert best.transformed_tum.exists()


def test_format_markdown_reports_best(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        sequence="short_test",
        reference=Path("/tmp/ref.tum"),
        estimate=Path("/tmp/est.tum"),
        scale=False,
        no_align=False,
    )
    result = module.ExtrinsicResult(
        candidate=module.ExtrinsicCandidate(0.5, 0.0, 0.0, 0.0, 0.0, math.radians(5.0)),
        transformed_tum=tmp_path / "best.tum",
        rmse_m=0.05,
        matched_seconds=10.0,
        count=100,
        mean_m=0.04,
        median_m=0.03,
        max_m=0.2,
    )

    text = module.format_markdown([result], args)

    assert "Visual Extrinsic Sweep" in text
    assert "| 0.500 | 0.000 | 0.000 | 0.00 | 0.00 | 5.00 | best | 0.0500 |" in text
    assert "Best RMSE" in text
