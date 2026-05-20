"""Pure tests for validate_visual_scale.py."""

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "validate_visual_scale.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("validate_visual_scale", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def make_rows(scale=1.0, n=40, y_offset=0.0):
    rows = []
    for i in range(n):
        t = float(i)
        x = 0.25 * float(i)
        y = y_offset + 0.05 * float(i % 7)
        z = 0.02 * float(i % 5)
        rows.append([t, x * scale, y * scale, z * scale, 0.0, 0.0, 0.0, 1.0])
    return rows


def test_compare_scaled_validation_recovers_scaled_path(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0))

    stats = module.compare_scaled_validation(ref, est, scale_factor=0.25)

    assert stats["count"] == 40
    assert stats["rmse"] == pytest.approx(0.0, abs=1.0e-9)


def test_run_validation_uses_calibration_scale_on_validation_pair(tmp_path):
    module = load_module()
    calib_ref = tmp_path / "calib_ref.tum"
    calib_est = tmp_path / "calib_est.tum"
    val_ref = tmp_path / "val_ref.tum"
    val_est = tmp_path / "val_est.tum"
    write_tum(calib_ref, make_rows(scale=1.0, y_offset=0.0))
    write_tum(calib_est, make_rows(scale=5.0, y_offset=0.0))
    write_tum(val_ref, make_rows(scale=1.0, y_offset=1.0))
    write_tum(val_est, make_rows(scale=5.0, y_offset=1.0))

    class Args:
        calibration_reference = calib_ref
        calibration_estimate = calib_est
        validation_reference = val_ref
        validation_estimate = val_est
        calibration_current_scale = 1.0
        validation_current_scale = 1.0
        no_align = False

    result = module.run_validation(Args)

    assert result["calibration"]["recommended_translation_scale"] == pytest.approx(0.2)
    assert result["scale_factor_applied_to_validation"] == pytest.approx(0.2)
    assert result["validation"]["rmse"] == pytest.approx(0.0, abs=1.0e-9)


def test_markdown_row_format():
    module = load_module()
    stats = {
        "count": 10,
        "matched_seconds": 1.5,
        "mean": 0.1,
        "median": 0.09,
        "rmse": 0.12,
        "max": 0.2,
    }

    row = module.markdown_row("Tank", "held_out", "visual", stats, "note")

    assert row == "| Tank | held_out | visual | SE(3) | 10 | 1.50 | 0.1000 | 0.0900 | 0.1200 | 0.2000 | note |"
