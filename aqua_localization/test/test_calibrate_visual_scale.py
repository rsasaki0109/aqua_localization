"""Pure tests for calibrate_visual_scale.py."""

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "calibrate_visual_scale.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("calibrate_visual_scale", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def test_estimate_scale_recovers_known_translation_scale(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    rows_ref = []
    rows_est = []
    for i in range(30):
        t = float(i)
        x = float(i) * 0.2
        y = float(i % 5) * 0.1
        z = float(i % 3) * 0.05
        rows_ref.append([t, x, y, z, 0.0, 0.0, 0.0, 1.0])
        rows_est.append([t, x * 5.0, y * 5.0, z * 5.0, 0.0, 0.0, 0.0, 1.0])
    write_tum(ref, rows_ref)
    write_tum(est, rows_est)

    result = module.estimate_scale(ref, est)

    assert result["matched_samples"] == 30
    assert result["recommended_translation_scale"] == pytest.approx(0.2, abs=1.0e-9)
    assert result["sim3_rmse"] < result["se3_rmse"]


def test_format_report_can_emit_ros_parameter_override():
    module = load_module()
    result = {
        "matched_samples": 10,
        "matched_seconds": 3.5,
        "current_scale": 1.0,
        "sim3_alignment_scale": 0.25,
        "recommended_translation_scale": 0.25,
        "se3_rmse": 1.0,
        "sim3_rmse": 0.1,
    }

    text = module.format_report(result, ros_args=True)

    assert "recommended tracking.translation_scale: 0.250000000" in text
    assert "-p tracking.translation_scale:=0.250000000" in text
