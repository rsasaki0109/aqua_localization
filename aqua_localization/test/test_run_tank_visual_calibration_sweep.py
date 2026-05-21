"""Tests for run_tank_visual_calibration_sweep.py."""

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "run_tank_visual_calibration_sweep.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_visual_calibration_sweep", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_cases_uses_cartesian_product_and_dedupes():
    module = load_module()
    args = SimpleNamespace(
        translation_scales="0.1,0.2,0.1",
        camera_bf_scales="1.0",
        camera_f_scales="0.98,1.0",
        base_from_camera_x_m="-0.25",
        base_from_camera_y_m="-0.45,-0.40",
    )

    cases = module.build_cases(args)

    assert len(cases) == 8
    assert cases[0] == module.CalibrationCase(0.1, 1.0, 0.98, -0.25, -0.45)
    assert cases[-1] == module.CalibrationCase(0.2, 1.0, 1.0, -0.25, -0.40)


def test_case_args_scales_camera_geometry(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--bag", "/tmp/bag",
        "--reference", "/tmp/ref.tum",
        "--camera-fx", "600",
        "--camera-fy", "620",
        "--camera-bf", "80",
        "--start-offset-s", "0.5",
        "--duration-s", "4.0",
    ])
    case = module.CalibrationCase(0.1, 1.05, 0.95, -0.2, -0.4)

    current = module.case_args(args, case, "seq", tmp_path)

    assert current.translation_scale == 0.1
    assert current.camera_fx == pytest.approx(570.0)
    assert current.camera_fy == pytest.approx(589.0)
    assert current.camera_bf == pytest.approx(84.0)
    assert current.base_from_camera_x_m == -0.2
    assert current.base_from_camera_y_m == -0.4
    assert current.start_offset_s == 0.5
    assert current.duration_s == 4.0


def test_format_markdown_selects_best_and_baseline_gap(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        sequence="short_test",
        reference=Path("/tmp/ref.tum"),
        start_offset_s=0.0,
        duration_s=11.25,
        camera_fx=655.0,
        camera_fy=655.0,
        camera_bf=78.89,
        base_from_camera_z_m=0.0,
        baseline_rmse_m=0.02,
    )
    first = module.CalibrationResult(
        case=module.CalibrationCase(0.10, 1.0, 1.0, -0.25, -0.45),
        sequence="case1",
        out_dir=tmp_path / "case1",
        estimate_tum=tmp_path / "case1.tum",
        status_csv=tmp_path / "case1.csv",
        rmse_m=0.11,
        matched_seconds=10.0,
        samples=200,
        sim3_scale=0.95,
        accepted_ratio=1.0,
        median_pnp_inliers=120.0,
        median_temporal_matches=140.0,
        processed_frames=201,
        accepted_frames=200,
        rejected_frames=0,
    )
    second = module.CalibrationResult(
        case=module.CalibrationCase(0.095, 1.0, 1.0, -0.25, -0.45),
        sequence="case2",
        out_dir=tmp_path / "case2",
        estimate_tum=tmp_path / "case2.tum",
        status_csv=tmp_path / "case2.csv",
        rmse_m=0.09,
        matched_seconds=10.0,
        samples=200,
        sim3_scale=1.0,
        accepted_ratio=1.0,
        median_pnp_inliers=122.0,
        median_temporal_matches=142.0,
        processed_frames=201,
        accepted_frames=200,
        rejected_frames=0,
    )

    markdown = module.format_markdown([first, second], args)

    assert "Tank Visual Calibration Sweep" in markdown
    assert "| 0.095000 | 1.0000 | 1.0000 | -0.250 | -0.450 | best | 0.0900 | 4.50 | 77.8% |" in markdown
    assert "Best gap to baseline: `4.50x`" in markdown


def test_gap_helpers_handle_nan():
    module = load_module()

    assert module.gap_ratio(0.10, 0.02) == 5.0
    assert module.improvement_to_tie_percent(0.10, 0.02) == 80.0
    assert math.isnan(module.gap_ratio(math.nan, 0.02))


def test_validate_rejects_invalid_scale():
    module = load_module()
    args = module.parse_args([
        "--bag", "/tmp/bag",
        "--reference", "/tmp/ref.tum",
        "--translation-scales", "0.1,0.0",
    ])

    with pytest.raises(ValueError, match="translation-scales"):
        module.validate_args(args)
