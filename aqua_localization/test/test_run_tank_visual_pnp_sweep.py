"""Tests for run_tank_visual_pnp_sweep.py."""

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "run_tank_visual_pnp_sweep.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_visual_pnp_sweep", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_cases_makes_quality_gate_matrix_and_dedupes():
    module = load_module()
    args = SimpleNamespace(
        reprojection_errors_px="2,3,2",
        min_inlier_ratios="0.5,0.8",
        max_step_translation_m="0.05",
        min_pnp_inliers="12",
        ransac_iterations="100",
        ransac_confidences="0.99",
    )

    cases = module.build_cases(args)

    assert cases == [
        module.PnpCase(2.0, 0.5, 0.05, 12, 100, 0.99),
        module.PnpCase(2.0, 0.8, 0.05, 12, 100, 0.99),
        module.PnpCase(3.0, 0.5, 0.05, 12, 100, 0.99),
        module.PnpCase(3.0, 0.8, 0.05, 12, 100, 0.99),
    ]


def test_case_args_passes_pnp_parameters(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--bag", "/tmp/bag",
        "--reference", "/tmp/ref.tum",
        "--translation-scale", "0.095",
    ])
    case = module.PnpCase(2.5, 0.65, 0.05, 24, 200, 0.995)

    current = module.case_args(args, case, "seq", tmp_path)

    assert current.ransac_reprojection_error_px == 2.5
    assert current.min_inlier_ratio == 0.65
    assert current.max_step_translation_m == 0.05
    assert current.min_pnp_inliers == 24
    assert current.ransac_iterations == 200
    assert current.ransac_confidence == 0.995
    assert current.translation_scale == 0.095


def test_format_markdown_selects_best_and_reports_rejections(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        sequence="short_test",
        reference=Path("/tmp/ref.tum"),
        start_offset_s=0.0,
        duration_s=11.25,
        translation_scale=0.095,
        baseline_rmse_m=0.0194,
    )
    first = module.PnpSweepResult(
        case=module.PnpCase(3.0, 0.25, 2.0, 12, 100, 0.99),
        sequence="case1",
        out_dir=tmp_path / "case1",
        rmse_m=0.12,
        matched_seconds=11.0,
        samples=220,
        accepted_ratio=1.0,
        rejected_frames=0,
        dominant_rejection="none",
        median_pnp_inliers=130.0,
        median_inlier_ratio=0.9,
        median_temporal_matches=144.0,
    )
    second = module.PnpSweepResult(
        case=module.PnpCase(2.0, 0.8, 0.05, 12, 100, 0.99),
        sequence="case2",
        out_dir=tmp_path / "case2",
        rmse_m=0.09,
        matched_seconds=11.0,
        samples=210,
        accepted_ratio=0.85,
        rejected_frames=30,
        dominant_rejection="low pnp inlier ratio",
        median_pnp_inliers=120.0,
        median_inlier_ratio=0.86,
        median_temporal_matches=140.0,
    )

    markdown = module.format_markdown([first, second], args)

    assert "Tank Visual PnP Quality Sweep" in markdown
    assert "| 2.00 | 0.80 | 0.050 | 12 | 100 | 0.990 | best | 0.0900 |" in markdown
    assert "low pnp inlier ratio" in markdown
    assert "Best gap to baseline: `4.64x`" in markdown


def test_validate_rejects_bad_ranges():
    module = load_module()
    args = module.parse_args([
        "--bag", "/tmp/bag",
        "--reference", "/tmp/ref.tum",
        "--min-inlier-ratios", "0.5,1.2",
    ])

    with pytest.raises(ValueError, match="min-inlier-ratios"):
        module.validate_args(args)


def test_gap_helpers_handle_nan():
    module = load_module()

    assert module.gap_ratio(0.10, 0.02) == 5.0
    assert module.improvement_to_tie_percent(0.10, 0.02) == 80.0
    assert math.isnan(module.gap_ratio(math.nan, 0.02))
