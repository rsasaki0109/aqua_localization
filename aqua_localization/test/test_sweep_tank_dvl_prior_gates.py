"""Tests for sweep_tank_dvl_prior_gates.py."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "sweep_tank_dvl_prior_gates.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("sweep_tank_dvl_prior_gates", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_float_list_rejects_empty_values():
    module = load_module()

    assert module.parse_float_list("1.0, 1.5") == [1.0, 1.5]
    with pytest.raises(ValueError, match="at least one"):
        module.parse_float_list(" , ")


def test_parse_mode_list_accepts_confidence_modes():
    module = load_module()

    assert module.parse_mode_list("confidence-blend-outliers,confidence-replace-outliers") == [
        "confidence-blend-outliers",
        "confidence-replace-outliers",
    ]


def test_validate_sequence_split_rejects_calibration_sequence():
    module = load_module()
    args = SimpleNamespace(
        profile=Path("profile.yaml"),
        sequence="short_test",
        allow_same_sequence=False,
        allow_profile_sequence_mismatch=True,
    )
    profile = {
        "name": "tank_profile",
        "metadata": {
            "calibration_sequence": "short_test",
            "validation_sequence": "Medium",
        },
    }

    with pytest.raises(ValueError, match="matches profile calibration_sequence"):
        module.validate_sequence_split(args, profile)


def test_aligned_error_stats_removes_global_offset():
    module = load_module()
    compare = module.dvl_apply.load_compare_module()
    times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    reference = np.asarray([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
    ])
    estimate = reference + np.asarray([10.0, -3.0, 0.0])

    stats = module.aligned_error_stats(compare, times, reference, estimate)

    assert stats["rmse"] == pytest.approx(0.0, abs=1.0e-12)
    assert stats["matched_s"] == pytest.approx(2.0)


def test_evaluate_candidate_replaces_bad_direction_step():
    module = load_module()
    compare = module.dvl_apply.load_compare_module()
    times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    reference = np.asarray([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
    ])
    visual = np.asarray([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
    ])
    prior = reference
    original_stats = module.aligned_error_stats(compare, times, reference, visual)

    row, corrected, sim_rows = module.evaluate_candidate(
        compare,
        times,
        reference,
        visual,
        prior,
        mode="replace-outliers",
        blend_alpha=0.5,
        prior_scale=1.0,
        min_prior_step_m=0.0,
        min_length_ratio=0.5,
        max_length_ratio=1.5,
        min_direction_cosine=0.5,
        original_rmse_m=original_stats["rmse"],
        baseline_rmse_m=0.1,
    )

    np.testing.assert_allclose(corrected, prior)
    assert sim_rows[1].used_prior is True
    assert row.corrected_rmse_m == pytest.approx(0.0, abs=1.0e-12)
    assert row.prior_steps == 1


def test_format_markdown_flags_diagnostic_override(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        profile=tmp_path / "profile.yaml",
        csv_out=tmp_path / "sweep.csv",
        best_corrected_out=tmp_path / "best.tum",
        best_dvl_prior_out=tmp_path / "prior.tum",
        best_step_csv_out=tmp_path / "steps.csv",
        baseline_rmse_m=0.0194,
        top_k=1,
    )
    metadata = module.SweepMetadata(
        profile_label="tank_profile",
        calibration_sequence="short_test",
        profile_validation_sequence="Medium",
        sequence="short_test",
        same_sequence_allowed=True,
        profile_sequence_mismatch_allowed=True,
    )
    row = module.SweepRow(
        rank=1,
        mode="replace-outliers",
        blend_alpha=0.5,
        prior_scale=1.15,
        min_length_ratio=0.65,
        max_length_ratio=1.25,
        min_direction_cosine=0.7,
        corrected_rmse_m=0.0154,
        mean_error_m=0.012,
        median_error_m=0.01,
        max_error_m=0.04,
        improvement_percent=86.3,
        gap_to_baseline_x=0.79,
        prior_steps=169,
        steps=217,
        samples=218,
        matched_s=11.1,
    )
    best = SimpleNamespace(row=row)

    text = module.format_markdown(args, metadata, [row], best)

    assert "diagnostic tuning evidence" in text
    assert "0.0154 m" in text
    assert "0.79x" in text
