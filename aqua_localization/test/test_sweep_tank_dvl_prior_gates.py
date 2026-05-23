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


def test_summarize_quality_rows_reports_confidence_and_rejects():
    module = load_module()
    rows = [
        {
            "dvl_covered": True,
            "prior_confidence_accepted": True,
            "prior_match_confidence": 0.8,
            "prior_confidence": 0.7,
            "effective_blend_alpha": 0.35,
            "prior_reject_reason": "accepted",
            "used_prior": True,
        },
        {
            "dvl_covered": True,
            "prior_confidence_accepted": False,
            "prior_match_confidence": 0.0,
            "prior_confidence": 0.0,
            "effective_blend_alpha": 0.0,
            "prior_reject_reason": "direction mismatch",
            "used_prior": False,
        },
        {
            "dvl_covered": False,
            "prior_confidence_accepted": False,
            "prior_match_confidence": 0.0,
            "prior_confidence": 0.0,
            "effective_blend_alpha": 0.0,
            "prior_reject_reason": "direction mismatch",
            "used_prior": False,
        },
    ]

    summary = module.summarize_quality_rows(rows)

    assert summary["dvl_covered_steps"] == 2
    assert summary["prior_match_accepted_steps"] == 1
    assert summary["mean_prior_match_confidence"] == pytest.approx(0.8 / 3.0)
    assert summary["mean_applied_prior_confidence"] == pytest.approx(0.7)
    assert summary["mean_effective_blend_alpha"] == pytest.approx(0.35 / 3.0)
    assert summary["dominant_prior_reject_reason"] == "direction mismatch"


def test_format_markdown_flags_diagnostic_override(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        profile=tmp_path / "profile.yaml",
        csv_out=tmp_path / "sweep.csv",
        best_corrected_out=tmp_path / "best.tum",
        best_dvl_prior_out=tmp_path / "prior.tum",
        best_step_csv_out=tmp_path / "steps.csv",
        best_profile_out=tmp_path / "best_profile.yaml",
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
        dvl_covered_steps=217,
        prior_match_accepted_steps=160,
        mean_prior_match_confidence=0.74,
        mean_applied_prior_confidence=0.83,
        mean_effective_blend_alpha=0.62,
        dominant_prior_reject_reason="direction mismatch",
        samples=218,
        matched_s=11.1,
    )
    best = SimpleNamespace(row=row)

    text = module.format_markdown(args, metadata, [row], best)

    assert "diagnostic tuning evidence" in text
    assert "0.0154 m" in text
    assert "0.79x" in text
    assert "160/217" in text
    assert "direction mismatch" in text


def test_write_best_profile_promotes_ranked_row(tmp_path):
    module = load_module()
    base_profile_path = tmp_path / "base.yaml"
    out = tmp_path / "best.yaml"
    base_profile = {
        "format_version": 1,
        "name": "base",
        "metadata": {
            "calibration_sequence": "short_test",
            "validation_sequence": "Medium",
        },
        "prior": {
            "dvl_yaw_mode": "imu_yaw",
            "dvl_frame_yaw_offset_deg": -90.0,
            "imu_yaw_offset_deg": 115.0,
            "prior_scale": 1.0,
        },
        "application": {
            "mode": "replace-outliers",
            "blend_alpha": 0.5,
            "min_prior_step_m": 1.0e-4,
            "min_length_ratio": 0.5,
            "max_length_ratio": 1.5,
            "min_direction_cosine": 0.5,
        },
    }
    args = SimpleNamespace(
        profile=base_profile_path,
        profile_data=base_profile,
        csv_out=tmp_path / "sweep.csv",
        best_profile_out=out,
        best_profile_name="tank_best_confidence",
        best_profile_validation_sequence="Medium",
        best_profile_note="promoted in sweep test",
    )
    row = module.SweepRow(
        rank=1,
        mode="confidence-blend-outliers",
        blend_alpha=0.4,
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
        prior_steps=120,
        steps=217,
        dvl_covered_steps=217,
        prior_match_accepted_steps=160,
        mean_prior_match_confidence=0.74,
        mean_applied_prior_confidence=0.83,
        mean_effective_blend_alpha=0.33,
        dominant_prior_reject_reason="direction mismatch",
        samples=218,
        matched_s=11.1,
    )

    profile = module.write_best_profile(args, SimpleNamespace(row=row))

    assert out.exists()
    assert profile["name"] == "tank_best_confidence"
    assert profile["prior"]["prior_scale"] == 1.15
    assert profile["application"]["mode"] == "confidence-blend-outliers"
    assert profile["application"]["blend_alpha"] == 0.4
    assert profile["metadata"]["source_sweep_rank"] == 1
    assert profile["metadata"]["validation_sequence"] == "Medium"
