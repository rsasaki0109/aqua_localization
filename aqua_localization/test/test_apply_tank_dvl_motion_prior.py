"""CLI/report tests for apply_tank_dvl_motion_prior.py."""

import importlib.util
import sys
from pathlib import Path

import numpy as np


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "apply_tank_dvl_motion_prior.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("apply_tank_dvl_motion_prior", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_args_sets_deployable_prior_defaults(tmp_path):
    module = load_module()

    args = module.parse_args([
        "--bag",
        str(tmp_path / "bag"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--visual",
        str(tmp_path / "visual.tum"),
    ])
    module.fill_default_outputs(args)

    assert args.dvl_yaw_mode == "imu_yaw"
    assert args.dvl_frame_yaw_offset_deg == -90.0
    assert args.imu_yaw_offset_deg == 115.0
    assert args.mode == "blend-outliers"
    assert args.corrected_out == args.out_dir / "tank_dvl_motion_prior_corrected.tum"


def test_parse_args_loads_profile_defaults_and_allows_override(tmp_path):
    module = load_module()
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        "\n".join([
            "format_version: 1",
            "name: profiled_prior",
            "metadata:",
            "  calibration_sequence: short_test",
            "  validation_sequence: Medium",
            "prior:",
            "  dvl_yaw_mode: imu_yaw",
            "  dvl_frame_yaw_offset_deg: -90.0",
            "  imu_yaw_offset_deg: 115.0",
            "  prior_scale: 1.25375",
            "application:",
            "  mode: replace-outliers",
            "  blend_alpha: 0.5",
            "  min_prior_step_m: 0.0001",
            "  min_visual_step_m: 0.02",
            "  min_length_ratio: 0.5",
            "  max_length_ratio: 1.5",
            "  min_direction_cosine: 0.5",
        ]),
        encoding="utf-8",
    )

    args = module.parse_args([
        "--profile",
        str(profile),
        "--bag",
        str(tmp_path / "bag"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--visual",
        str(tmp_path / "visual.tum"),
        "--prior-scale",
        "1.1",
    ])

    assert args.profile_label == "profiled_prior"
    assert args.profile_calibration_sequence == "short_test"
    assert args.profile_validation_sequence == "Medium"
    assert args.mode == "replace-outliers"
    assert args.prior_scale == 1.1
    assert args.imu_yaw_offset_deg == 115.0
    assert args.min_visual_step_m == 0.02


def test_prior_step_quality_rows_merges_dvl_coverage_and_gate_rows():
    module = load_module()
    from simulate_visual_motion_prior import PriorStep
    from tank_dvl_prior_core import DvlPriorDelta

    times = np.asarray([0.0, 1.0], dtype=np.float64)
    visual_xyz = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
    prior_xyz = np.asarray([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    deltas = [DvlPriorDelta(0.0, 1.0, np.asarray([0.0, 1.0, 0.0]), 3, True)]
    sim_rows = [
        PriorStep(
            start_stamp_s=0.0,
            end_stamp_s=1.0,
            offset_s=1.0,
            dt_s=1.0,
            visual_step_m=1.0,
            prior_step_m=1.0,
            corrected_step_m=1.0,
            length_ratio=1.0,
            direction_cosine=0.0,
            heading_error_deg=-90.0,
            used_prior=True,
            reason="direction mismatch",
        )
    ]

    rows = module.prior_step_quality_rows(times, visual_xyz, prior_xyz, deltas, sim_rows)

    assert rows[0]["dvl_covered"] is True
    assert rows[0]["dvl_samples"] == 3
    assert rows[0]["used_prior"] is True
    assert rows[0]["reason"] == "direction mismatch"
    assert rows[0]["visual_prior_residual_m"] > 0.0
    assert rows[0]["prior_confidence"] == 0.0
    assert rows[0]["prior_confidence_accepted"] is False
    assert rows[0]["prior_reject_reason"] == "direction mismatch"


def test_prior_step_quality_rows_reports_high_confidence_for_consistent_prior():
    module = load_module()
    from simulate_visual_motion_prior import PriorStep
    from tank_dvl_prior_core import DvlPriorDelta

    times = np.asarray([0.0, 1.0], dtype=np.float64)
    visual_xyz = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
    prior_xyz = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
    deltas = [DvlPriorDelta(0.0, 1.0, np.asarray([1.0, 0.0, 0.0]), 3, True)]
    sim_rows = [
        PriorStep(
            start_stamp_s=0.0,
            end_stamp_s=1.0,
            offset_s=1.0,
            dt_s=1.0,
            visual_step_m=1.0,
            prior_step_m=1.0,
            corrected_step_m=1.0,
            length_ratio=1.0,
            direction_cosine=1.0,
            heading_error_deg=0.0,
            used_prior=False,
            reason="visual",
        )
    ]

    rows = module.prior_step_quality_rows(
        times,
        visual_xyz,
        prior_xyz,
        deltas,
        sim_rows,
        min_visual_step_m=0.01,
        min_prior_step_m=0.01,
    )

    assert rows[0]["prior_confidence"] == 1.0
    assert rows[0]["prior_confidence_accepted"] is True
    assert rows[0]["prior_reject_reason"] == "accepted"


def test_validate_args_rejects_bad_scale(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--bag",
        str(tmp_path / "bag"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--visual",
        str(tmp_path / "visual.tum"),
        "--prior-scale",
        "0.0",
    ])

    try:
        module.validate_args(args)
    except ValueError as exc:
        assert "prior-scale" in str(exc)
    else:
        raise AssertionError("expected invalid prior scale to raise")
