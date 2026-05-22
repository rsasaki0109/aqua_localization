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
