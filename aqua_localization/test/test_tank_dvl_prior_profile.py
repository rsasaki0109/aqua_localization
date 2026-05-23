"""Tests for tank_dvl_prior_profile.py."""

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "tank_dvl_prior_profile.py"


def load_module():
    spec = importlib.util.spec_from_file_location("tank_dvl_prior_profile", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_write_and_load_profile(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--out",
        str(tmp_path / "profile.yaml"),
        "--name",
        "tank_short_to_medium",
        "--calibration-sequence",
        "short_test",
        "--validation-sequence",
        "Medium",
        "--dvl-frame-yaw-offset-deg",
        "-90",
        "--imu-yaw-offset-deg",
        "115",
        "--prior-scale",
        "1.25375",
        "--mode",
        "replace-outliers",
    ])

    profile = module.build_profile(args)
    module.write_profile(args.out, profile)
    loaded = module.load_profile(args.out)

    assert loaded["name"] == "tank_short_to_medium"
    assert loaded["prior"]["dvl_frame_yaw_offset_deg"] == -90.0
    assert loaded["prior"]["imu_yaw_offset_deg"] == 115.0
    assert loaded["prior"]["prior_scale"] == 1.25375
    assert loaded["application"]["mode"] == "replace-outliers"
    assert loaded["application"]["min_visual_step_m"] == 0.0


def test_profile_arg_defaults_reads_application_and_prior(tmp_path):
    module = load_module()
    profile_path = tmp_path / "profile.yaml"
    module.write_profile(profile_path, {
        "format_version": 1,
        "name": "profile",
        "metadata": {},
        "prior": {
            "dvl_yaw_mode": "imu_yaw",
            "dvl_frame_yaw_offset_deg": -91.0,
            "imu_yaw_offset_deg": 116.0,
            "prior_scale": 1.25,
        },
        "application": {
            "mode": "replace-outliers",
            "blend_alpha": 0.75,
            "min_prior_step_m": 0.01,
            "min_visual_step_m": 0.02,
            "min_length_ratio": 0.6,
            "max_length_ratio": 1.4,
            "min_direction_cosine": 0.7,
        },
    })

    defaults = module.profile_arg_defaults(profile_path)

    assert defaults["dvl_frame_yaw_offset_deg"] == -91.0
    assert defaults["imu_yaw_offset_deg"] == 116.0
    assert defaults["prior_scale"] == 1.25
    assert defaults["mode"] == "replace-outliers"
    assert defaults["min_visual_step_m"] == 0.02
    assert defaults["min_direction_cosine"] == 0.7


def test_profile_label_prefers_name_then_metadata(tmp_path):
    module = load_module()
    named = {"name": "named_profile", "metadata": {"calibration_sequence": "A"}}
    unnamed = {"metadata": {"calibration_sequence": "A", "validation_sequence": "B"}}

    assert module.profile_label(tmp_path / "p.yaml", named) == "named_profile"
    assert module.profile_label(tmp_path / "p.yaml", unnamed) == "A_to_B"


def test_validate_args_rejects_invalid_thresholds(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--out",
        str(tmp_path / "profile.yaml"),
        "--min-length-ratio",
        "2.0",
        "--max-length-ratio",
        "1.0",
    ])

    with pytest.raises(ValueError, match="max-length-ratio"):
        module.validate_args(args)


def test_main_writes_profile_and_summary(tmp_path, capsys):
    module = load_module()
    out = tmp_path / "profile.yaml"

    rc = module.main([
        "--out",
        str(out),
        "--name",
        "tank_short",
        "--calibration-sequence",
        "short_test",
    ])

    assert rc == 0
    assert out.exists()
    assert "wrote Tank DVL prior profile" in capsys.readouterr().out
