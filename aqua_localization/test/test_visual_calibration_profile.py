"""Tests for visual_calibration_profile.py."""

import importlib.util
import sys
from pathlib import Path

import pytest
import yaml


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "visual_calibration_profile.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("visual_calibration_profile", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{v:.9f}" for v in row) + "\n")


def make_rows(scale=1.0, n=40):
    rows = []
    for i in range(n):
        t = float(i)
        rows.append([t, 0.25 * i * scale, 0.05 * (i % 5) * scale, 0.0, 0.0, 0.0, 0.0, 1.0])
    return rows


def test_build_profile_uses_explicit_values(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--out",
        str(tmp_path / "profile.yaml"),
        "--name",
        "short_test_diag",
        "--translation-scale",
        "0.169623465",
        "--base-from-camera-x-m",
        "-0.25",
        "--base-from-camera-y-m",
        "-0.45",
        "--orb-n-features",
        "700",
    ])

    profile = module.build_profile(args)

    assert profile["name"] == "short_test_diag"
    assert profile["frontend"]["translation_scale"] == pytest.approx(0.169623465)
    assert profile["base_from_camera"]["x_m"] == pytest.approx(-0.25)
    assert profile["base_from_camera"]["y_m"] == pytest.approx(-0.45)
    assert profile["frontend"]["orb_n_features"] == 700


def test_main_writes_profile_from_calibration_pair(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    out = tmp_path / "profile.yaml"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=5.0))

    rc = module.main([
        "--out",
        str(out),
        "--calibration-reference",
        str(ref),
        "--calibration-estimate",
        str(est),
        "--calibration-sequence",
        "calib",
        "--validation-sequence",
        "held_out",
    ])

    assert rc == 0
    profile = yaml.safe_load(out.read_text(encoding="utf-8"))
    assert profile["frontend"]["translation_scale"] == pytest.approx(0.2)
    assert profile["scale_calibration"]["matched_samples"] == 40
    assert profile["metadata"]["calibration_sequence"] == "calib"
    assert profile["metadata"]["validation_sequence"] == "held_out"


def test_profile_arg_defaults_maps_yaml_to_runner_args(tmp_path):
    module = load_module()
    path = tmp_path / "profile.yaml"
    path.write_text(
        """
name: tank_profile
frontend:
  translation_scale: 0.2
  max_stereo_descriptor_distance: 64
  max_temporal_descriptor_distance: 72
  orb_n_features: 700
  orb_fast_threshold: 16
  opencv_threads: 2
base_from_camera:
  x_m: -0.25
  y_m: -0.45
  z_m: 0.0
  roll_rad: 0.0
  pitch_rad: 0.0
  yaw_rad: 0.0
fusion:
  visual_position_variance_floor: 0.01
""",
        encoding="utf-8",
    )

    defaults = module.profile_arg_defaults(path)

    assert defaults["translation_scale"] == 0.2
    assert defaults["max_temporal_descriptor_distance"] == 72
    assert defaults["base_from_camera_x_m"] == -0.25
    assert defaults["visual_position_variance_floor"] == 0.01
    assert module.profile_label(path, module.load_profile(path)) == "tank_profile"


def test_requires_scale_source(tmp_path):
    module = load_module()
    args = module.parse_args(["--out", str(tmp_path / "profile.yaml")])

    with pytest.raises(ValueError, match="translation-scale"):
        module.build_profile(args)
