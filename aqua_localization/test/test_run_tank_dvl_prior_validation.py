"""Tests for run_tank_dvl_prior_validation.py."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "run_tank_dvl_prior_validation.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_dvl_prior_validation", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def profile(calibration="short_test", validation="Medium"):
    return {
        "name": "tank_profile",
        "metadata": {
            "calibration_sequence": calibration,
            "validation_sequence": validation,
        },
    }


def args(sequence="Medium", allow_same=False, allow_mismatch=False):
    return SimpleNamespace(
        profile=Path("profile.yaml"),
        sequence=sequence,
        allow_same_sequence=allow_same,
        allow_profile_sequence_mismatch=allow_mismatch,
    )


def test_validate_sequence_split_accepts_declared_heldout_sequence():
    module = load_module()

    metadata = module.validate_sequence_split(args("Medium"), profile())

    assert metadata.profile_label == "tank_profile"
    assert metadata.calibration_sequence == "short_test"
    assert metadata.profile_validation_sequence == "Medium"
    assert metadata.sequence == "Medium"


def test_validate_sequence_split_rejects_calibration_sequence_without_override():
    module = load_module()

    with pytest.raises(ValueError, match="matches profile calibration_sequence"):
        module.validate_sequence_split(args("short_test"), profile())


def test_validate_sequence_split_allows_explicit_same_sequence_diagnostic():
    module = load_module()

    metadata = module.validate_sequence_split(
        args("short_test", allow_same=True, allow_mismatch=True),
        profile(),
    )

    assert metadata.same_sequence_allowed is True
    assert metadata.profile_sequence_mismatch_allowed is True


def test_validate_sequence_split_rejects_profile_validation_mismatch():
    module = load_module()

    with pytest.raises(ValueError, match="does not match profile validation_sequence"):
        module.validate_sequence_split(args("Other"), profile())


def test_validation_status_applies_rmse_and_improvement_gates():
    module = load_module()
    gate_args = SimpleNamespace(max_corrected_rmse_m=0.05, min_improvement_percent=20.0)
    result = SimpleNamespace(corrected_rmse_m=0.06, rmse_improvement_percent=10.0)

    status, failures = module.validation_status(gate_args, result)

    assert status == "FAIL"
    assert len(failures) == 2


def test_format_markdown_marks_benchmark_candidate(tmp_path):
    module = load_module()
    validation_args = SimpleNamespace(
        profile=tmp_path / "profile.yaml",
        bag=tmp_path / "bag",
        reference=tmp_path / "ref.tum",
        visual=tmp_path / "visual.tum",
        max_corrected_rmse_m=0.05,
        min_improvement_percent=20.0,
    )
    metadata = module.ValidationMetadata(
        profile_label="tank_profile",
        calibration_sequence="short_test",
        profile_validation_sequence="Medium",
        sequence="Medium",
        same_sequence_allowed=False,
        profile_sequence_mismatch_allowed=False,
    )
    result = SimpleNamespace(
        original_rmse_m=0.1,
        corrected_rmse_m=0.03,
        rmse_improvement_percent=70.0,
        covered_steps=10,
        steps=12,
        prior_steps=8,
        corrected_tum=tmp_path / "corrected.tum",
        aligned_visual_tum=tmp_path / "aligned.tum",
        dvl_prior_tum=tmp_path / "prior.tum",
    )
    app_args = SimpleNamespace(csv_out=tmp_path / "steps.csv")

    text = module.format_markdown(validation_args, metadata, result, [], app_args)

    assert "Status: `PASS`" in text
    assert "benchmark candidate" in text


def test_fill_default_outputs(tmp_path):
    module = load_module()
    validation_args = SimpleNamespace(
        out_dir=tmp_path / "out",
        summary_out=None,
        csv_out=None,
        corrected_out=None,
        aligned_visual_out=None,
        dvl_prior_out=None,
    )

    module.fill_default_outputs(validation_args)

    assert validation_args.summary_out == validation_args.out_dir / "tank_dvl_prior_validation.md"
    assert validation_args.csv_out == validation_args.out_dir / "tank_dvl_prior_validation_steps.csv"


def test_main_reports_sequence_errors_without_traceback(tmp_path, capsys):
    module = load_module()
    profile_path = tmp_path / "profile.yaml"
    profile_path.write_text(
        "\n".join([
            "format_version: 1",
            "name: profile",
            "metadata:",
            "  calibration_sequence: short_test",
            "  validation_sequence: Medium",
            "prior:",
            "  dvl_yaw_mode: imu_yaw",
            "application:",
            "  mode: replace-outliers",
        ]),
        encoding="utf-8",
    )

    rc = module.main([
        "--profile",
        str(profile_path),
        "--sequence",
        "short_test",
        "--bag",
        str(tmp_path / "bag"),
        "--reference",
        str(tmp_path / "ref.tum"),
        "--visual",
        str(tmp_path / "visual.tum"),
    ])

    assert rc == 2
    captured = capsys.readouterr()
    assert "matches profile calibration_sequence" in captured.err
    assert "Traceback" not in captured.err
