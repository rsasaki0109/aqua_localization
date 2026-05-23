"""Tests for promote_tank_dvl_sweep_profile.py."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "promote_tank_dvl_sweep_profile.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("promote_tank_dvl_sweep_profile", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def base_profile():
    return {
        "format_version": 1,
        "name": "tank_short",
        "metadata": {
            "dataset": "Tank Dataset",
            "calibration_sequence": "short_test",
            "validation_sequence": "Medium",
        },
        "prior": {
            "dvl_yaw_mode": "imu_yaw",
            "dvl_frame_yaw_offset_deg": -90.0,
            "imu_yaw_offset_deg": 115.0,
            "prior_scale": 1.25375,
        },
        "application": {
            "mode": "replace-outliers",
            "blend_alpha": 0.5,
            "min_prior_step_m": 0.0001,
            "min_length_ratio": 0.5,
            "max_length_ratio": 1.5,
            "min_direction_cosine": 0.5,
        },
    }


def sweep_row(rank="1"):
    return {
        "rank": rank,
        "mode": "replace-outliers",
        "blend_alpha": "0.5",
        "prior_scale": "1.15",
        "min_length_ratio": "0.65",
        "max_length_ratio": "1.25",
        "min_direction_cosine": "0.7",
        "corrected_rmse_m": "0.0154348",
        "gap_to_baseline_x": "0.7956",
    }


def test_promote_profile_updates_gate_values():
    module = load_module()
    args = SimpleNamespace(
        base_profile=Path("base.yaml"),
        sweep_csv=Path("sweep.csv"),
        rank=1,
        name="tank_best",
        validation_sequence="",
        note="",
    )

    promoted = module.promote_profile(base_profile(), sweep_row(), args)

    assert promoted["name"] == "tank_best"
    assert promoted["prior"]["prior_scale"] == 1.15
    assert promoted["prior"]["imu_yaw_offset_deg"] == 115.0
    assert promoted["application"]["min_length_ratio"] == 0.65
    assert promoted["application"]["max_length_ratio"] == 1.25
    assert promoted["application"]["min_direction_cosine"] == 0.7
    assert promoted["metadata"]["source_sweep_rank"] == 1
    assert promoted["metadata"]["source_sweep_corrected_rmse_m"] == pytest.approx(0.0154348)


def test_select_sweep_row_uses_rank():
    module = load_module()
    rows = [sweep_row("2"), sweep_row("1")]

    selected = module.select_sweep_row(rows, 1)

    assert selected["rank"] == "1"


def test_main_writes_promoted_profile(tmp_path, capsys):
    module = load_module()
    base = tmp_path / "base.yaml"
    sweep = tmp_path / "sweep.csv"
    out = tmp_path / "promoted.yaml"
    base.write_text(yaml.safe_dump(base_profile(), sort_keys=False), encoding="utf-8")
    sweep.write_text(
        ",".join(sweep_row().keys()) + "\n" + ",".join(sweep_row().values()) + "\n",
        encoding="utf-8",
    )

    rc = module.main([
        "--base-profile",
        str(base),
        "--sweep-csv",
        str(sweep),
        "--out",
        str(out),
        "--name",
        "tank_promoted",
    ])

    assert rc == 0
    loaded = yaml.safe_load(out.read_text(encoding="utf-8"))
    assert loaded["name"] == "tank_promoted"
    assert loaded["prior"]["prior_scale"] == 1.15
    assert "source_sweep_rmse: 0.0154 m" in capsys.readouterr().out


def test_main_reports_missing_rank_without_traceback(tmp_path, capsys):
    module = load_module()
    base = tmp_path / "base.yaml"
    sweep = tmp_path / "sweep.csv"
    base.write_text(yaml.safe_dump(base_profile(), sort_keys=False), encoding="utf-8")
    sweep.write_text(
        ",".join(sweep_row().keys()) + "\n" + ",".join(sweep_row().values()) + "\n",
        encoding="utf-8",
    )

    rc = module.main([
        "--base-profile",
        str(base),
        "--sweep-csv",
        str(sweep),
        "--out",
        str(tmp_path / "out.yaml"),
        "--rank",
        "3",
    ])

    assert rc == 2
    captured = capsys.readouterr()
    assert "rank 3 not found" in captured.err
    assert "Traceback" not in captured.err
