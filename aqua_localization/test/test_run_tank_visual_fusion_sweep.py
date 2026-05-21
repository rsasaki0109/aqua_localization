"""Tests for run_tank_visual_fusion_sweep.py."""

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "run_tank_visual_fusion_sweep.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_visual_fusion_sweep", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_pairs_and_matrix_cases():
    module = load_module()

    assert module.parse_pairs("0.01:0.25,0.04/1.0") == [
        module.SweepCase(0.01, 0.25),
        module.SweepCase(0.04, 1.0),
    ]

    args = SimpleNamespace(
        pairs="",
        variance_floors="0.01,0.04",
        max_age_s_values="0.25,1.0",
        matrix=True,
    )
    assert module.build_cases(args) == [
        module.SweepCase(0.01, 0.25),
        module.SweepCase(0.01, 1.0),
        module.SweepCase(0.04, 0.25),
        module.SweepCase(0.04, 1.0),
    ]


def test_benchmark_command_wires_fusion_knobs(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        bag=Path("/tmp/tank_bag"),
        reference=Path("/tmp/ref.tum"),
        imu_params=Path("/tmp/tank_dataset.yaml"),
        dataset="Tank Dataset",
        sequence="short_test",
        system="aqua_localization+visual",
        visual_calibration_profile=None,
        translation_scale=0.169623465,
        base_from_camera_x_m=-0.25,
        base_from_camera_y_m=-0.45,
        base_from_camera_z_m=0.0,
        base_from_camera_roll_rad=0.0,
        base_from_camera_pitch_rad=0.0,
        base_from_camera_yaw_rad=0.0,
        max_stereo_descriptor_distance=64.0,
        max_temporal_descriptor_distance=64.0,
        orb_n_features=700,
        orb_fast_threshold=16,
        opencv_threads=2,
        play_rate=1.0,
        expected_visual_frames=300,
        min_visual_coverage=0.98,
        startup_delay=0.1,
        post_play_delay=0.2,
        stop_timeout=1.0,
        use_sim_time=True,
    )
    case = module.SweepCase(0.01, 0.25)

    command = module.benchmark_command(
        args,
        case,
        "short_test_var_0p01__age_0p25",
        tmp_path,
    )

    assert "--visual-position-variance-floor" in command
    assert "0.01" in command
    assert "--visual-max-age-s" in command
    assert "0.25" in command
    assert "--visual-odom-topic" in command
    assert "/aqua_visual_frontend/fusion_sweep/var_0p01__age_0p25/odometry" in command
    assert "--fused-odom-topic" in command
    assert "/aqua_imu_loc/fusion_sweep/var_0p01__age_0p25/odometry" in command
    assert "--expected-visual-frames" in command
    assert "300" in command


def test_format_markdown_reports_baseline_and_standalone_delta(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        sequence="short_test",
        reference=Path("/tmp/ref.tum"),
        translation_scale=0.169623465,
        orb_n_features=700,
        orb_fast_threshold=16,
        opencv_threads=2,
        baseline_rmse_m=0.0194,
        standalone_visual_rmse_m=0.0947,
    )
    first = module.SweepResult(
        case=module.SweepCase(0.04, 1.0),
        sequence="case_a",
        out_dir=tmp_path / "a",
        fused_tum=tmp_path / "a.tum",
        status_csv=tmp_path / "a.csv",
        returncode=0,
        rmse_m=0.2175,
        matched_seconds=14.95,
        visual_frames=300,
        visual_coverage_ratio=1.0,
        command=["ros2", "run", "example"],
    )
    second = module.SweepResult(
        case=module.SweepCase(0.01, 0.25),
        sequence="case_b",
        out_dir=tmp_path / "b",
        fused_tum=tmp_path / "b.tum",
        status_csv=tmp_path / "b.csv",
        returncode=0,
        rmse_m=0.1000,
        matched_seconds=14.95,
        visual_frames=300,
        visual_coverage_ratio=1.0,
        command=["ros2", "run", "example"],
    )

    text = module.format_markdown([first, second], args)

    assert "AQUA-SLAM baseline RMSE: `0.0194` m" in text
    assert "Standalone visual RMSE: `0.0947` m" in text
    assert "| 0.01 | 0.25 | best | 0.1000 | 5.15x | 0.0053 |" in text
    assert "Best fused row is still `0.0053` m worse than standalone visual." in text


def test_evaluate_result_handles_empty_fused_tum(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        reference=Path("/tmp/ref.tum"),
        expected_visual_frames=0,
        min_visual_coverage=0.98,
    )
    case = module.SweepCase(0.01, 0.25)
    sequence = "empty_case"
    paths = module.run_tank_visual_fusion_benchmark.default_paths(tmp_path, sequence)
    paths.fused_tum.write_text("", encoding="utf-8")

    result = module.evaluate_result(
        args,
        case,
        sequence,
        tmp_path,
        command=["ros2", "run", "example"],
        returncode=1,
    )

    assert math.isnan(result.rmse_m)
    assert math.isnan(result.matched_seconds)
    assert result.returncode == 1


def test_main_dry_run_writes_planned_summary(tmp_path):
    module = load_module()
    out_dir = tmp_path / "sweep"
    summary = tmp_path / "summary.md"

    rc = module.main([
        "--bag",
        "/tmp/tank_bag",
        "--reference",
        "/tmp/ref.tum",
        "--out-dir",
        str(out_dir),
        "--summary-out",
        str(summary),
        "--pairs",
        "0.01:0.25,0.04:1.0",
        "--baseline-rmse-m",
        "0.0194",
        "--standalone-visual-rmse-m",
        "0.0947",
        "--dry-run",
    ])

    text = summary.read_text(encoding="utf-8")
    assert rc == 0
    assert "Tank Visual Fusion Sweep" in text
    assert "var_0p01__age_0p25" in text
    assert "| 0.04 | 1 | ok | n/a | n/a | n/a | n/a | n/a |" in text


def test_invalid_helpers_reject_non_positive_values():
    module = load_module()

    try:
        module.parse_pairs("0.0:1.0")
    except ValueError as exc:
        assert "positive" in str(exc)
    else:
        raise AssertionError("expected ValueError")

    assert module.gap_ratio(0.1, 0.02) == 5.0
    assert module.standalone_delta_m(0.1, 0.0947) == 0.1 - 0.0947
    assert math.isnan(module.gap_ratio(math.nan, 0.02))
