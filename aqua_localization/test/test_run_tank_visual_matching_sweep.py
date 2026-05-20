"""Tests for run_tank_visual_matching_sweep.py."""

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "run_tank_visual_matching_sweep.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_visual_matching_sweep", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_parse_pairs_accepts_disabled_aliases():
    module = load_module()

    cases = module.parse_pairs("64:80,96,disabled:off")

    assert cases == [
        module.SweepCase(64.0, 80.0),
        module.SweepCase(96.0, 96.0),
        module.SweepCase(0.0, 0.0),
    ]


def test_build_cases_can_make_matrix():
    module = load_module()
    args = SimpleNamespace(
        pairs="",
        stereo_distances="64,80",
        temporal_distances="96,0",
        matrix=True,
    )

    cases = module.build_cases(args)

    assert cases == [
        module.SweepCase(64.0, 96.0),
        module.SweepCase(64.0, 0.0),
        module.SweepCase(80.0, 96.0),
        module.SweepCase(80.0, 0.0),
    ]


def test_benchmark_command_contains_thresholds(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        bag=Path("/tmp/tank_bag"),
        reference=Path("/tmp/ref.tum"),
        dataset="Tank Dataset",
        system="aqua_visual_frontend",
        translation_scale=0.25,
        play_rate=0.5,
        startup_delay=0.1,
        stop_timeout=1.0,
        use_sim_time=True,
    )

    command = module.benchmark_command(
        args,
        module.SweepCase(64.0, 80.0),
        "short_test_stereo_64__temporal_80",
        tmp_path,
    )

    assert "--max-stereo-descriptor-distance" in command
    assert "64.0" in command
    assert "--max-temporal-descriptor-distance" in command
    assert "80.0" in command
    assert "--no-sim-time" not in command


def test_format_markdown_selects_best_result(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        sequence="short_test",
        reference=Path("/tmp/ref.tum"),
        translation_scale=0.25,
    )
    first = module.SweepResult(
        case=module.SweepCase(64.0, 64.0),
        sequence="short_test_stereo_64__temporal_64",
        out_dir=tmp_path / "64",
        estimate_tum=tmp_path / "64.tum",
        status_csv=tmp_path / "64.csv",
        returncode=0,
        rmse_m=0.12,
        matched_seconds=10.0,
        accepted_ratio=0.75,
        median_pnp_inliers=24.0,
        median_temporal_matches=40.0,
        command=["ros2", "run", "example"],
    )
    second = module.SweepResult(
        case=module.SweepCase(96.0, 96.0),
        sequence="short_test_stereo_96__temporal_96",
        out_dir=tmp_path / "96",
        estimate_tum=tmp_path / "96.tum",
        status_csv=tmp_path / "96.csv",
        returncode=0,
        rmse_m=0.09,
        matched_seconds=10.0,
        accepted_ratio=0.9,
        median_pnp_inliers=30.0,
        median_temporal_matches=55.0,
        command=["ros2", "run", "example"],
    )

    markdown = module.format_markdown([first, second], args)

    assert "| 96 | 96 | best | 0.0900 |" in markdown
    assert "Best RMSE in this sweep" in markdown


def test_format_markdown_includes_baseline_gap_when_requested(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        sequence="short_test",
        reference=Path("/tmp/ref.tum"),
        translation_scale=0.25,
        baseline_rmse_m=0.02,
    )
    result = module.SweepResult(
        case=module.SweepCase(80.0, 80.0),
        sequence="short_test_stereo_80__temporal_80",
        out_dir=tmp_path / "80",
        estimate_tum=tmp_path / "80.tum",
        status_csv=tmp_path / "80.csv",
        returncode=0,
        rmse_m=0.10,
        matched_seconds=10.0,
        accepted_ratio=0.8,
        median_pnp_inliers=30.0,
        median_temporal_matches=50.0,
        command=["ros2", "run", "example"],
    )

    markdown = module.format_markdown([result], args)

    assert "Baseline RMSE: `0.02` m" in markdown
    assert "| 80 | 80 | best | 0.1000 | 5.00 | 80.0% |" in markdown
    assert "Best gap to baseline: `5.00x`" in markdown


def test_gap_helpers_handle_invalid_values():
    module = load_module()

    assert module.gap_ratio(0.10, 0.02) == 5.0
    assert module.improvement_to_tie_percent(0.10, 0.02) == 80.0
    assert math.isnan(module.gap_ratio(math.nan, 0.02))


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
        "64:64,0:0",
        "--baseline-rmse-m",
        "0.0194",
        "--dry-run",
    ])

    text = summary.read_text(encoding="utf-8")
    assert rc == 0
    assert "Tank Visual Matching Sweep" in text
    assert "Baseline RMSE: `0.0194` m" in text
    assert "stereo_64__temporal_64" in text
    assert "| disabled | disabled | ok | n/a | n/a | n/a% |" in text


def test_load_status_metrics_returns_nan_for_missing_csv(tmp_path):
    module = load_module()

    values = module.load_status_metrics(tmp_path / "missing.csv")

    assert all(math.isnan(value) for value in values)
