"""Pure tests for run_tank_visual_benchmark.py."""

import importlib.util
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "run_tank_visual_benchmark.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_visual_benchmark", SCRIPT_PATH)
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
        x = 0.2 * float(i)
        y = 0.05 * float(i % 5)
        z = 0.03 * float(i % 3)
        rows.append([t, x * scale, y * scale, z * scale, 0.0, 0.0, 0.0, 1.0])
    return rows


def test_default_paths_sanitize_sequence_name(tmp_path):
    module = load_module()

    paths = module.default_paths(tmp_path, "Tank short/test")

    assert paths.estimate_tum == tmp_path / "Tank_short_test_visual_frontend.tum"
    assert paths.status_csv == tmp_path / "Tank_short_test_visual_frontend_status.csv"
    assert paths.status_summary == tmp_path / "Tank_short_test_visual_frontend_status.md"
    assert paths.drift_report == tmp_path / "Tank_short_test_visual_drift.md"
    assert paths.scale_report == tmp_path / "Tank_short_test_visual_scale_report.txt"
    assert paths.benchmark_row == tmp_path / "Tank_short_test_visual_benchmark.md"
    assert paths.replay_script == tmp_path / "Tank_short_test_visual_replay.sh"


def test_build_commands_include_camera_scale_and_clock(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--bag", "/tmp/tank_bag",
        "--reference", "/tmp/ref.tum",
        "--out-dir", str(tmp_path),
        "--translation-scale", "0.25",
        "--play-rate", "0.5",
    ])
    paths = module.default_paths(tmp_path, args.sequence)

    visual_command = module.build_visual_command(args)
    record_command = module.build_record_command(args.odom_topic, paths.estimate_tum)
    bag_command = module.build_bag_play_command(args)

    assert "tracking.translation_scale:=0.25" in visual_command
    assert "camera.bf:=78.89165891925023" in visual_command
    assert record_command[-2:] == ["--format", "tum"]
    assert str(paths.estimate_tum) in record_command
    assert bag_command == ["ros2", "bag", "play", "/tmp/tank_bag", "--clock", "--rate", "0.5"]


def test_build_visual_command_can_enable_status_csv(tmp_path):
    module = load_module()
    status_csv = tmp_path / "visual_status.csv"
    args = module.parse_args([
        "--bag", "/tmp/tank_bag",
        "--reference", "/tmp/ref.tum",
        "--status-csv", str(status_csv),
    ])

    command = module.build_visual_command(args)

    assert f"diagnostics.status_csv_path:={status_csv}" in command


def test_write_replay_script_quotes_paths(tmp_path):
    module = load_module()
    path = tmp_path / "replay.sh"

    module.write_replay_script(path, [["ros2", "bag", "play", "/tmp/with space/bag"]])

    text = path.read_text(encoding="utf-8")
    assert text.startswith("#!/usr/bin/env bash\nset -euo pipefail\n")
    assert "'/tmp/with space/bag'" in text
    assert path.stat().st_mode & 0o111


def test_evaluate_writes_scale_report_and_markdown_row(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0))
    args = module.parse_args([
        "--estimate", str(est),
        "--reference", str(ref),
        "--out-dir", str(tmp_path),
        "--sequence", "short_test",
        "--translation-scale", "1.0",
        "--drift-window-s", "10.0",
        "--drift-min-samples", "2",
    ])
    paths = module.default_paths(tmp_path, args.sequence)

    text = module.evaluate(args, est, paths)

    assert "recommended tracking.translation_scale: 0.250000000" in text
    assert paths.scale_report.exists()
    assert paths.benchmark_row.exists()
    assert paths.drift_report.exists()
    assert "Visual Drift Analysis" in paths.drift_report.read_text(encoding="utf-8")
    assert "same-sequence Sim(3) scale diagnostic=0.250000000" in paths.benchmark_row.read_text(
        encoding="utf-8")


def test_evaluate_writes_status_summary_when_csv_is_available(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0))
    status_csv = tmp_path / "visual_status.csv"
    status_csv.write_text(
        "timestamp,frame_index,accepted_count,rejected_count,left_features,right_features,"
        "stereo_matches,stereo_points,temporal_matches,pnp_inliers,inlier_ratio,"
        "step_translation_m,accepted,status\n"
        "1.000000000,1,1,0,100,95,80,70,60,50,0.800000000,0.050000000,1,accepted\n",
        encoding="utf-8",
    )
    args = module.parse_args([
        "--estimate", str(est),
        "--reference", str(ref),
        "--out-dir", str(tmp_path),
        "--sequence", "short_test",
        "--status-csv", str(status_csv),
        "--drift-window-s", "10.0",
        "--drift-min-samples", "2",
    ])
    paths = module.default_paths(tmp_path, args.sequence)

    text = module.evaluate(args, est, paths)

    assert f"status csv: {status_csv}" in text
    assert paths.status_summary.exists()
    assert "Visual Frontend Status Summary" in paths.status_summary.read_text(encoding="utf-8")


def test_rejects_non_positive_scale():
    module = load_module()
    with pytest.raises(ValueError, match="translation-scale"):
        module.main([
            "--estimate", "/tmp/est.tum",
            "--reference", "/tmp/ref.tum",
            "--translation-scale",
            "0.0",
        ])
