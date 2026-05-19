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
    ])
    paths = module.default_paths(tmp_path, args.sequence)

    text = module.evaluate(args, est, paths)

    assert "recommended tracking.translation_scale: 0.250000000" in text
    assert paths.scale_report.exists()
    assert paths.benchmark_row.exists()
    assert "same-sequence Sim(3) scale diagnostic=0.250000000" in paths.benchmark_row.read_text(
        encoding="utf-8")


def test_rejects_non_positive_scale():
    module = load_module()
    with pytest.raises(ValueError, match="translation-scale"):
        module.main([
            "--estimate", "/tmp/est.tum",
            "--reference", "/tmp/ref.tum",
            "--translation-scale",
            "0.0",
        ])
