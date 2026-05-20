"""Tests for run_tank_visual_fusion_benchmark.py."""

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "run_tank_visual_fusion_benchmark.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_visual_fusion_benchmark", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_default_paths_use_sequence_stem(tmp_path):
    module = load_module()

    paths = module.default_paths(tmp_path, "short/test visual")

    assert paths.fused_tum == tmp_path / "short_test_visual_visual_fused.tum"
    assert paths.visual_status_csv == tmp_path / "short_test_visual_visual_status.csv"
    assert paths.benchmark_row == tmp_path / "short_test_visual_visual_fusion_benchmark.md"
    assert paths.replay_script == tmp_path / "short_test_visual_visual_fusion_replay.sh"
    assert paths.visual_log == tmp_path / "short_test_visual_visual_frontend.log"
    assert paths.imu_log == tmp_path / "short_test_visual_imu_loc.log"
    assert paths.record_log == tmp_path / "short_test_visual_record_odometry.log"
    assert paths.bag_play_log == tmp_path / "short_test_visual_bag_play.log"


def test_build_commands_wire_visual_topic_and_extrinsics(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--bag",
        "/tmp/tank_bag",
        "--reference",
        "/tmp/ref.tum",
        "--out-dir",
        str(tmp_path),
        "--imu-params",
        "/tmp/tank_dataset.yaml",
        "--visual-odom-topic",
        "/visual/fusion/odometry",
        "--translation-scale",
        "0.169623465",
        "--visual-position-variance-floor",
        "0.01",
        "--base-from-camera-x-m",
        "-0.25",
        "--base-from-camera-y-m",
        "-0.45",
        "--max-stereo-descriptor-distance",
        "64",
        "--max-temporal-descriptor-distance",
        "64",
    ])
    paths = module.default_paths(tmp_path, args.sequence)

    visual, imu, recorder, bag = module.build_commands(args, paths)

    assert "topics.odometry:=/visual/fusion/odometry" in visual
    assert "tracking.translation_scale:=0.169623465" in visual
    assert "extrinsics.base_from_camera.x_m:=-0.25" in visual
    assert "extrinsics.base_from_camera.y_m:=-0.45" in visual
    assert "matching.max_stereo_descriptor_distance:=64.0" in visual
    assert "--params-file" in imu
    assert "/tmp/tank_dataset.yaml" in imu
    assert "topics.visual_odometry:=/visual/fusion/odometry" in imu
    assert "imu.visual.position_variance_floor:=0.01" in imu
    assert args.post_play_delay == 2.0
    assert recorder[:3] == ["ros2", "run", "aqua_localization"]
    assert str(paths.fused_tum) in recorder
    assert bag == ["ros2", "bag", "play", "/tmp/tank_bag", "--clock"]


def test_rejects_non_positive_visual_variance():
    module = load_module()

    try:
        module.main([
            "--bag",
            "/tmp/tank_bag",
            "--reference",
            "/tmp/ref.tum",
            "--visual-position-variance-floor",
            "0.0",
        ])
    except ValueError as exc:
        assert "visual-position-variance-floor" in str(exc)
    else:
        raise AssertionError("expected ValueError")
