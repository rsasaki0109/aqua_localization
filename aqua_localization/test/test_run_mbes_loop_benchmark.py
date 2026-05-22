"""Tests for run_mbes_loop_benchmark.sh."""

import os
import subprocess
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "run_mbes_loop_benchmark.sh"


def test_dry_run_prints_pipeline_commands(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "DRY_RUN": "1",
            "WORKSPACE": str(tmp_path),
            "MBES_SRC": str(tmp_path / "beach_pond_ros2"),
            "MBES_OUT": str(tmp_path / "recorded"),
            "MBES_DURATION": "42",
            "OUT_DIR": str(tmp_path / "out"),
            "NOTE": "dry run",
            "MBES_LOOP_MIN_POINTS": "120",
            "MBES_LOOP_VOXEL_LEAF_M": "0.25",
            "POSE_GRAPH_KEYFRAME_TRANSLATION_M": "1.0",
            "MBES_LOOP_MAX_CORRECTION_ROTATION_RAD": "0.4",
            "MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO": "5.0",
            "PLAY_TOPIC_ARGS": "--topics /norbit/detections",
            "AUDIT_MAX_ACCEPTED": "77",
            "AUDIT_MAX_MARKERS": "88",
        }
    )

    proc = subprocess.run(
        [str(SCRIPT_PATH)],
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )

    assert "check_mbes_benchmark_ready.py" in proc.stdout
    assert "record_mbes_demo.sh" in proc.stdout
    assert "export_mbes_loop_status.py" in proc.stdout
    assert "mbes_loop_benchmark_row.py" in proc.stdout
    assert "audit_mbes_loop_candidates.py" in proc.stdout
    assert "plot_mbes_loop_audit.py" in proc.stdout
    assert "audit_mbes_loop_geometry.py" in proc.stdout
    assert "--duration 42" in proc.stdout
    assert "--note dry\\ run" in proc.stdout
    assert "MBES_LOOP_MIN_POINTS=120" in proc.stdout
    assert "MBES_LOOP_VOXEL_LEAF_M=0.25" in proc.stdout
    assert "POSE_GRAPH_KEYFRAME_TRANSLATION_M=1.0" in proc.stdout
    assert "MBES_LOOP_MAX_CORRECTION_ROTATION_RAD=0.4" in proc.stdout
    assert "MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO=5.0" in proc.stdout
    assert "PLAY_TOPIC_ARGS=--topics\\ /norbit/detections" in proc.stdout
    assert "--max-rotation-rad 0.4" in proc.stdout
    assert "--descriptor-extent-warn 5.0" in proc.stdout
    assert "--max-accepted 77" in proc.stdout
    assert "--max-markers 88" in proc.stdout
    assert "--require-complete" in proc.stdout
    assert "MBES loop benchmark artifacts:" in proc.stdout


def test_dry_run_uses_default_artifact_names(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "DRY_RUN": "1",
            "WORKSPACE": str(tmp_path),
            "OUT_DIR": str(tmp_path / "out"),
        }
    )

    proc = subprocess.run(
        [str(SCRIPT_PATH)],
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )

    assert str(tmp_path / "datasets/public/mbes_slam/beach_pond_ros2") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_readiness.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_status.csv") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_descriptor_sweep.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_benchmark_row.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_audit.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_audit.png") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_geometry.md") in proc.stdout
    assert "--max-accepted 100" in proc.stdout
    assert "--max-markers 100" in proc.stdout
    assert "--require-complete" in proc.stdout
