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
            "LOCAL_SETUP": "/tmp/current_install/setup.bash",
            "MBES_SRC": str(tmp_path / "beach_pond_ros2"),
            "MBES_PREPARE_HUMBLE_METADATA": "1",
            "MBES_OUT": str(tmp_path / "recorded"),
            "MBES_DURATION": "42",
            "OUT_DIR": str(tmp_path / "out"),
            "RECORD_READY_TIMEOUT_S": "33",
            "RECORD_READY_TOPICS": "/aqua_imu_loc/odometry /mbes_loop_closure/status",
            "PLAY_START_DELAY_S": "7",
            "NOTE": "dry run",
            "MBES_LOOP_MIN_POINTS": "120",
            "MBES_LOOP_VOXEL_LEAF_M": "0.25",
            "POSE_GRAPH_ODOMETRY_TOPIC": "/nav/processed/odometry",
            "POSE_GRAPH_KEYFRAME_TRANSLATION_M": "1.0",
            "POSE_GRAPH_OPTIMIZATION_ITERATIONS": "35",
            "POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE": "dcs",
            "POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA": "0.2",
            "MBES_LOOP_MAX_CORRECTION_ROTATION_RAD": "0.4",
            "MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M": "1.0",
            "MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD": "0.2",
            "MBES_LOOP_TRANSLATION_SIGMA_M": "8.0",
            "MBES_LOOP_ROTATION_SIGMA_RAD": "1.2",
            "MBES_LOOP_OPTIMIZE_AFTER_INSERT": "false",
            "MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO": "5.0",
            "MBES_LOOP_CONSISTENCY_MAX_TRANSLATION_DELTA_M": "1.3",
            "MBES_LOOP_CONSISTENCY_MAX_ROTATION_DELTA_RAD": "0.16",
            "MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT": "2",
            "MBES_LOOP_BATCH_CONSISTENCY_AUTO_QUANTILE": "0.9",
            "MBES_LOOP_SELECTION_ALLOWLIST_CSV": str(tmp_path / "selected.csv"),
            "MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S": "0.25",
            "MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA": "0.02",
            "MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M": "0.4",
            "MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD": "0.05",
            "MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES": "false",
            "PLAY_TOPIC_ARGS": "--topics /norbit/detections",
            "AUDIT_MAX_ACCEPTED": "77",
            "AUDIT_MAX_MARKERS": "88",
            "AUDIT_PLOT_ALLOW_EMPTY": "1",
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
    assert "prepare_rosbag2_humble_metadata.py" in proc.stdout
    assert "rosbags-convert" in proc.stdout
    assert "--dst-storage sqlite3" in proc.stdout
    assert "record_mbes_demo.sh" in proc.stdout
    assert "export_mbes_loop_status.py" in proc.stdout
    assert "--consistency-sweep-out" in proc.stdout
    assert "--consistency-rejection-audit-out" in proc.stdout
    assert "--batch-consistency-out" in proc.stdout
    assert "--batch-consistency-selected-csv-out" in proc.stdout
    assert "mbes_loop_benchmark_row.py" in proc.stdout
    assert "mbes_loop_trajectory_metrics.py" in proc.stdout
    assert "audit_mbes_loop_candidates.py" in proc.stdout
    assert "plot_mbes_loop_audit.py" in proc.stdout
    assert "audit_mbes_loop_geometry.py" in proc.stdout
    assert "--duration 42" in proc.stdout
    assert "--note dry\\ run" in proc.stdout
    assert "MBES_LOOP_MIN_POINTS=120" in proc.stdout
    assert "MBES_LOOP_VOXEL_LEAF_M=0.25" in proc.stdout
    assert "POSE_GRAPH_ODOMETRY_TOPIC=/nav/processed/odometry" in proc.stdout
    assert "POSE_GRAPH_KEYFRAME_TRANSLATION_M=1.0" in proc.stdout
    assert "POSE_GRAPH_OPTIMIZATION_ITERATIONS=35" in proc.stdout
    assert "POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE=dcs" in proc.stdout
    assert "POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA=0.2" in proc.stdout
    assert "MBES_LOOP_MAX_CORRECTION_ROTATION_RAD=0.4" in proc.stdout
    assert "MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M=1.0" in proc.stdout
    assert "MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD=0.2" in proc.stdout
    assert "MBES_LOOP_TRANSLATION_SIGMA_M=8.0" in proc.stdout
    assert "MBES_LOOP_ROTATION_SIGMA_RAD=1.2" in proc.stdout
    assert "MBES_LOOP_OPTIMIZE_AFTER_INSERT=false" in proc.stdout
    assert "MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO=5.0" in proc.stdout
    assert "MBES_LOOP_CONSISTENCY_MAX_TRANSLATION_DELTA_M=1.3" in proc.stdout
    assert "MBES_LOOP_CONSISTENCY_MAX_ROTATION_DELTA_RAD=0.16" in proc.stdout
    assert "MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT=2" in proc.stdout
    assert f"MBES_LOOP_SELECTION_ALLOWLIST_CSV={tmp_path / 'selected.csv'}" in proc.stdout
    assert "MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S=0.25" in proc.stdout
    assert "MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA=0.02" in proc.stdout
    assert "MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M=0.4" in proc.stdout
    assert "MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD=0.05" in proc.stdout
    assert "MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES=false" in proc.stdout
    assert f"MBES_SRC={tmp_path / 'out/mbes_source_humble_sqlite'}" in proc.stdout
    assert "PLAY_TOPIC_ARGS=--topics\\ /norbit/detections" in proc.stdout
    assert "LOCAL_SETUP=/tmp/current_install/setup.bash" in proc.stdout
    assert "RECORD_READY_TIMEOUT_S=33" in proc.stdout
    assert "RECORD_READY_TOPICS=/aqua_imu_loc/odometry\\ /mbes_loop_closure/status" in proc.stdout
    assert "PLAY_START_DELAY_S=7" in proc.stdout
    assert "--max-rotation-rad 0.4" in proc.stdout
    assert "--descriptor-extent-warn 5.0" in proc.stdout
    assert "--consistency-min-support-count 2" in proc.stdout
    assert "--batch-consistency-translation-threshold-m 1.3" in proc.stdout
    assert "--batch-consistency-rotation-threshold-rad 0.16" in proc.stdout
    assert "--batch-consistency-auto-quantile 0.9" in proc.stdout
    assert "--pose-graph-path-topic" not in proc.stdout
    assert "--max-accepted 77" in proc.stdout
    assert "--max-markers 88" in proc.stdout
    assert "--allow-empty" in proc.stdout
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
    assert str(tmp_path / "out/mbes_beach_pond_consistency_sweep.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_consistency_rejections.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_batch_consistency.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_batch_selected_loops.csv") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_benchmark_row.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_trajectory_metrics.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_trajectory_metrics") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_audit.md") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_audit.png") in proc.stdout
    assert str(tmp_path / "out/mbes_beach_pond_loop_geometry.md") in proc.stdout
    assert "--max-accepted 100" in proc.stdout
    assert "--max-markers 100" in proc.stdout
    assert "--allow-empty" in proc.stdout
    assert "--require-complete" in proc.stdout
