"""Tests for run_mbes_selected_loop_replay_comparison.sh."""

import os
import subprocess
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "run_mbes_selected_loop_replay_comparison.sh"
)


def test_dry_run_prints_normal_selected_and_comparison_commands(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "DRY_RUN": "1",
            "WORKSPACE": str(tmp_path / "ws"),
            "OUT_ROOT": str(tmp_path / "comparison"),
            "MBES_DURATION": "5",
            "LOCAL_SETUP": "/tmp/current_install/setup.bash",
            "NORMAL_ROS_DOMAIN_ID": "31",
            "SELECTED_ROS_DOMAIN_ID": "131",
            "POSE_GRAPH_ODOMETRY_TOPIC": "/nav/processed/odometry",
            "POSE_GRAPH_OPTIMIZATION_ITERATIONS": "35",
            "POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE": "dcs",
            "POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA": "0.2",
            "MBES_LOOP_MIN_POINTS": "120",
            "MBES_LOOP_VOXEL_LEAF_M": "0.25",
            "MBES_LOOP_MIN_KEYFRAME_SEPARATION": "40",
            "MBES_LOOP_MAX_CORRECTION_ROTATION_RAD": "0.4",
            "MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M": "1.0",
            "MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD": "0.2",
            "MBES_LOOP_TRANSLATION_SIGMA_M": "8.0",
            "MBES_LOOP_ROTATION_SIGMA_RAD": "1.2",
            "MBES_LOOP_OPTIMIZE_AFTER_INSERT": "false",
            "MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO": "5.0",
            "MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT": "2",
            "MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S": "0.25",
            "MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA": "0.02",
            "MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M": "0.4",
            "MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD": "0.05",
            "MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES": "false",
        }
    )

    proc = subprocess.run(
        [str(SCRIPT_PATH)],
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )

    normal_dir = tmp_path / "comparison/normal"
    selected_dir = tmp_path / "comparison/selected"
    selected_csv = normal_dir / "mbes_beach_pond_batch_selected_loops.csv"
    selected_status = selected_dir / "mbes_beach_pond_loop_status.csv"
    assert "run_mbes_loop_benchmark.sh" in proc.stdout
    assert "ROS_DOMAIN_ID=31" in proc.stdout
    assert "ROS_DOMAIN_ID=131" in proc.stdout
    assert "POSE_GRAPH_ODOMETRY_TOPIC=/nav/processed/odometry" in proc.stdout
    assert "POSE_GRAPH_OPTIMIZATION_ITERATIONS=35" in proc.stdout
    assert "POSE_GRAPH_LOOP_ROBUST_KERNEL_TYPE=dcs" in proc.stdout
    assert "POSE_GRAPH_LOOP_ROBUST_KERNEL_DELTA=0.2" in proc.stdout
    assert "MBES_LOOP_MIN_POINTS=120" in proc.stdout
    assert "MBES_LOOP_VOXEL_LEAF_M=0.25" in proc.stdout
    assert "MBES_LOOP_MIN_KEYFRAME_SEPARATION=40" in proc.stdout
    assert "MBES_LOOP_MAX_CORRECTION_ROTATION_RAD=0.4" in proc.stdout
    assert "MBES_LOOP_MIN_PLAN_VIEW_SEPARATION_M=1.0" in proc.stdout
    assert "MBES_LOOP_MAX_SHORT_PLAN_VIEW_ROTATION_RAD=0.2" in proc.stdout
    assert "MBES_LOOP_TRANSLATION_SIGMA_M=8.0" in proc.stdout
    assert "MBES_LOOP_ROTATION_SIGMA_RAD=1.2" in proc.stdout
    assert "MBES_LOOP_OPTIMIZE_AFTER_INSERT=false" in proc.stdout
    assert "MBES_LOOP_DESCRIPTOR_MAX_EXTENT_RATIO=5.0" in proc.stdout
    assert "MBES_LOOP_CONSISTENCY_MIN_SUPPORT_COUNT=2" in proc.stdout
    assert "MBES_LOOP_SELECTION_MATCH_TIMESTAMP_WINDOW_S=0.25" in proc.stdout
    assert "MBES_LOOP_SELECTION_MATCH_MAX_FITNESS_DELTA=0.02" in proc.stdout
    assert "MBES_LOOP_SELECTION_MATCH_MAX_TRANSLATION_DELTA_M=0.4" in proc.stdout
    assert "MBES_LOOP_SELECTION_MATCH_MAX_ROTATION_DELTA_RAD=0.05" in proc.stdout
    assert "MBES_LOOP_SELECTION_PRIORITIZE_CANDIDATES=false" in proc.stdout
    assert f"WORKSPACE={tmp_path / 'ws'}" in proc.stdout
    assert f"OUT_DIR={normal_dir}" in proc.stdout
    assert f"OUT_DIR={selected_dir}" in proc.stdout
    assert f"MBES_LOOP_SELECTION_ALLOWLIST_CSV={selected_csv}" in proc.stdout
    assert "GEOMETRY_AUDIT_REQUIRE_COMPLETE=0" in proc.stdout
    assert "compare_mbes_loop_metric_reports.py" in proc.stdout
    assert "--normal" in proc.stdout
    assert "--selected" in proc.stdout
    assert "mbes_selected_loop_replay_comparison.md" in proc.stdout
    assert "diagnose_mbes_selected_loop_coverage.py" in proc.stdout
    assert "--timestamp-window-s 1.0" in proc.stdout
    assert "mbes_selected_loop_coverage.md" in proc.stdout
    assert "remap_mbes_selected_loop_allowlist.py" in proc.stdout
    assert "mbes_selected_loop_remapped_allowlist.csv" in proc.stdout
    assert "mbes_selected_loop_remap.md" in proc.stdout
    assert "check_mbes_loop_allowlist_replay.py" in proc.stdout
    assert f"--status {selected_status}" in proc.stdout
    assert "--strict" in proc.stdout
    assert "--signature-timestamp-window-s 0.25" in proc.stdout
    assert "--signature-max-fitness-delta 0.02" in proc.stdout
    assert "--signature-max-translation-delta-m 0.4" in proc.stdout
    assert "--signature-max-rotation-delta-rad 0.05" in proc.stdout
    assert "mbes_selected_loop_allowlist_audit.md" in proc.stdout
    assert "normal ROS_DOMAIN_ID: 31" in proc.stdout
    assert "selected ROS_DOMAIN_ID: 131" in proc.stdout
    assert "remapped allowlist:" in proc.stdout
    assert "remap report:" in proc.stdout
    assert "MBES selected-loop replay comparison artifacts:" in proc.stdout


def test_rejects_invalid_selected_ros_domain_id(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "DRY_RUN": "1",
            "WORKSPACE": str(tmp_path / "ws"),
            "OUT_ROOT": str(tmp_path / "comparison"),
            "NORMAL_ROS_DOMAIN_ID": "31",
            "SELECTED_ROS_DOMAIN_ID": "253",
        }
    )

    proc = subprocess.run(
        [str(SCRIPT_PATH)],
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode != 0
    assert "SELECTED_ROS_DOMAIN_ID must be an integer from 0 to 232" in proc.stderr
