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
    assert "check_mbes_loop_allowlist_replay.py" in proc.stdout
    assert f"--status {selected_status}" in proc.stdout
    assert "--strict" in proc.stdout
    assert "mbes_selected_loop_allowlist_audit.md" in proc.stdout
    assert "MBES selected-loop replay comparison artifacts:" in proc.stdout
