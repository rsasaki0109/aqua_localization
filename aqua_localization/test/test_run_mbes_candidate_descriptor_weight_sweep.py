"""Tests for run_mbes_candidate_descriptor_weight_sweep.sh."""

import os
import subprocess
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "run_mbes_candidate_descriptor_weight_sweep.sh"
)


def test_dry_run_prints_weight_cases_and_summary(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "DRY_RUN": "1",
            "WORKSPACE": str(tmp_path),
            "OUT_ROOT": str(tmp_path / "sweep"),
            "WEIGHTS": "0,0.5,1.0",
            "MBES_DURATION": "42",
            "SWEEP_ROS_DOMAIN_ID_START": "41",
            "DATASET": "MBES-SLAM",
            "SEQUENCE": "beach_pond",
            "MBES_LOOP_CANDIDATE_DESCRIPTOR_CENTROID_SCALE_M": "0.7",
            "MBES_LOOP_CANDIDATE_DESCRIPTOR_EXTENT_SCALE": "0.2",
            "MBES_LOOP_CANDIDATE_DESCRIPTOR_POINT_RATIO_SCALE": "0.12",
        }
    )

    proc = subprocess.run(
        [str(SCRIPT_PATH)],
        env=env,
        text=True,
        capture_output=True,
        check=True,
    )

    assert "run_mbes_loop_benchmark.sh" in proc.stdout
    assert "MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT=0" in proc.stdout
    assert "MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT=0.5" in proc.stdout
    assert "MBES_LOOP_CANDIDATE_DESCRIPTOR_WEIGHT=1.0" in proc.stdout
    assert "ROS_DOMAIN_ID=41" in proc.stdout
    assert "ROS_DOMAIN_ID=42" in proc.stdout
    assert "ROS_DOMAIN_ID=43" in proc.stdout
    assert f"OUT_DIR={tmp_path / 'sweep/weight_0p5'}" in proc.stdout
    assert f"MBES_OUT={tmp_path / 'sweep/bags/mbes_weight_1p0'}" in proc.stdout
    assert "summarize_mbes_candidate_descriptor_weight_sweep.py" in proc.stdout
    assert f"--case 0.5:{tmp_path / 'sweep/weight_0p5'}" in proc.stdout
    assert "--descriptor-centroid-scale-m 0.7" in proc.stdout
    assert "--descriptor-extent-scale 0.2" in proc.stdout
    assert "--descriptor-point-ratio-scale 0.12" in proc.stdout
    assert "MBES candidate descriptor-weight sweep artifacts:" in proc.stdout
    assert "ROS_DOMAIN_ID start: 41" in proc.stdout


def test_rejects_invalid_domain_start(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "DRY_RUN": "1",
            "WORKSPACE": str(tmp_path),
            "OUT_ROOT": str(tmp_path / "sweep"),
            "SWEEP_ROS_DOMAIN_ID_START": "253",
        }
    )

    proc = subprocess.run(
        [str(SCRIPT_PATH)],
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 1
    assert "SWEEP_ROS_DOMAIN_ID_START must be an integer from 0 to 232" in proc.stderr
