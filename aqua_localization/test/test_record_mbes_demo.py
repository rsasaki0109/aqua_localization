"""Tests for record_mbes_demo.sh launch-time validation."""

import os
import subprocess
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "record_mbes_demo.sh"


def test_rejects_invalid_ros_domain_id(tmp_path):
    env = os.environ.copy()
    env.update(
        {
            "WORKSPACE": str(tmp_path),
            "ROS_DOMAIN_ID": "253",
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
    assert "ROS_DOMAIN_ID must be an integer from 0 to 232" in proc.stderr
