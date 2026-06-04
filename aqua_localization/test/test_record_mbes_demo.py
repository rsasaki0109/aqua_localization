"""Tests for record_mbes_demo.sh launch-time validation."""

import os
import subprocess
import time
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


def _alive_non_zombie_pids(pids):
    if not pids:
        return []
    proc = subprocess.run(
        ["ps", "-o", "pid=,stat=,cmd=", "-p", ",".join(pids)],
        text=True,
        capture_output=True,
        check=False,
    )
    alive = []
    for line in proc.stdout.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) >= 2 and not parts[1].startswith("Z"):
            alive.append(parts[0])
    return alive


def test_strict_recorder_readiness_cleans_up_background_processes(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_log = tmp_path / "fake_ros2_pids.log"
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/usr/bin/env bash\n"
        "echo \"$$ $*\" >> \"$FAKE_ROS2_LOG\"\n"
        "exec sleep 60\n",
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)
    ros_setup = tmp_path / "ros_setup.bash"
    local_setup = tmp_path / "local_setup.bash"
    ros_setup.write_text("", encoding="utf-8")
    local_setup.write_text("", encoding="utf-8")

    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{fake_bin}:{env['PATH']}",
            "FAKE_ROS2_LOG": str(fake_log),
            "WORKSPACE": str(tmp_path),
            "ROS_SETUP": str(ros_setup),
            "LOCAL_SETUP": str(local_setup),
            "MBES_SRC": str(tmp_path / "source_bag"),
            "MBES_OUT": str(tmp_path / "recorded_bag"),
            "RECORD_READY_TIMEOUT_S": "1",
            "RECORD_READY_STRICT": "1",
            "PLAY_START_DELAY_S": "0",
        }
    )

    proc = subprocess.run(
        [str(SCRIPT_PATH)],
        env=env,
        text=True,
        capture_output=True,
        check=False,
        timeout=20,
    )

    assert proc.returncode != 0
    assert "strict recorder readiness is enabled; aborting replay" in proc.stderr
    pids = [line.split()[0] for line in fake_log.read_text(encoding="utf-8").splitlines()]
    for _ in range(20):
        alive = _alive_non_zombie_pids(pids)
        if not alive:
            break
        time.sleep(0.1)
    assert alive == []
