# SPDX-License-Identifier: Apache-2.0
"""End-to-end determinism gate for the offline replay harness.

Generates the synthetic MBES survey bag, replays it twice through the full
tightly-coupled pipeline (UKF with sonar feedback, sonar registration, pose
graph, MBES loop closure), and requires bit-identical outputs. This is the
Phase 1 "deterministic MBES replay" definition of done, enforced in CI on
data that needs no download.
"""

import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

BIN = os.environ.get("AQUA_REPLAY_BIN", "offline_replay")
SCRIPTS = Path(os.environ.get("AQUA_REPLAY_SCRIPTS", Path(__file__).parents[1] / "scripts"))
SCENARIOS = Path(
    os.environ.get("AQUA_REPLAY_SCENARIOS", Path(__file__).parents[1] / "scenarios"))


def run(cmd, **kwargs):
    result = subprocess.run(cmd, capture_output=True, text=True, **kwargs)
    if result.returncode != 0:
        raise AssertionError(
            f"command failed ({result.returncode}): {' '.join(map(str, cmd))}\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}")
    return result


@pytest.fixture(scope="module")
def synthetic_bag(tmp_path_factory):
    bag = tmp_path_factory.mktemp("bags") / "synthetic_mbes"
    run([sys.executable, str(SCRIPTS / "make_synthetic_mbes_bag.py"),
         "--dst", str(bag), "--duration-s", "60"])
    return bag


def replay(bag, out, *extra):
    run([BIN, "--scenario", str(SCENARIOS / "synthetic_mbes.yaml"),
         "--bag", str(bag), "--out", str(out), "--quiet", *extra])
    with (out / "replay_summary.yaml").open() as f:
        return yaml.safe_load(f)


def test_two_replays_are_bit_identical(synthetic_bag, tmp_path):
    a = replay(synthetic_bag, tmp_path / "run_a")
    b = replay(synthetic_bag, tmp_path / "run_b")

    # The pipeline actually ran end to end.
    outputs = a["outputs"]
    assert outputs["/aqua_imu_loc/odometry"]["count"] > 1000
    assert outputs["/aqua_sonar_loc/odometry"]["count"] > 100
    assert outputs["/aqua_pose_graph/keyframe"]["count"] > 5
    assert outputs["/mbes_loop_closure/status"]["count"] > 0
    assert a["late_messages"] == 0
    assert a["inputs"]["/sonar/points"]["delivered"] == 300

    compare = run([sys.executable, str(SCRIPTS / "compare_replay_summaries.py"),
                   str(tmp_path / "run_a"), str(tmp_path / "run_b")])
    assert "IDENTICAL" in compare.stdout
    assert a["fingerprint"] == b["fingerprint"]


def test_parameter_override_changes_output(synthetic_bag, tmp_path):
    base = replay(synthetic_bag, tmp_path / "base")
    tweaked = replay(synthetic_bag, tmp_path / "tweaked",
                     "--param", "aqua_imu_loc::ImuLocNode:topics.sonar_odometry:=''")
    # Cutting the sonar feedback path changes the UKF trajectory but not the
    # raw sonar registration input count.
    assert (base["outputs"]["/aqua_imu_loc/odometry"]["fnv1a64"] !=
            tweaked["outputs"]["/aqua_imu_loc/odometry"]["fnv1a64"])
    assert tweaked["inputs"]["/sonar/points"]["delivered"] == 300


def load_odometry(bag, topic):
    """Return (N, 4) [t, x, y, z] from a nav_msgs/Odometry topic."""
    import numpy as np
    from rosbags.highlevel import AnyReader

    rows = []
    with AnyReader([Path(bag)]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            p = msg.pose.pose.position
            rows.append((msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec, p.x, p.y, p.z))
    return np.array(rows)


def se3_aligned_rmse(reference, estimate):
    """Translation APE RMSE after interpolating the estimate onto the reference
    stamps and a closed-form rigid (Umeyama, no scale) alignment."""
    import numpy as np

    t0, t1 = max(reference[0, 0], estimate[0, 0]), min(reference[-1, 0], estimate[-1, 0])
    ref = reference[(reference[:, 0] >= t0) & (reference[:, 0] <= t1)]
    est = np.stack([np.interp(ref[:, 0], estimate[:, 0], estimate[:, k]) for k in (1, 2, 3)], 1)
    gt = ref[:, 1:4]
    mu_e, mu_g = est.mean(0), gt.mean(0)
    u, _, vt = np.linalg.svd((gt - mu_g).T @ (est - mu_e))
    d = np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))])
    rotation = u @ d @ vt
    aligned = (rotation @ (est - mu_e).T).T + mu_g
    return float(np.sqrt(np.mean(np.sum((aligned - gt) ** 2, axis=1))))


def test_graph_backend_benchmark_is_deterministic_and_accurate(tmp_path):
    bag = tmp_path / "synthetic_120s"
    run([sys.executable, str(SCRIPTS / "make_synthetic_mbes_bag.py"),
         "--dst", str(bag), "--duration-s", "120"])
    outs = []
    for name in ("bench_a", "bench_b"):
        out = tmp_path / name
        run([BIN, "--scenario", str(SCENARIOS / "synthetic_dvl_benchmark.yaml"),
             "--bag", str(bag), "--out", str(out), "--quiet"])
        with (out / "replay_summary.yaml").open() as f:
            outs.append(yaml.safe_load(f))
    assert outs[0]["fingerprint"] == outs[1]["fingerprint"]

    gt = load_odometry(tmp_path / "bench_a", "/ground_truth/odometry")
    graph = load_odometry(tmp_path / "bench_a", "/aqua_graph/odometry")
    rmse = se3_aligned_rmse(gt, graph)
    print(f"aqua_graph synthetic 120 s APE RMSE: {rmse:.4f} m")
    # Regression guard for the tightly-coupled backend (measured 0.157 m when
    # this test was written); the UKF on the same stream is ~3-4 m.
    assert rmse < 0.4
