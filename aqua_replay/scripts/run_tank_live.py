#!/usr/bin/env python3
"""Live (real-time) Tank Dataset run of the online stereo frontend + aqua_graph.

Unlike offline_replay, nothing here is deterministic: the bag is played at
real-time rate with `ros2 bag play --clock`, the nodes run in a
multi-threaded component container (aqua_visual_frontend's
tank_vio_live.launch.py), and frames that arrive while the frontend is busy
are dropped exactly as on a vehicle. The outputs are recorded on the sim
clock, so each odometry message's receive time minus its header stamp is
the end-to-end latency.

    run_tank_live.py --bag short_test_ros2 --out-dir /tmp/tank_live --runs 3
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
RECORD_TOPICS = [
    "/apriltag_slam/GT",
    "/aqua_graph/odometry",
    "/aqua_graph/status",
    "/aqua_visual_frontend/stereo_features",
    "/aqua_visual_frontend/processing_time_ms",
]


def start(cmd, env, log):
    return subprocess.Popen([str(c) for c in cmd], env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)


def stop(proc, timeout=20.0):
    if proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()


def latency_and_timing(bag: Path):
    from rosbags.highlevel import AnyReader

    lat, proc_ms, features = [], [], 0
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic in RECORD_TOPICS[1:]]
        for conn, t_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == "/aqua_graph/odometry":
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                lat.append((t_ns - stamp) * 1e-6)
            elif conn.topic.endswith("processing_time_ms"):
                proc_ms.append(msg.data)
            elif conn.topic.endswith("stereo_features"):
                features += 1
    return np.array(lat), np.array(proc_ms), features


def one_run(args, run_dir: Path, env) -> dict:
    run_dir.mkdir(parents=True, exist_ok=True)
    out_bag = run_dir / "live"
    logs = {name: open(run_dir / f"{name}.log", "w") for name in ("launch", "record", "play")}
    launch = start(["ros2", "launch", "aqua_visual_frontend", "tank_vio_live.launch.py",
                    "use_sim_time:=true"], env, logs["launch"])
    record = start(["ros2", "bag", "record", "--use-sim-time", "-s", "mcap", "-o", out_bag,
                    *RECORD_TOPICS], env, logs["record"])
    time.sleep(args.startup_s)  # discovery + subscriptions before playback starts
    # 1 kHz /clock so sim-time receive stamps (latency) resolve to ~1 ms.
    play = start(["ros2", "bag", "play", "--clock", "1000", "--rate", str(args.rate), args.bag],
                 env, logs["play"])
    play.wait()
    time.sleep(args.drain_s)  # let the smoother publish its last states
    stop(record)
    stop(launch)
    for log in logs.values():
        log.close()

    score = {}
    for name, extra in (("full", []), ("last", ["--last-s", str(args.baseline_window_s)])):
        js = run_dir / f"ape_{name}.json"
        subprocess.run([sys.executable, SCRIPT_DIR / "evaluate_replay_trajectories.py",
                        "--bag", out_bag, "--reference", "/apriltag_slam/GT",
                        "--estimate", "/aqua_graph/odometry", "--json", js, *extra],
                       check=True, stdout=subprocess.DEVNULL)
        score[name] = json.loads(js.read_text())["rows"]["/aqua_graph/odometry"]
    lat, proc_ms, features = latency_and_timing(out_bag)
    return {
        "rmse_full_m": score["full"]["rmse_m"],
        "rmse_last_m": score["last"]["rmse_m"],
        "feature_frames": features,
        "frontend_frames": int(proc_ms.size),
        "frontend_ms_p50": float(np.median(proc_ms)) if proc_ms.size else float("nan"),
        "frontend_ms_p95": float(np.percentile(proc_ms, 95)) if proc_ms.size else float("nan"),
        "odom_latency_ms_p50": float(np.median(lat)) if lat.size else float("nan"),
        "odom_latency_ms_p95": float(np.percentile(lat, 95)) if lat.size else float("nan"),
    }


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--bag", required=True, type=Path)
    p.add_argument("--out-dir", required=True, type=Path)
    p.add_argument("--runs", type=int, default=3)
    p.add_argument("--rate", type=float, default=1.0, help="Playback rate (1.0 = real time).")
    p.add_argument("--baseline-window-s", type=float, default=11.65)
    p.add_argument("--startup-s", type=float, default=4.0)
    p.add_argument("--drain-s", type=float, default=2.0)
    p.add_argument("--domain-id", type=int, default=77)
    args = p.parse_args(argv)
    if args.out_dir.exists():
        raise SystemExit(f"{args.out_dir} exists; choose a fresh --out-dir")
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id),
               ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    rows = [one_run(args, args.out_dir / f"run{i}", env) for i in range(args.runs)]
    (args.out_dir / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")
    print("| Run | RMSE full m | RMSE last %g s m | feature frames | frontend ms p50/p95 | "
          "odometry latency ms p50/p95 |" % args.baseline_window_s)
    print("|----:|------:|------:|------:|------:|------:|")
    for i, r in enumerate(rows):
        print(f"| {i} | {r['rmse_full_m']:.4f} | {r['rmse_last_m']:.4f} | {r['feature_frames']} | "
              f"{r['frontend_ms_p50']:.1f} / {r['frontend_ms_p95']:.1f} | "
              f"{r['odom_latency_ms_p50']:.1f} / {r['odom_latency_ms_p95']:.1f} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
