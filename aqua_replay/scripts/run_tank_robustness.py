#!/usr/bin/env python3
"""Sensor-outage robustness sweep for aqua_graph on a Tank Dataset bag.

Replays the same bag with simulated outages and reports how gracefully the
estimate degrades:

- camera outages: the stereo frontend publishes nothing during the window
  and every feature track is lost (make_stereo_feature_track.py --drop);
- DVL outages: offline_replay drops /dvl/twist in the window (--drop-topic);
- both at once (IMU + pressure only).

Windows are seconds from the start of the bag. Every row is a deterministic
replay, so the table is reproducible from this command:

    run_tank_robustness.py --bag short_test_ros2 --out-dir /tmp/tank_robustness
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

DEFAULT_CASES = [
    # name, camera outage windows, DVL outage windows
    ("nominal", [], []),
    ("camera out 2 s", [(6.0, 8.0)], []),
    ("camera out 4 s", [(5.0, 9.0)], []),
    ("camera out 6 s", [(5.0, 11.0)], []),
    ("camera out 10 s", [(4.0, 14.0)], []),
    ("DVL out 4 s", [], [(5.0, 9.0)]),
    ("DVL out 10 s", [], [(4.0, 14.0)]),
    ("camera + DVL out 2 s", [(6.0, 8.0)], [(6.0, 8.0)]),
    ("camera + DVL out 4 s", [(5.0, 9.0)], [(5.0, 9.0)]),
]


def run(cmd, **kwargs):
    print("+ " + " ".join(str(c) for c in cmd), file=sys.stderr)
    return subprocess.run([str(c) for c in cmd], check=True, **kwargs)


def find_share(package: str) -> Path:
    prefix = subprocess.run(["ros2", "pkg", "prefix", package], check=True,
                            capture_output=True, text=True).stdout.strip()
    return Path(prefix) / "share" / package


def score(bag: Path, reference: str, extra=()):
    out = bag.parent / (bag.name + ("_last" if extra else "_full") + ".json")
    run([sys.executable, SCRIPT_DIR / "evaluate_replay_trajectories.py", "--bag", bag,
         "--reference", reference, "--estimate", "/aqua_graph/odometry",
         "--estimate", "/aqua_imu_loc/odometry", "--json", out, *extra],
        stdout=subprocess.DEVNULL)
    return json.loads(out.read_text())["rows"]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--bag", required=True, type=Path)
    p.add_argument("--out-dir", required=True, type=Path)
    p.add_argument("--scenario", type=Path)
    p.add_argument("--reference", default="/apriltag_slam/GT")
    p.add_argument("--baseline-window-s", type=float, default=11.65)
    p.add_argument("--max-features", type=int, default=100)
    p.add_argument("--publish-every", type=int, default=2)
    p.add_argument("--markdown", type=Path)
    args = p.parse_args(argv)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    scenario = args.scenario or find_share("aqua_replay") / "scenarios" / "tank_short_test_vio.yaml"
    replay_bin = os.environ.get("AQUA_REPLAY_BIN", "")
    replay_cmd = [replay_bin] if replay_bin else ["ros2", "run", "aqua_replay", "offline_replay"]

    rows = []
    for i, (name, camera_drops, dvl_drops) in enumerate(DEFAULT_CASES):
        case_dir = args.out_dir / f"case{i:02d}"
        features = case_dir / "features"
        replay = case_dir / "replay"
        if replay.exists():
            raise SystemExit(f"{replay} exists; choose a fresh --out-dir")
        case_dir.mkdir(parents=True, exist_ok=True)
        feature_cmd = [sys.executable, SCRIPT_DIR / "make_stereo_feature_track.py",
                       "--bag", args.bag, "--dst", features, "--overwrite",
                       "--max-features", args.max_features, "--publish-every", args.publish_every]
        for a, b in camera_drops:
            feature_cmd += ["--drop", f"{a}:{b}"]
        run(feature_cmd, stdout=subprocess.DEVNULL)
        cmd = replay_cmd + ["--scenario", scenario, "--bag", args.bag, "--extra-bag", features,
                            "--out", replay, "--quiet"]
        for a, b in dvl_drops:
            cmd += ["--drop-topic", f"/dvl/twist:{a}:{b}"]
        run(cmd, stdout=subprocess.DEVNULL)
        full = score(replay, args.reference)
        last = score(replay, args.reference, ("--last-s", f"{args.baseline_window_s}"))
        rows.append((name, full, last))

    lines = [
        f"| Outage | aqua_graph RMSE m (last {args.baseline_window_s:g} s) | aqua_graph max m | "
        "aqua_graph RMSE m (full) | UKF RMSE m (full) |",
        "|--------|------:|------:|------:|------:|",
    ]
    for name, full, last in rows:
        g_last = last["/aqua_graph/odometry"]
        g_full = full["/aqua_graph/odometry"]
        u_full = full["/aqua_imu_loc/odometry"]
        lines.append(f"| {name} | {g_last['rmse_m']:.4f} | {g_full['max_m']:.4f} | "
                     f"{g_full['rmse_m']:.4f} | {u_full['rmse_m']:.4f} |")
    text = "\n".join(lines) + "\n"
    print(text)
    if args.markdown:
        args.markdown.write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
