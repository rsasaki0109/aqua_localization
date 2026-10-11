#!/usr/bin/env python3
"""Reproduce the beach_pond dead-reckoning benchmark (aqua_graph vs classical DR).

One command from the converted MBES-SLAM beach_pond bags to the markdown
tables in docs/benchmarks/beach_pond_dead_reckoning.md:

1. replay aqua_graph deterministically with the GNSS compass
   (beach_pond_graph scenario) and without it (beach_pond_graph_no_compass);
2. compute classical DVL dead reckoning with compass, AHRS and integrated-gyro
   yaw (dvl_dead_reckoning_baseline.py);
3. score every trajectory against the RTK GNSS-INS reference: SE(3)-aligned
   APE at the reference's base_link origin plus segment drift in % of path.

    run_beach_pond_benchmark.py --bag beach_pond_ros2 --aux-bag beach_pond_aux \\
        --out-dir /tmp/beach_pond_bench

GNSS position is never an input; only the dual-antenna heading is, and only
in the "compass" rows.
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REFERENCE = "/nav/processed/odometry"
# base_link origin in the ustrain_imu frame (from the bag's /tf_static):
# the estimates are IMU poses, the reference is base_link.
BODY_OFFSET = ("-0.057", "-0.187", "0.365")
SEGMENTS = ("100", "500", "1000", "2000")


def find_share(package: str) -> Path:
    prefix = subprocess.run(["ros2", "pkg", "prefix", package], check=True,
                            capture_output=True, text=True).stdout.strip()
    return Path(prefix) / "share" / package


def git_commit() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], check=True,
                              capture_output=True, text=True, cwd=SCRIPT_DIR).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def run(cmd, **kwargs):
    print("+ " + " ".join(str(c) for c in cmd), file=sys.stderr)
    return subprocess.run([str(c) for c in cmd], check=True, **kwargs)


def fingerprint(replay: Path) -> str:
    for line in (replay / "replay_summary.yaml").read_text().splitlines():
        if line.strip().startswith("fingerprint:"):
            return line.split(":", 1)[1].strip().strip('"')
    return "unknown"


def score(bags, estimates, out: Path, window=()):
    cmd = [sys.executable, SCRIPT_DIR / "evaluate_replay_trajectories.py",
           "--reference", REFERENCE, "--json", out, "--body-offset", *BODY_OFFSET,
           "--segment-lengths", *SEGMENTS, *window]
    for bag in bags:
        cmd += ["--bag", bag]
    for topic in estimates:
        cmd += ["--estimate", topic]
    run(cmd, stdout=subprocess.DEVNULL)
    return json.loads(out.read_text())["rows"]


def table(title, rows, labels):
    lines = [f"### {title}", "",
             "| Method | RMSE m | Max m | Final m | " +
             " | ".join(f"drift {s} m %" for s in SEGMENTS) + " |",
             "|--------|------:|------:|------:|" + "------:|" * len(SEGMENTS)]
    for topic, label in labels:
        r = rows[topic]
        drift = r.get("segment_drift", {})
        cells = [f"{drift[s]['mean_pct']:.2f}" if s in drift else "-" for s in SEGMENTS]
        lines.append(f"| {label} | {r['rmse_m']:.2f} | {r['max_m']:.2f} | {r['final_m']:.2f} | "
                     + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--bag", required=True, type=Path, help="Converted beach_pond ROS 2 bag.")
    p.add_argument("--aux-bag", required=True, type=Path,
                   help="Aux bag with the DVL twist and GNSS heading.")
    p.add_argument("--out-dir", required=True, type=Path)
    p.add_argument("--start-offset-s", type=float, default=855.0,
                   help="Replay start (the DVL comes up ~853 s into the recording).")
    p.add_argument("--duration-s", type=float, default=0.0, help="0: to the end.")
    p.add_argument("--short-window-s", type=float, default=300.0,
                   help="Also score the first N seconds (no-compass drift grows quickly).")
    p.add_argument("--markdown", type=Path)
    args = p.parse_args(argv)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    scenarios = find_share("aqua_replay") / "scenarios"
    replay_bin = os.environ.get("AQUA_REPLAY_BIN", "")
    replay_cmd = [replay_bin] if replay_bin else ["ros2", "run", "aqua_replay", "offline_replay"]
    window = ["--duration", args.duration_s] if args.duration_s > 0 else []

    replays = {}
    for name, scenario in (("compass", "beach_pond_graph.yaml"),
                           ("no_compass", "beach_pond_graph_no_compass.yaml")):
        out = args.out_dir / f"graph_{name}"
        if out.exists():
            raise SystemExit(f"{out} exists; choose a fresh --out-dir")
        run(replay_cmd + ["--scenario", scenarios / scenario, "--bag", args.bag,
                          "--extra-bag", args.aux_bag, "--out", out, "--quiet", *window],
            stdout=subprocess.DEVNULL)
        replays[name] = out

    baselines = {}
    for yaw in ("heading", "gyro", "ahrs"):
        out = args.out_dir / f"dr_{yaw}"
        cmd = [sys.executable, SCRIPT_DIR / "dvl_dead_reckoning_baseline.py",
               "--bag", args.bag, "--bag", args.aux_bag, "--dst", out, "--overwrite",
               "--yaw-source", yaw, "--topic", f"/baseline/dvl_dr_{yaw}/odometry",
               "--start-offset-s", args.start_offset_s]
        if args.duration_s > 0:
            cmd += ["--duration-s", args.duration_s]
        run(cmd, stdout=subprocess.DEVNULL)
        baselines[yaw] = out

    text = (f"Commit `{git_commit()}`; replay fingerprints: compass "
            f"`{fingerprint(replays['compass'])}`, no compass "
            f"`{fingerprint(replays['no_compass'])}`.\n\n")
    compass = score([replays["compass"], baselines["heading"], baselines["ahrs"]],
                    ["/aqua_graph/odometry", "/baseline/dvl_dr_heading/odometry",
                     "/baseline/dvl_dr_ahrs/odometry"], args.out_dir / "compass.json")
    path_km = compass["/aqua_graph/odometry"]["path_m"] / 1000.0
    matched = compass["/aqua_graph/odometry"]["matched_s"]
    text += table(f"With GNSS compass heading ({matched:.0f} s, {path_km:.1f} km)", compass, [
        ("/aqua_graph/odometry", "aqua_graph (IMU + DVL + compass + depth hold)"),
        ("/baseline/dvl_dr_heading/odometry", "DVL DR, compass yaw"),
        ("/baseline/dvl_dr_ahrs/odometry", "DVL DR, Madgwick AHRS yaw"),
    ]) + "\n"
    labels = [("/aqua_graph/odometry", "aqua_graph (IMU + DVL + depth hold)"),
              ("/baseline/dvl_dr_gyro/odometry", "DVL DR, integrated gyro yaw")]
    bags = [replays["no_compass"], baselines["gyro"]]
    estimates = [t for t, _ in labels]
    short = score(bags, estimates, args.out_dir / "no_compass_short.json",
                  ["--duration-s", str(args.short_window_s)])
    text += table(f"Without compass, first {args.short_window_s:g} s "
                  f"({short[estimates[0]]['path_m']:.0f} m)", short, labels) + "\n"
    full = score(bags, estimates, args.out_dir / "no_compass_full.json")
    text += table(f"Without compass, full run ({matched:.0f} s)", full, labels)
    text = text.rstrip("\n") + "\n"
    print(text)
    if args.markdown:
        args.markdown.write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
