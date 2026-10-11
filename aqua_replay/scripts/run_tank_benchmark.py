#!/usr/bin/env python3
"""Reproduce the Tank Dataset benchmark rows for aqua_graph and the UKF.

One command from a converted ROS 2 bag to markdown rows in the format of
docs/benchmarks/tank_aqua_slam.md (which the AQUA-SLAM head-to-head and
claim-readiness reports parse):

1. track stereo features offline (make_stereo_feature_track.py);
2. replay the bag + features deterministically through the UKF Tank profile
   and aqua_graph (tank_short_test_vio scenario) with offline_replay;
3. score SE(3)-aligned APE against the AprilTag reference on the full
   overlap and on a trailing window matching a late-initializing baseline.

    run_tank_benchmark.py --bag short_test_ros2 --out-dir /tmp/tank_bench

Rows carry the git commit, the replay fingerprint, and the exact feature
settings, so a table entry can be traced to the run that produced it.
"""

import argparse
import json
import os
import subprocess
import sys
from datetime import date
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


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


def score(bag: Path, reference: str, estimates, extra=()) -> dict:
    out = bag.parent / (bag.name + "_ape" + ("_".join(extra).replace("-", "") or "") + ".json")
    cmd = [sys.executable, SCRIPT_DIR / "evaluate_replay_trajectories.py", "--bag", bag,
           "--reference", reference, "--json", out, *extra]
    for topic in estimates:
        cmd += ["--estimate", topic]
    run(cmd, stdout=subprocess.DEVNULL)
    return json.loads(out.read_text())["rows"]


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--bag", required=True, type=Path, help="Converted Tank ROS 2 bag.")
    p.add_argument("--out-dir", required=True, type=Path)
    p.add_argument("--sequence", default="short_test")
    p.add_argument("--scenario", type=Path,
                   help="Replay scenario (default: installed tank_short_test_vio.yaml).")
    p.add_argument("--reference", default="/apriltag_slam/GT")
    p.add_argument("--baseline-window-s", type=float, default=11.65,
                   help="Trailing window to also score (AQUA-SLAM's matched span on "
                        "short_test).")
    p.add_argument("--max-features", type=int, default=100)
    p.add_argument("--publish-every", type=int, default=2)
    p.add_argument("--markdown", type=Path, help="Also write the rows to this file.")
    args = p.parse_args(argv)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    features = args.out_dir / "features"
    replay = args.out_dir / "replay"
    if replay.exists():
        raise SystemExit(f"{replay} exists; choose a fresh --out-dir")
    scenario = args.scenario or find_share("aqua_replay") / "scenarios" / "tank_short_test_vio.yaml"

    run([sys.executable, SCRIPT_DIR / "make_stereo_feature_track.py", "--bag", args.bag,
         "--dst", features, "--overwrite", "--max-features", args.max_features,
         "--publish-every", args.publish_every], stdout=subprocess.DEVNULL)
    replay_bin = os.environ.get("AQUA_REPLAY_BIN", "")
    replay_cmd = [replay_bin] if replay_bin else ["ros2", "run", "aqua_replay", "offline_replay"]
    run(replay_cmd + ["--scenario", scenario, "--bag", args.bag, "--extra-bag", features,
                      "--out", replay, "--quiet"], stdout=subprocess.DEVNULL)

    import yaml  # noqa: E402 (ROS python env)
    summary = yaml.safe_load((replay / "replay_summary.yaml").read_text())
    estimates = {
        "/aqua_graph/odometry": ("aqua_graph", "stereo + IMU + DVL + pressure"),
        "/aqua_imu_loc/odometry": ("aqua_localization_ukf", "IMU + DVL + pressure"),
    }
    full = score(replay, args.reference, estimates)
    window = score(replay, args.reference, estimates,
                   ("--last-s", f"{args.baseline_window_s}"))

    commit = git_commit()
    lines = [
        "| Dataset | Sequence | System | Inputs | Alignment | Samples | Matched s | Mean m | "
        "Median m | RMSE m | Max m | Note |",
        "|---------|----------|--------|--------|-----------|--------:|----------:|-------:|"
        "---------:|-------:|------:|------|",
    ]
    for rows, label in ((window, f"last {args.baseline_window_s:g} s"), (full, "full overlap")):
        for topic, (system, inputs) in estimates.items():
            r = rows[topic]
            note = (f"{label}; offline_replay fingerprint {summary['fingerprint']}; commit "
                    f"{commit}; features max {args.max_features} every {args.publish_every}; "
                    f"{date.today().isoformat()}")
            lines.append(
                f"| Tank Dataset | `{args.sequence}` | {system} | {inputs} | SE(3) | "
                f"{r['samples']} | {r['matched_s']:.2f} | {r['mean_m']:.4f} | "
                f"{r['median_m']:.4f} | {r['rmse_m']:.4f} | {r['max_m']:.4f} | {note} |")
    text = "\n".join(lines) + "\n"
    print(text)
    if args.markdown:
        args.markdown.write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
