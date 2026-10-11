#!/usr/bin/env python3
"""aqua_graph stereo VIO on NTNU underwater-datasets fjord sequences.

For each sequence: stream the ROS 1 bag from the dataset repository straight
into convert_ntnu_stereo_bag.py (no local raw copy, so 20+ GB bags fit), run
the ntnu_fjord_vio scenario deterministically, and score against the
sequence's ReAqROVIO reference (`<seq>_baseline.tum`, four cameras + IMU).
Converted bags can be deleted after scoring with --cleanup.

    run_ntnu_benchmark.py --calib-dir calib --work-dir /data/ntnu \\
        --sequences fjord_1 fjord_2 --cleanup --markdown rows.md
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
BASE_URL = "https://huggingface.co/datasets/ntnu-arl/underwater-datasets/resolve/main"
SEGMENTS = ("10", "50", "100")


def run(cmd, **kwargs):
    print("+ " + " ".join(str(c) for c in cmd), file=sys.stderr)
    return subprocess.run([str(c) for c in cmd], check=True, **kwargs)


def fingerprint(replay: Path) -> str:
    for line in (replay / "replay_summary.yaml").read_text().splitlines():
        if line.strip().startswith("fingerprint:"):
            return line.split(":", 1)[1].strip().strip('"')
    return "unknown"


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--calib-dir", required=True, type=Path,
                   help="Holds camchain-stereo-intrinsics-underwater.yaml and "
                        "camchain-imucam-stereo-extrinsics-air.yaml (cam0_cam1_stereo).")
    p.add_argument("--work-dir", required=True, type=Path)
    p.add_argument("--sequences", nargs="+", default=[f"fjord_{i}" for i in range(1, 7)])
    p.add_argument("--subset", default="subset-fjord")
    p.add_argument("--scenario", type=Path)
    p.add_argument("--cleanup", action="store_true",
                   help="Delete each converted bag and replay output after scoring.")
    p.add_argument("--markdown", type=Path)
    args = p.parse_args(argv)

    scenario = args.scenario
    if scenario is None:
        prefix = subprocess.run(["ros2", "pkg", "prefix", "aqua_replay"], check=True,
                                capture_output=True, text=True).stdout.strip()
        scenario = Path(prefix) / "share" / "aqua_replay" / "scenarios" / "ntnu_fjord_vio.yaml"
    replay_bin = os.environ.get("AQUA_REPLAY_BIN", "")
    replay_cmd = [replay_bin] if replay_bin else ["ros2", "run", "aqua_replay", "offline_replay"]
    args.work_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for seq in args.sequences:
        seq_dir = args.work_dir / seq
        seq_dir.mkdir(parents=True, exist_ok=True)
        reference = seq_dir / f"{seq}_baseline.tum"
        if not reference.exists():
            run(["curl", "-sSL", "--fail", "-o", reference,
                 f"{BASE_URL}/{args.subset}/{seq}/{seq}_baseline.tum"])
        converted = seq_dir / "stereo_ros2"
        if not converted.exists():
            run(["ros2", "run", "aqua_localization", "convert_ntnu_stereo_bag.py",
                 "--src-url", f"{BASE_URL}/{args.subset}/{seq}/{seq}.bag",
                 "--reference", reference,
                 "--intrinsics", args.calib_dir / "camchain-stereo-intrinsics-underwater.yaml",
                 "--imu-extrinsics", args.calib_dir / "camchain-imucam-stereo-extrinsics-air.yaml",
                 "--dst", converted], stdout=subprocess.DEVNULL)
        replay = seq_dir / "replay"
        if replay.exists():
            raise SystemExit(f"{replay} exists; remove it or use a fresh --work-dir")
        run(replay_cmd + ["--scenario", scenario, "--bag", converted, "--out", replay,
                          "--quiet"], stdout=subprocess.DEVNULL)
        score_json = seq_dir / "ape.json"
        run([sys.executable, SCRIPT_DIR / "evaluate_replay_trajectories.py", "--bag", replay,
             "--reference", "/reference/odometry", "--estimate", "/aqua_graph/odometry",
             "--segment-lengths", *SEGMENTS, "--json", score_json], stdout=subprocess.DEVNULL)
        row = json.loads(score_json.read_text())["rows"]["/aqua_graph/odometry"]
        row.update(sequence=seq, fingerprint=fingerprint(replay))
        rows.append(row)
        print(f"{seq}: RMSE {row['rmse_m']:.3f} m over {row['path_m']:.0f} m", file=sys.stderr)
        if args.cleanup:
            run(["rm", "-rf", "--", converted, replay])

    lines = ["| Sequence | Path m | Duration s | RMSE m | Max m | Final m | "
             + " | ".join(f"drift {s} m %" for s in SEGMENTS) + " | Fingerprint |",
             "|----------|------:|------:|------:|------:|------:|"
             + "------:|" * len(SEGMENTS) + "------|"]
    for r in rows:
        drift = r.get("segment_drift", {})
        cells = [f"{drift[s]['mean_pct']:.2f}" if s in drift else "-" for s in SEGMENTS]
        lines.append(f"| `{r['sequence']}` | {r['path_m']:.0f} | {r['matched_s']:.0f} | "
                     f"{r['rmse_m']:.3f} | {r['max_m']:.3f} | {r['final_m']:.3f} | "
                     + " | ".join(cells) + f" | `{r['fingerprint']}` |")
    text = "\n".join(lines) + "\n"
    print(text)
    if args.markdown:
        args.markdown.write_text(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
