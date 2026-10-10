#!/usr/bin/env python3
"""Score estimator trajectories in an offline_replay output bag against a reference.

Reads nav_msgs/Odometry topics straight from the bag, interpolates each
estimate onto the reference stamps over their common time window, applies a
closed-form rigid SE(3) (or Sim(3) with --scale) Umeyama alignment, and
prints translation APE statistics as a markdown table. Optionally writes the
rows as JSON and each trajectory as TUM for other tools.

    evaluate_replay_trajectories.py --bag /tmp/tank_dvl_replay \
        --reference /apriltag_slam/GT \
        --estimate /aqua_imu_loc/odometry --estimate /aqua_graph/odometry
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def load_odometry(bag: Path, topic: str) -> np.ndarray:
    """Return an (N, 8) TUM array [t, x, y, z, qx, qy, qz, qw] for an Odometry topic."""
    from rosbags.highlevel import AnyReader

    rows = []
    with AnyReader([Path(bag)]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            raise SystemExit(f"topic {topic} not found in {bag}")
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    data = np.array(rows, dtype=np.float64)
    if data.size == 0:
        raise SystemExit(f"topic {topic} in {bag} has no messages")
    order = np.argsort(data[:, 0], kind="stable")
    return data[order]


def umeyama(source: np.ndarray, target: np.ndarray, with_scale: bool):
    """Return (scale, R, t) minimizing ||target - (s R source + t)||."""
    mu_s, mu_t = source.mean(0), target.mean(0)
    xs, xt = source - mu_s, target - mu_t
    u, d, vt = np.linalg.svd(xt.T @ xs / source.shape[0])
    sign = np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))])
    rotation = u @ sign @ vt
    scale = float(np.trace(np.diag(d) @ sign) / xs.var(0).sum()) if with_scale else 1.0
    return scale, rotation, mu_t - scale * rotation @ mu_s


def ape(reference: np.ndarray, estimate: np.ndarray, with_scale: bool = False) -> dict:
    t0 = max(reference[0, 0], estimate[0, 0])
    t1 = min(reference[-1, 0], estimate[-1, 0])
    ref = reference[(reference[:, 0] >= t0) & (reference[:, 0] <= t1)]
    if ref.shape[0] < 3:
        return {"samples": int(ref.shape[0]), "error": "insufficient overlap"}
    est = np.stack([np.interp(ref[:, 0], estimate[:, 0], estimate[:, k]) for k in (1, 2, 3)], 1)
    gt = ref[:, 1:4]
    scale, rotation, translation = umeyama(est, gt, with_scale)
    aligned = (scale * (rotation @ est.T)).T + translation
    err = np.linalg.norm(aligned - gt, axis=1)
    return {
        "samples": int(err.size),
        "matched_s": float(ref[-1, 0] - ref[0, 0]),
        "rmse_m": float(np.sqrt(np.mean(err ** 2))),
        "mean_m": float(err.mean()),
        "median_m": float(np.median(err)),
        "max_m": float(err.max()),
        "final_m": float(err[-1]),
        "scale": scale,
    }


def write_tum(path: Path, data: np.ndarray) -> None:
    np.savetxt(path, data, fmt="%.9f", header="timestamp tx ty tz qx qy qz qw")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bag", required=True, type=Path, help="offline_replay output bag.")
    parser.add_argument("--reference", required=True, help="Reference nav_msgs/Odometry topic.")
    parser.add_argument("--estimate", action="append", required=True,
                        help="Estimate nav_msgs/Odometry topic (repeatable).")
    parser.add_argument("--scale", action="store_true", help="Sim(3) instead of SE(3) alignment.")
    parser.add_argument("--json", type=Path, help="Write the result rows as JSON.")
    parser.add_argument("--tum-dir", type=Path, help="Also export every trajectory as TUM here.")
    args = parser.parse_args(argv)

    reference = load_odometry(args.bag, args.reference)
    rows = {}
    trajectories = {args.reference: reference}
    for topic in args.estimate:
        estimate = load_odometry(args.bag, topic)
        trajectories[topic] = estimate
        rows[topic] = ape(reference, estimate, args.scale)

    align = "Sim(3)" if args.scale else "SE(3)"
    print(f"Reference: {args.reference} ({reference.shape[0]} samples), alignment {align}\n")
    print("| Estimate | Samples | Matched s | RMSE m | Mean m | Median m | Max m | Final m |")
    print("|----------|--------:|----------:|-------:|-------:|---------:|------:|--------:|")
    for topic, r in rows.items():
        if "error" in r:
            print(f"| `{topic}` | {r['samples']} | - | {r['error']} | | | | |")
            continue
        print(f"| `{topic}` | {r['samples']} | {r['matched_s']:.2f} | {r['rmse_m']:.4f} | "
              f"{r['mean_m']:.4f} | {r['median_m']:.4f} | {r['max_m']:.4f} | {r['final_m']:.4f} |")

    if args.json:
        args.json.write_text(json.dumps(
            {"bag": str(args.bag), "reference": args.reference, "alignment": align,
             "rows": rows}, indent=2) + "\n")
    if args.tum_dir:
        args.tum_dir.mkdir(parents=True, exist_ok=True)
        for topic, data in trajectories.items():
            write_tum(args.tum_dir / (topic.strip("/").replace("/", "_") + ".tum"), data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
