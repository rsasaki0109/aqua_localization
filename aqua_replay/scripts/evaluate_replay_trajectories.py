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


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate v by unit quaternions q = [qx, qy, qz, qw] (both (N, ...))."""
    u, w = q[:, :3], q[:, 3:4]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def apply_body_offset(data: np.ndarray, offset) -> np.ndarray:
    """Move a TUM trajectory to a point fixed in its body frame."""
    out = data.copy()
    v = np.broadcast_to(np.asarray(offset, dtype=np.float64), (data.shape[0], 3))
    out[:, 1:4] += quat_rotate(data[:, 4:8], v)
    return out


def load_odometry(bags, topic: str) -> np.ndarray:
    """Return an (N, 8) TUM array [t, x, y, z, qx, qy, qz, qw] for an Odometry topic."""
    from rosbags.highlevel import AnyReader

    bag = bags if isinstance(bags, (list, tuple)) else [bags]
    rows = []
    with AnyReader([Path(b) for b in bag]) as reader:
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


def segment_drift(gt: np.ndarray, aligned: np.ndarray, lengths) -> dict:
    """KITTI-style relative translation error in the aligned world frame.

    For every reference sample (start) and segment length L, take the first
    sample at least L metres of reference path later and compare the
    displacement vectors. Orientation is not used, so estimate and reference
    may use different body frames. Returns {L: drift in % of L}.
    """
    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(gt, axis=0), axis=1))])
    step = max(1, gt.shape[0] // 2000)
    out = {}
    for length in lengths:
        starts = np.arange(0, gt.shape[0], step)
        ends = np.searchsorted(dist, dist[starts] + length)
        ok = ends < gt.shape[0]
        if not ok.any():
            continue
        s, e = starts[ok], ends[ok]
        err = np.linalg.norm((aligned[e] - aligned[s]) - (gt[e] - gt[s]), axis=1)
        out[f"{length:g}"] = {"segments": int(ok.sum()),
                              "mean_pct": float(100.0 * np.mean(err / (dist[e] - dist[s]))),
                              "max_m": float(err.max())}
    return out


def ape(reference: np.ndarray, estimate: np.ndarray, with_scale: bool = False,
        window_start: float | None = None, window_end: float | None = None,
        segment_lengths=()) -> dict:
    t0 = max(reference[0, 0], estimate[0, 0])
    t1 = min(reference[-1, 0], estimate[-1, 0])
    if window_start is not None:
        t0 = max(t0, window_start)
    if window_end is not None:
        t1 = min(t1, window_end)
    ref = reference[(reference[:, 0] >= t0) & (reference[:, 0] <= t1)]
    if ref.shape[0] < 3:
        return {"samples": int(ref.shape[0]), "error": "insufficient overlap"}
    est = np.stack([np.interp(ref[:, 0], estimate[:, 0], estimate[:, k]) for k in (1, 2, 3)], 1)
    gt = ref[:, 1:4]
    scale, rotation, translation = umeyama(est, gt, with_scale)
    aligned = (scale * (rotation @ est.T)).T + translation
    err = np.linalg.norm(aligned - gt, axis=1)
    path = float(np.linalg.norm(np.diff(gt, axis=0), axis=1).sum())
    result = {
        "samples": int(err.size),
        "matched_s": float(ref[-1, 0] - ref[0, 0]),
        "rmse_m": float(np.sqrt(np.mean(err ** 2))),
        "mean_m": float(err.mean()),
        "median_m": float(np.median(err)),
        "max_m": float(err.max()),
        "final_m": float(err[-1]),
        "scale": scale,
        "path_m": path,
    }
    if segment_lengths:
        result["segment_drift"] = segment_drift(gt, aligned, segment_lengths)
    return result


def write_tum(path: Path, data: np.ndarray) -> None:
    np.savetxt(path, data, fmt="%.9f", header="timestamp tx ty tz qx qy qz qw")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bag", required=True, type=Path, action="append",
                        help="offline_replay output bag (repeatable: topics are looked up "
                             "across all of them).")
    parser.add_argument("--reference", required=True, help="Reference nav_msgs/Odometry topic.")
    parser.add_argument("--estimate", action="append", required=True,
                        help="Estimate nav_msgs/Odometry topic (repeatable).")
    parser.add_argument("--scale", action="store_true", help="Sim(3) instead of SE(3) alignment.")
    parser.add_argument("--json", type=Path, help="Write the result rows as JSON.")
    window = parser.add_mutually_exclusive_group()
    window.add_argument("--skip-first-s", type=float,
                        help="Only score reference samples at least this long after the "
                             "reference's first stamp.")
    window.add_argument("--last-s", type=float,
                        help="Only score the final N seconds of the reference (e.g. to match "
                             "a baseline that initializes late).")
    parser.add_argument("--body-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="Score the estimates at this point of their body frame (e.g. the "
                             "reference's base_link origin seen from an IMU-frame estimate).")
    parser.add_argument("--segment-lengths", type=float, nargs="+", default=[],
                        metavar="M", help="Also report relative drift over these path lengths.")
    parser.add_argument("--duration-s", type=float,
                        help="Only score this many seconds from the window start (the "
                             "reference's first stamp unless --skip-first-s/--last-s).")
    parser.add_argument("--tum-dir", type=Path, help="Also export every trajectory as TUM here.")
    args = parser.parse_args(argv)

    reference = load_odometry(args.bag, args.reference)
    window_start = None
    if args.skip_first_s is not None:
        window_start = reference[0, 0] + args.skip_first_s
    elif args.last_s is not None:
        window_start = reference[-1, 0] - args.last_s
    window_end = None
    if args.duration_s is not None:
        window_end = (reference[0, 0] if window_start is None else window_start) + args.duration_s
    rows = {}
    trajectories = {args.reference: reference}
    for topic in args.estimate:
        estimate = load_odometry(args.bag, topic)
        if args.body_offset:
            estimate = apply_body_offset(estimate, args.body_offset)
        trajectories[topic] = estimate
        rows[topic] = ape(reference, estimate, args.scale, window_start, window_end,
                          segment_lengths=args.segment_lengths)

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
    if args.segment_lengths:
        print("\nRelative drift (% of segment length, mean over segments):\n")
        lengths = [f"{x:g}" for x in args.segment_lengths]
        print("| Estimate | " + " | ".join(f"{x} m" for x in lengths) + " |")
        print("|----------|" + "------:|" * len(lengths))
        for topic, r in rows.items():
            drift = r.get("segment_drift", {})
            cells = [f"{drift[x]['mean_pct']:.2f}" if x in drift else "-" for x in lengths]
            print(f"| `{topic}` | " + " | ".join(cells) + " |")

    if args.json:
        args.json.write_text(json.dumps(
            {"bag": [str(b) for b in args.bag], "reference": args.reference, "alignment": align,
             "rows": rows}, indent=2) + "\n")
    if args.tum_dir:
        args.tum_dir.mkdir(parents=True, exist_ok=True)
        for topic, data in trajectories.items():
            write_tum(args.tum_dir / (topic.strip("/").replace("/", "_") + ".tum"), data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
