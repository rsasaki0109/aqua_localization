#!/usr/bin/env python3
"""Export the NTNU `subset-fjord/fjord_1` `aqua_localization` results-included
demo bag plus the dataset's own baseline TUM trajectory as a rerun.io
recording.

What you get:

  - `world/baseline`       — dataset baseline trajectory (green)
  - `world/aqua_imu_loc`   — `aqua_imu_loc` estimate (blue), Umeyama-aligned
                              to the baseline so both share an origin
  - `plots/depth/{baseline,estimate}` — z(t) over time
  - `plots/pressure`       — raw pressure (Pa)
  - `plots/imu/a{x,y,z}`   — IMU acceleration

Quick usage:

  ./aqua_localization/scripts/rerun_export_ntnu.py \\
    --bag aqua_localization/datasets/public/ntnu/demo_with_estimate \\
    --baseline-tum aqua_localization/datasets/public/ntnu/subset-fjord/fjord_1/fjord_1_baseline.tum \\
    --out docs/media/ntnu_fjord.rrd
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

try:
    import numpy as np
    import rerun as rr
    import rerun.blueprint as rrb
    from rosbags.highlevel import AnyReader
except ImportError as e:
    sys.stderr.write(f"missing dependency: {e}\n")
    raise


BASELINE_COLOR = (39, 200, 154)   # #27c89a
ESTIMATE_COLOR = (58, 161, 255)   # #3aa1ff


def umeyama_se3(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    cov = ((dst - dst_mean).T @ (src - src_mean)) / src.shape[0]
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt
    t = dst_mean - R @ src_mean
    return R, t


def interp_xyz(times_q: np.ndarray, times_ref: np.ndarray, xyz_ref: np.ndarray) -> np.ndarray:
    out = np.full((times_q.shape[0], 3), np.nan)
    if times_ref.size < 2:
        return out
    in_range = (times_q >= times_ref[0]) & (times_q <= times_ref[-1])
    for ax in range(3):
        out[in_range, ax] = np.interp(times_q[in_range], times_ref, xyz_ref[:, ax])
    return out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--baseline-tum", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--imu-topic", default="/mavros/imu/data")
    parser.add_argument("--pressure-topic", default="/mavros/imu/static_pressure")
    parser.add_argument("--estimate-topic", default="/aqua_imu_loc/odometry")
    parser.add_argument("--decimate-imu", type=int, default=20,
                        help="Plot every Nth IMU sample (default: 20)")
    parser.add_argument("--decimate-pressure", type=int, default=4,
                        help="Plot every Nth pressure sample (default: 4)")
    parser.add_argument("--application-id", default="aqua_localization NTNU fjord_1")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag_dir = args.bag if args.bag.is_dir() else args.bag.parent
    if not bag_dir.is_dir():
        sys.stderr.write(f"not a rosbag2 directory: {bag_dir}\n")
        return 1
    if not args.baseline_tum.is_file():
        sys.stderr.write(f"baseline TUM not found: {args.baseline_tum}\n")
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)

    baseline = np.loadtxt(args.baseline_tum)  # (N, 8): t, x, y, z, qx, qy, qz, qw
    base_t = baseline[:, 0]
    base_xyz = baseline[:, 1:4]
    print(f"baseline: {len(baseline)} samples, {base_t[-1] - base_t[0]:.1f} s, "
          f"bbox {base_xyz.min(axis=0)} → {base_xyz.max(axis=0)}")

    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="NTNU fjord_1 — dataset baseline trajectory",
                origin="/world",
                contents=[
                    "+ /world/**",
                    "- /world/aqua_imu_loc_xy_drifts/**",
                ],
                background=[15, 22, 32],
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="depth z (m): baseline vs aqua_imu_loc",
                    contents="/plots/depth/**",
                ),
                rrb.TimeSeriesView(
                    name="pressure (Pa)",
                    contents="/plots/pressure",
                ),
                rrb.TimeSeriesView(
                    name="IMU linear acceleration (m/s^2)",
                    contents="/plots/imu/**",
                ),
            ),
            column_shares=[3, 1],
        ),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
        rrb.BlueprintPanel(state="collapsed"),
    )

    rr.init(args.application_id, default_blueprint=blueprint)
    rr.save(str(args.out), default_blueprint=blueprint)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # Pass 1: collect estimate / pressure / imu samples.
    est_buf: list[tuple[float, np.ndarray]] = []
    press_buf: list[tuple[float, float]] = []
    imu_buf: list[tuple[float, np.ndarray]] = []

    def header_seconds(msg) -> float:
        s = msg.header.stamp
        return float(s.sec) + float(s.nanosec) * 1e-9

    targets = {args.estimate_topic, args.pressure_topic, args.imu_topic}
    with AnyReader([bag_dir]) as reader:
        wanted = [c for c in reader.connections if c.topic in targets]
        for connection, t_ns, raw in reader.messages(connections=wanted):
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception:
                continue
            # Use header.stamp instead of bag receive-time so we get the
            # source dataset's epoch (1700604781+). The recorder bag was
            # stamped with wall-clock receive time (1778…) which doesn't
            # overlap with the baseline TUM's epoch.
            t = header_seconds(msg)
            if connection.topic == args.estimate_topic:
                p = msg.pose.pose.position
                est_buf.append((t, np.array([p.x, p.y, p.z])))
            elif connection.topic == args.pressure_topic:
                press_buf.append((t, float(msg.fluid_pressure)))
            elif connection.topic == args.imu_topic:
                a = msg.linear_acceleration
                imu_buf.append((t, np.array([a.x, a.y, a.z])))

    if not est_buf:
        sys.stderr.write("no estimate samples in bag\n")
        return 2

    est_t = np.array([t for t, _ in est_buf])
    est_xyz = np.array([p for _, p in est_buf])

    # Both the bag and the baseline use ROS time stamps in the same epoch
    # (the bag is the converted ROS 2 version of the same sequence). Umeyama-
    # align estimate to baseline at overlapping timestamps.
    baseline_at_est = interp_xyz(est_t, base_t, base_xyz)
    valid = ~np.isnan(baseline_at_est).any(axis=1)
    if valid.sum() < 10:
        sys.stderr.write(
            f"only {int(valid.sum())} overlapping samples; baseline epoch "
            f"({base_t[0]:.1f}-{base_t[-1]:.1f}) vs estimate epoch "
            f"({est_t.min():.1f}-{est_t.max():.1f}). Re-record the bag with "
            f"original timestamps.\n"
        )
        return 3
    R_align, t_align = umeyama_se3(est_xyz[valid], baseline_at_est[valid])
    aligned_xyz = est_xyz @ R_align.T + t_align
    rmse = float(np.sqrt(np.mean(np.linalg.norm(
        aligned_xyz[valid] - baseline_at_est[valid], axis=1) ** 2)))
    print(f"Umeyama SE(3): aligned RMSE = {rmse:.3f} m on {int(valid.sum())} samples")

    t0 = min(est_t.min(), base_t.min())

    # Log baseline path as static. The aligned aqua_imu_loc XY estimate
    # drifts hundreds of meters on this bag (IMU-only dead reckoning, no
    # DVL/visual aiding) so we route it under a path that the default 3D
    # view filters out — depth still tracks well because the pressure
    # update closes the z loop.
    rr.log("world/baseline/path",
           rr.LineStrips3D([base_xyz], colors=BASELINE_COLOR, radii=0.06),
           static=True)
    rr.log("world/aqua_imu_loc_xy_drifts/path",
           rr.LineStrips3D([aligned_xyz], colors=ESTIMATE_COLOR, radii=0.04),
           static=True)

    # Plot streams: align time stamps to t0 and decimate where the rate is
    # very high (IMU at ~200 Hz, pressure at ~50 Hz).
    for i, (t, p) in enumerate(zip(base_t, base_xyz)):
        if i % 10 != 0:
            continue
        rr.set_time("bag_time", duration=t - t0)
        rr.log("plots/depth/baseline", rr.Scalars(float(p[2])))

    for i, (t, p) in enumerate(zip(est_t, aligned_xyz)):
        if i % 10 != 0:
            continue
        rr.set_time("bag_time", duration=t - t0)
        rr.log("plots/depth/estimate", rr.Scalars(float(p[2])))

    for i, (t, pp) in enumerate(press_buf):
        if i % args.decimate_pressure != 0:
            continue
        rr.set_time("bag_time", duration=t - t0)
        rr.log("plots/pressure", rr.Scalars(pp))

    for i, (t, a) in enumerate(imu_buf):
        if i % args.decimate_imu != 0:
            continue
        rr.set_time("bag_time", duration=t - t0)
        rr.log("plots/imu/ax", rr.Scalars(float(a[0])))
        rr.log("plots/imu/ay", rr.Scalars(float(a[1])))
        rr.log("plots/imu/az", rr.Scalars(float(a[2])))

    print(f"wrote {args.out} ({args.out.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
