#!/usr/bin/env python3
"""Export an `aqua_localization` results-included demo bag as a rerun.io
recording (`.rrd`) plus a static screenshot.

rerun.io is a Python-native, web-viewer-capable alternative to Foxglove
Studio that does not require an account or a browser-side data source.
The exported `.rrd` can be opened in the standalone rerun viewer
(`rerun some.rrd`) or served via the rerun web viewer.

This script focuses on the Tank Dataset short_test demo (IMU + pressure +
DVL fusion vs AprilTag GT). It logs:

  - `world/gt`        — AprilTag GT path (green)
  - `world/estimate`  — `aqua_imu_loc` estimate path (blue)
  - `world/dvl`       — DVL body-frame velocity arrow at base_link
  - `plots/depth/{gt,estimate}` — depth time series
  - `plots/dvl/{vx,vy,vz}`      — DVL body-frame velocity time series

Quick usage:

  pip install --user rerun-sdk rosbags
  ./aqua_localization/scripts/rerun_export.py \\
    --bag aqua_localization/datasets/public/tank_dataset/demo_with_estimate \\
    --out docs/media/tank_dataset.rrd
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

try:
    import numpy as np
    import rerun as rr
    from rosbags.highlevel import AnyReader
except ImportError as e:
    sys.stderr.write(f"missing dependency: {e}. Install rerun-sdk + rosbags.\n")
    raise


GT_COLOR = (39, 200, 154)        # #27c89a
EST_COLOR = (58, 161, 255)       # #3aa1ff
DVL_COLOR = (245, 166, 35)       # #f5a623


def umeyama_se3(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid (no-scale) Umeyama alignment that maps src onto dst.

    src, dst: (N, 3) arrays. Returns (R, t) so that dst ~= R @ src.T + t."""
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


def interp_at(times_query: np.ndarray, times_ref: np.ndarray, xyz_ref: np.ndarray) -> np.ndarray:
    """Linearly interpolate xyz_ref(times_ref) at times_query. NaN outside range."""
    out = np.full((times_query.shape[0], 3), np.nan)
    if times_ref.size < 2:
        return out
    in_range = (times_query >= times_ref[0]) & (times_query <= times_ref[-1])
    for ax in range(3):
        out[in_range, ax] = np.interp(times_query[in_range], times_ref, xyz_ref[:, ax])
    return out


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Hamiltonian quaternion (xyzw) to 3x3 rotation matrix."""
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = qx * qx * s, qy * qy * s, qz * qz * s
    xy, xz, yz = qx * qy * s, qx * qz * s, qy * qz * s
    wx, wy, wz = qw * qx * s, qw * qy * s, qw * qz * s
    return np.array([
        [1 - (yy + zz),     xy - wz,         xz + wy],
        [xy + wz,           1 - (xx + zz),   yz - wx],
        [xz - wy,           yz + wx,         1 - (xx + yy)],
    ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True,
                        help="rosbag2 directory or .mcap file")
    parser.add_argument("--out", type=Path, required=True,
                        help="Output .rrd recording")
    parser.add_argument("--screenshot", type=Path, default=None,
                        help="Optional final-frame PNG snapshot")
    parser.add_argument(
        "--gt-topic", default="/apriltag_slam/GT",
        help="Reference (ground-truth) odometry topic name")
    parser.add_argument(
        "--estimate-topic", default="/aqua_imu_loc/odometry",
        help="aqua_localization estimate odometry topic name")
    parser.add_argument(
        "--dvl-topic", default="/dvl/twist",
        help="DVL body-frame twist topic name (TwistStamped)")
    parser.add_argument(
        "--imu-topic", default="/imu/data",
        help="IMU topic name (sensor_msgs/Imu)")
    parser.add_argument(
        "--pressure-topic", default="/pressure",
        help="Pressure topic name (sensor_msgs/FluidPressure)")
    parser.add_argument(
        "--application-id", default="aqua_localization tank short_test",
        help="rerun application id displayed in the viewer")
    return parser.parse_args()


def odometry_position(msg) -> np.ndarray:
    p = msg.pose.pose.position
    return np.array([p.x, p.y, p.z], dtype=np.float64)


def odometry_quat(msg) -> tuple[float, float, float, float]:
    q = msg.pose.pose.orientation
    return q.x, q.y, q.z, q.w


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main() -> int:
    args = parse_args()

    # Locate the rosbag2 directory (rosbags' AnyReader expects it).
    if args.bag.is_file() and args.bag.suffix == ".mcap":
        bag_dir = args.bag.parent
    else:
        bag_dir = args.bag
    if not bag_dir.is_dir():
        sys.stderr.write(f"not a rosbag2 directory: {bag_dir}\n")
        return 1

    args.out.parent.mkdir(parents=True, exist_ok=True)

    import rerun.blueprint as rrb
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="World view (GT vs aqua_imu_loc)",
                origin="/world",
                background=[20, 26, 34],
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="Depth z (m): GT vs aqua_imu_loc",
                    contents="/plots/depth/**",
                ),
                rrb.TimeSeriesView(
                    name="DVL body-frame velocity (m/s)",
                    contents="/plots/dvl/**",
                ),
                rrb.TimeSeriesView(
                    name="IMU linear acceleration (m/s^2)",
                    contents="/plots/imu/**",
                ),
            ),
            column_shares=[2, 1],
        ),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
        rrb.BlueprintPanel(state="collapsed"),
    )

    rr.init(args.application_id, default_blueprint=blueprint)
    rr.save(str(args.out), default_blueprint=blueprint)

    # Static layout / metadata. Z-up REP-103 frame.
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # Pass 1: buffer messages and collect timestamps + positions for alignment.
    gt_buf: list[tuple[float, np.ndarray]] = []
    est_buf: list[tuple[float, np.ndarray, tuple[float, float, float, float]]] = []
    dvl_buf: list[tuple[float, np.ndarray]] = []
    pressure_buf: list[tuple[float, float]] = []
    imu_buf: list[tuple[float, np.ndarray]] = []

    with AnyReader([bag_dir]) as reader:
        targets = [
            args.gt_topic, args.estimate_topic, args.dvl_topic,
            args.imu_topic, args.pressure_topic,
        ]
        wanted = [c for c in reader.connections if c.topic in targets]
        if not wanted:
            sys.stderr.write("no matching topics found in bag\n")
            return 2

        for connection, timestamp_ns, raw in reader.messages(connections=wanted):
            t = timestamp_ns * 1e-9
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception:
                continue

            if connection.topic == args.gt_topic:
                gt_buf.append((t, odometry_position(msg)))
            elif connection.topic == args.estimate_topic:
                est_buf.append((t, odometry_position(msg), odometry_quat(msg)))
            elif connection.topic == args.dvl_topic:
                v = msg.twist.linear
                dvl_buf.append((t, np.array([v.x, v.y, v.z])))
            elif connection.topic == args.pressure_topic:
                pressure_buf.append((t, float(msg.fluid_pressure)))
            elif connection.topic == args.imu_topic:
                a = msg.linear_acceleration
                imu_buf.append((t, np.array([a.x, a.y, a.z])))

    if not gt_buf or not est_buf:
        sys.stderr.write("missing GT or estimate samples; cannot align\n")
        return 3

    # Compute Umeyama SE(3) alignment of estimate onto GT, then apply.
    gt_t = np.array([t for t, _ in gt_buf])
    gt_xyz = np.array([p for _, p in gt_buf])
    est_t = np.array([t for t, _, _ in est_buf])
    est_xyz = np.array([p for _, p, _ in est_buf])
    gt_at_est = interp_at(est_t, gt_t, gt_xyz)
    valid = ~np.isnan(gt_at_est).any(axis=1)
    if valid.sum() < 10:
        sys.stderr.write("not enough overlapping GT/est samples for Umeyama alignment\n")
        return 4
    R_align, t_align = umeyama_se3(est_xyz[valid], gt_at_est[valid])
    print(f"Umeyama SE(3) alignment: |t|={np.linalg.norm(t_align):.3f} m")

    aligned_est_xyz = (est_xyz @ R_align.T) + t_align
    rmse = float(np.sqrt(np.mean(np.linalg.norm(
        aligned_est_xyz[valid] - gt_at_est[valid], axis=1) ** 2)))
    print(f"APE translation RMSE: {rmse:.3f} m on {int(valid.sum())} samples")

    # Pass 2: log everything in chronological order, with estimate aligned.
    t0 = min(
        gt_buf[0][0] if gt_buf else float("inf"),
        est_buf[0][0] if est_buf else float("inf"),
        dvl_buf[0][0] if dvl_buf else float("inf"),
        pressure_buf[0][0] if pressure_buf else float("inf"),
        imu_buf[0][0] if imu_buf else float("inf"),
    )

    events: list[tuple[float, str, int]] = []
    for i, (t, _) in enumerate(gt_buf):
        events.append((t, "gt", i))
    for i, _ in enumerate(est_buf):
        events.append((est_t[i], "est", i))
    for i, (t, _) in enumerate(dvl_buf):
        events.append((t, "dvl", i))
    for i, (t, _) in enumerate(pressure_buf):
        events.append((t, "pressure", i))
    for i, (t, _) in enumerate(imu_buf):
        events.append((t, "imu", i))
    events.sort(key=lambda x: x[0])

    gt_path: list[np.ndarray] = []
    est_path: list[np.ndarray] = []

    for t, kind, i in events:
        rr.set_time("bag_time", duration=t - t0)

        if kind == "gt":
            p = gt_buf[i][1]
            gt_path.append(p)
            rr.log("world/gt/pose", rr.Points3D([p], colors=GT_COLOR, radii=0.02))
            rr.log("world/gt/path",
                   rr.LineStrips3D([np.asarray(gt_path)], colors=GT_COLOR, radii=0.005))
            rr.log("plots/depth/gt", rr.Scalars(float(p[2])))

        elif kind == "est":
            p = aligned_est_xyz[i]
            qx, qy, qz, qw = est_buf[i][2]
            est_path.append(p)
            rr.log("world/estimate/pose",
                   rr.Points3D([p], colors=EST_COLOR, radii=0.02))
            rr.log("world/estimate/path",
                   rr.LineStrips3D([np.asarray(est_path)],
                                   colors=EST_COLOR, radii=0.005))
            rr.log("plots/depth/estimate", rr.Scalars(float(p[2])))

            # Compose body orientation in aligned frame: R_align @ R_body.
            Rb = quat_to_rot_matrix(qx, qy, qz, qw)
            R_body_aligned = R_align @ Rb
            rr.log("world/base_link",
                   rr.Transform3D(translation=p, mat3x3=R_body_aligned))

        elif kind == "dvl":
            v = dvl_buf[i][1]
            rr.log("world/base_link/dvl",
                   rr.Arrows3D(
                       origins=[[0, 0, 0]],
                       vectors=[v.tolist()],
                       colors=[DVL_COLOR],
                   ))
            rr.log("plots/dvl/vx", rr.Scalars(float(v[0])))
            rr.log("plots/dvl/vy", rr.Scalars(float(v[1])))
            rr.log("plots/dvl/vz", rr.Scalars(float(v[2])))

        elif kind == "pressure":
            rr.log("plots/pressure", rr.Scalars(float(pressure_buf[i][1])))

        elif kind == "imu":
            a = imu_buf[i][1]
            rr.log("plots/imu/ax", rr.Scalars(float(a[0])))
            rr.log("plots/imu/ay", rr.Scalars(float(a[1])))
            rr.log("plots/imu/az", rr.Scalars(float(a[2])))

    print(f"wrote {args.out}")
    print(f"  GT samples       : {len(gt_buf)}")
    print(f"  estimate samples : {len(est_buf)} ({int(valid.sum())} aligned)")
    print(f"  DVL samples      : {len(dvl_buf)}")
    print(f"  pressure samples : {len(pressure_buf)}")
    print(f"  IMU samples      : {len(imu_buf)}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
