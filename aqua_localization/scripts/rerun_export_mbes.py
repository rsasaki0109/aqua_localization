#!/usr/bin/env python3
"""Export an MBES-SLAM `aqua_localization` results-included demo bag as a
rerun.io recording.

What you get in the recording:

  - `world/reference`    — reference odometry path (green)
  - `world/imu_estimate` — `aqua_imu_loc` path (orange)
  - `world/sonar_estimate` — `aqua_sonar_loc` path (blue)
  - `world/pose_graph` — optimised pose-graph path and accepted loop edges
  - `world/sonar_fans/<i>` — accumulated multibeam fans transformed into world
                              coordinates by the bag's reference pose at the
                              fan timestamp. Color-coded by depth.
  - `plots/sonar/fitness`, `plots/sonar/inliers` — registration diagnostics
  - `plots/loop_closure/*` — MBES loop closure accept/reject diagnostics
  - `plots/depth/{reference,imu,sonar}` — z(t) traces

Quick usage:

  ./aqua_localization/scripts/rerun_export_mbes.py \\
    --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \\
    --out docs/media/mbes_slam.rrd
"""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import sys

try:
    import numpy as np
    import rerun as rr
    import rerun.blueprint as rrb
    from rosbags.highlevel import AnyReader
except ImportError as e:
    sys.stderr.write(f"missing dependency: {e}\n")
    raise


REF_COLOR = (39, 200, 154)      # #27c89a green
IMU_COLOR = (245, 166, 35)      # #f5a623 orange
SONAR_COLOR = (58, 161, 255)    # #3aa1ff blue
POSE_GRAPH_COLOR = (247, 216, 80)  # #f7d850 yellow
LOOP_COLOR = (255, 82, 119)        # #ff5277 pink


def umeyama_se3(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid SE(3) Umeyama: returns (R, t) so that dst ≈ R @ src.T + t."""
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
    """Linearly interpolate xyz_ref(times_ref) at times_q. NaN outside range."""
    out = np.full((times_q.shape[0], 3), np.nan)
    if times_ref.size < 2:
        return out
    in_range = (times_q >= times_ref[0]) & (times_q <= times_ref[-1])
    for ax in range(3):
        out[in_range, ax] = np.interp(times_q[in_range], times_ref, xyz_ref[:, ax])
    return out


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
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
    parser.add_argument("--reference-topic", default="/nav/processed/odometry")
    parser.add_argument("--imu-topic", default="/aqua_imu_loc/odometry")
    parser.add_argument("--sonar-odom-topic", default="/aqua_sonar_loc/odometry")
    parser.add_argument("--sonar-status-topic", default="/aqua_sonar_loc/status")
    parser.add_argument("--points-topic", default="/aqua_sonar_loc/points_filtered")
    parser.add_argument("--pose-graph-path-topic", default="/aqua_pose_graph/path")
    parser.add_argument("--loop-constraint-topic", default="/aqua_pose_graph/loop_constraint")
    parser.add_argument("--loop-status-topic", default="/mbes_loop_closure/status")
    parser.add_argument("--application-id", default="aqua_localization MBES-SLAM beach_pond")
    parser.add_argument("--fan-stride", type=int, default=4,
                        help="Log every Nth fan to keep the .rrd small (default: 4)")
    parser.add_argument("--max-fan-points", type=int, default=512,
                        help="Subsample each fan to at most this many points")
    return parser.parse_args()


def odometry_position(msg) -> np.ndarray:
    p = msg.pose.pose.position
    return np.array([p.x, p.y, p.z], dtype=np.float64)


def odometry_quat(msg) -> tuple[float, float, float, float]:
    q = msg.pose.pose.orientation
    return q.x, q.y, q.z, q.w


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def path_samples(msg, fallback_time: float) -> list[tuple[float, np.ndarray]]:
    samples: list[tuple[float, np.ndarray]] = []
    for pose_stamped in msg.poses:
        p = pose_stamped.pose.position
        t = stamp_seconds(pose_stamped.header.stamp)
        if t <= 0.0:
            t = fallback_time
        samples.append((t, np.array([p.x, p.y, p.z], dtype=np.float64)))
    return samples


def loop_edge_segments(
    pose_graph_world: np.ndarray,
    loop_constraints: list[tuple[float, int, int]],
) -> list[np.ndarray]:
    segments = []
    seen_edges: set[tuple[int, int]] = set()
    for _t, from_id, to_id in loop_constraints:
        edge = (from_id, to_id)
        if edge in seen_edges:
            continue
        seen_edges.add(edge)
        if from_id < 0 or to_id < 0:
            continue
        if from_id >= len(pose_graph_world) or to_id >= len(pose_graph_world):
            continue
        segments.append(np.asarray([pose_graph_world[from_id], pose_graph_world[to_id]]))
    return segments


def decode_pointcloud2(msg) -> np.ndarray:
    """Decode a sensor_msgs/PointCloud2 to (N, 3) float32 in the sonar frame."""
    fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float32)
    if fields["x"][1] != 7:  # only float32 supported here
        return np.zeros((0, 3), dtype=np.float32)
    n = msg.width * msg.height
    out = np.empty((n, 3), dtype=np.float32)
    fmt = "<f"
    fox, foy, foz = fields["x"][0], fields["y"][0], fields["z"][0]
    data = bytes(msg.data)
    step = msg.point_step
    for i in range(n):
        b = i * step
        out[i, 0] = struct.unpack_from(fmt, data, b + fox)[0]
        out[i, 1] = struct.unpack_from(fmt, data, b + foy)[0]
        out[i, 2] = struct.unpack_from(fmt, data, b + foz)[0]
    finite = np.isfinite(out).all(axis=1)
    return out[finite]


def interpolate_pose(times: np.ndarray, positions: np.ndarray,
                     quats: np.ndarray, t_query: float) -> tuple[np.ndarray, np.ndarray] | None:
    """Linearly interpolate position + slerp-substitute (nearest) for orientation
    at t_query against (sorted) times. Returns None if out of range."""
    if times.size < 2 or t_query < times[0] or t_query > times[-1]:
        return None
    i = int(np.searchsorted(times, t_query) - 1)
    i = max(0, min(i, times.size - 2))
    t0, t1 = times[i], times[i + 1]
    if t1 == t0:
        a = 0.0
    else:
        a = (t_query - t0) / (t1 - t0)
    p = (1 - a) * positions[i] + a * positions[i + 1]
    # Use the nearer quaternion (cheap; multibeam fans are tightly spaced).
    q = quats[i] if a < 0.5 else quats[i + 1]
    return p, q


def main() -> int:
    args = parse_args()
    bag_dir = args.bag if args.bag.is_dir() else args.bag.parent
    if not bag_dir.is_dir():
        sys.stderr.write(f"not a rosbag2 directory: {bag_dir}\n")
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)

    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(
                name="MBES-SLAM beach_pond — accumulated fans + trajectories",
                origin="/world",
                contents=[
                    "+ /world/**",
                    "- /world/imu_estimate_hidden/**",
                ],
                background=[15, 22, 32],
                eye_controls=rrb.archetypes.EyeControls3D(
                    position=[10.0, 10.0, 60.0],
                    look_target=[15.0, 12.0, -2.0],
                    eye_up=[0.0, 0.0, 1.0],
                ),
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="z (m): reference / aqua_imu_loc / aqua_sonar_loc",
                    contents="/plots/depth/**",
                ),
                rrb.TimeSeriesView(
                    name="aqua_sonar_loc fitness",
                    contents="/plots/sonar/fitness",
                ),
                rrb.TimeSeriesView(
                    name="aqua_sonar_loc inliers",
                    contents="/plots/sonar/inliers",
                ),
                rrb.TimeSeriesView(
                    name="MBES loop closure status",
                    contents="/plots/loop_closure/**",
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

    # Pass 1: collect reference odometry and other buffers.
    ref_buf: list[tuple[float, np.ndarray, tuple[float, float, float, float]]] = []
    imu_buf: list[tuple[float, np.ndarray]] = []
    sonar_buf: list[tuple[float, np.ndarray]] = []
    fans_buf: list[tuple[float, np.ndarray]] = []
    fitness_buf: list[tuple[float, float]] = []
    inliers_buf: list[tuple[float, int]] = []
    pose_graph_paths: list[tuple[float, list[tuple[float, np.ndarray]]]] = []
    loop_constraints: list[tuple[float, int, int]] = []
    loop_status: list[tuple[float, bool, bool, float, float, float, int, int, str]] = []

    targets = {
        args.reference_topic, args.imu_topic, args.sonar_odom_topic,
        args.sonar_status_topic, args.points_topic, args.pose_graph_path_topic,
        args.loop_constraint_topic, args.loop_status_topic,
    }

    with AnyReader([bag_dir]) as reader:
        wanted = [c for c in reader.connections if c.topic in targets]
        if not wanted:
            sys.stderr.write(f"no matching topics. expected: {targets}\n")
            return 2

        for connection, t_ns, raw in reader.messages(connections=wanted):
            t = t_ns * 1e-9
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception:
                continue

            if connection.topic == args.reference_topic:
                ref_buf.append((t, odometry_position(msg), odometry_quat(msg)))
            elif connection.topic == args.imu_topic:
                imu_buf.append((t, odometry_position(msg)))
            elif connection.topic == args.sonar_odom_topic:
                sonar_buf.append((t, odometry_position(msg)))
            elif connection.topic == args.sonar_status_topic:
                # aqua_msgs/ScanMatchingStatus exposes fitness_score and the
                # filtered-points counts.
                try:
                    fitness_buf.append((t, float(msg.fitness_score)))
                    inliers_buf.append((t, int(msg.in_range_points)))
                except AttributeError:
                    pass
            elif connection.topic == args.points_topic:
                pts = decode_pointcloud2(msg)
                if pts.size:
                    if pts.shape[0] > args.max_fan_points:
                        idx = np.linspace(0, pts.shape[0] - 1, args.max_fan_points).astype(int)
                        pts = pts[idx]
                    fans_buf.append((t, pts))
            elif connection.topic == args.pose_graph_path_topic:
                samples = path_samples(msg, t)
                if samples:
                    pose_graph_paths.append((t, samples))
            elif connection.topic == args.loop_constraint_topic:
                try:
                    loop_constraints.append((t, int(msg.from_id), int(msg.to_id)))
                except AttributeError:
                    pass
            elif connection.topic == args.loop_status_topic:
                try:
                    loop_status.append((
                        t, bool(msg.accepted), bool(msg.converged),
                        float(msg.fitness_score), float(msg.correction_translation_m),
                        float(msg.correction_rotation_rad), int(msg.current_id),
                        int(msg.candidate_id), str(msg.status)))
                except AttributeError:
                    pass

    print(
        f"buffered: ref={len(ref_buf)} imu={len(imu_buf)} sonar={len(sonar_buf)}"
        f" fans={len(fans_buf)} status={len(fitness_buf)}"
        f" pose_graph_paths={len(pose_graph_paths)} loops={len(loop_constraints)}"
        f" loop_status={len(loop_status)}"
    )
    if not ref_buf:
        sys.stderr.write("no reference odometry; cannot transform fans into world\n")
        return 3

    ref_t = np.array([t for t, _, _ in ref_buf])
    ref_xyz_raw = np.array([p for _, p, _ in ref_buf])
    ref_q = np.array([q for _, _, q in ref_buf])

    # MBES bags ship odometry in a UTM-style local frame (e.g. ~(-825, 620, -2)
    # for beach_pond), so subtracting the first reference pose puts the
    # rerun world origin at the start of the trajectory and keeps the auto-
    # camera framing tight.
    origin_xyz = ref_xyz_raw[0].copy()
    print(f"world origin offset (first ref pose): {origin_xyz.tolist()}")
    ref_xyz = ref_xyz_raw - origin_xyz

    # Umeyama-align the IMU and sonar estimates to the reference. They live in
    # their own first-IMU-pose origin which is geographically arbitrary versus
    # the reference's UTM frame, so without alignment they can drift hundreds
    # of meters off-screen.
    def _fit_alignment(est_t, est_xyz, label, min_samples=10, fallback=None):
        if est_t.size == 0:
            return fallback if fallback is not None else (np.eye(3), np.zeros(3))
        ref_at = interp_xyz(est_t, ref_t, ref_xyz_raw)
        valid = ~np.isnan(ref_at).any(axis=1)
        if valid.sum() < min_samples:
            print(f"  {label}: too few overlapping samples ({int(valid.sum())}), skipping align")
            return fallback if fallback is not None else (np.eye(3), np.zeros(3))
        R, t = umeyama_se3(est_xyz[valid], ref_at[valid])
        rmse = float(np.sqrt(np.mean(np.linalg.norm(
            (est_xyz[valid] @ R.T + t) - ref_at[valid], axis=1) ** 2)))
        print(f"  {label} aligned: RMSE={rmse:.3f} m on {int(valid.sum())} samples")
        return R, t

    def _align(buf, label):
        if not buf:
            return np.eye(3), np.zeros(3)
        est_t = np.array([t for t, _ in buf])
        est_xyz = np.array([p for _, p in buf])
        return _fit_alignment(est_t, est_xyz, label)

    print("Umeyama alignment vs reference:")
    R_imu, t_imu = _align(imu_buf, "imu")
    R_son, t_son = _align(sonar_buf, "sonar")

    t0 = ref_t[0]

    # Build chronological event stream so logging hits rerun in time order.
    events: list[tuple[float, str, int]] = []
    for i, (t, _, _) in enumerate(ref_buf):
        events.append((t, "ref", i))
    for i, (t, _) in enumerate(imu_buf):
        events.append((t, "imu", i))
    for i, (t, _) in enumerate(sonar_buf):
        events.append((t, "sonar", i))
    for i, (t, _) in enumerate(fans_buf):
        events.append((t, "fan", i))
    for i, (t, _) in enumerate(fitness_buf):
        events.append((t, "fit", i))
    for i, (t, _) in enumerate(inliers_buf):
        events.append((t, "inl", i))
    for i, (t, *_rest) in enumerate(loop_status):
        events.append((t, "loop_status", i))
    events.sort(key=lambda x: x[0])

    ref_path: list[np.ndarray] = []
    imu_path: list[np.ndarray] = []
    sonar_path: list[np.ndarray] = []
    fans_logged = 0

    for t, kind, i in events:
        rr.set_time("bag_time", duration=t - t0)

        if kind == "ref":
            p = ref_buf[i][1] - origin_xyz
            ref_path.append(p)
            # Decimate plot stream by 10x — 2986 ref samples is plenty.
            if i % 10 == 0:
                rr.log("plots/depth/reference", rr.Scalars(float(p[2])))

        elif kind == "imu":
            # Apply Umeyama alignment + origin offset to bring IMU estimate
            # into the same shifted reference frame as ref_path.
            p_raw = imu_buf[i][1]
            p = (R_imu @ p_raw + t_imu) - origin_xyz
            imu_path.append(p)
            if i % 20 == 0:  # IMU ticks at higher rate; decimate harder.
                rr.log("plots/depth/imu", rr.Scalars(float(p[2])))

        elif kind == "sonar":
            p_raw = sonar_buf[i][1]
            p = (R_son @ p_raw + t_son) - origin_xyz
            sonar_path.append(p)
            rr.log("plots/depth/sonar", rr.Scalars(float(p[2])))

        elif kind == "fan":
            if (fans_logged % args.fan_stride) != 0:
                fans_logged += 1
                continue
            t_fan, pts_local = fans_buf[i]
            pose = interpolate_pose(ref_t, ref_xyz, ref_q, t_fan)
            if pose is None:
                fans_logged += 1
                continue
            p_world, q = pose  # ref_xyz is already origin-shifted, no double subtract.
            R = quat_to_rot_matrix(*q)
            pts_world = pts_local @ R.T + p_world
            # Color by depth so the bathymetric structure pops.
            z = pts_world[:, 2]
            zmin, zmax = float(z.min()), float(z.max())
            if zmax - zmin > 1e-3:
                t_norm = (z - zmin) / (zmax - zmin)
            else:
                t_norm = np.zeros_like(z)
            r_ch = (1.0 - t_norm) * 80 + t_norm * 240
            g_ch = (1.0 - t_norm) * 180 + t_norm * 90
            b_ch = (1.0 - t_norm) * 240 + t_norm * 60
            colors = np.stack([r_ch, g_ch, b_ch], axis=1).astype(np.uint8)
            # Log fans as static so the accumulated bathymetric scan is visible
            # at every time slider position, not just the moment each fan came
            # in.
            rr.log(f"world/sonar_fans/{fans_logged:05d}",
                   rr.Points3D(pts_world, colors=colors, radii=0.15),
                   static=True)
            fans_logged += 1

        elif kind == "fit":
            rr.log("plots/sonar/fitness", rr.Scalars(float(fitness_buf[i][1])))
        elif kind == "inl":
            rr.log("plots/sonar/inliers", rr.Scalars(float(inliers_buf[i][1])))
        elif kind == "loop_status":
            (_t, accepted, _converged, fitness, correction_t, correction_r,
             _current_id, _candidate_id, _status) = loop_status[i]
            rr.log("plots/loop_closure/accepted", rr.Scalars(1.0 if accepted else 0.0))
            if np.isfinite(fitness):
                rr.log("plots/loop_closure/fitness", rr.Scalars(float(fitness)))
            if np.isfinite(correction_t):
                rr.log("plots/loop_closure/correction/translation_m",
                       rr.Scalars(float(correction_t)))
            if np.isfinite(correction_r):
                rr.log("plots/loop_closure/correction/rotation_rad",
                       rr.Scalars(float(correction_r)))

    # Log full trajectories as static (after the timeline scrub), so they show
    # at every time slider position without bloating the recording with N
    # incremental snapshots.
    if ref_path:
        rr.log("world/reference/path",
               rr.LineStrips3D([np.asarray(ref_path)],
                               colors=REF_COLOR, radii=0.4),
               static=True)
    # IMU path on MBES bags is dead-reckoned and drifts ~40 m RMSE, which
    # dominates the 3D scene visually. Hide it from the default view (still
    # available via the entity panel) so the bathymetric scan + reference
    # trajectory tell a cleaner visual story.
    if imu_path:
        rr.log("world/imu_estimate_hidden/path",
               rr.LineStrips3D([np.asarray(imu_path)],
                               colors=IMU_COLOR, radii=0.2),
               static=True)
    if sonar_path:
        rr.log("world/sonar_estimate/path",
               rr.LineStrips3D([np.asarray(sonar_path)],
                               colors=SONAR_COLOR, radii=0.4),
               static=True)

    pose_graph_world: np.ndarray | None = None
    if pose_graph_paths:
        _, pose_graph_samples = max(pose_graph_paths, key=lambda item: len(item[1]))
        pg_t = np.array([t for t, _ in pose_graph_samples])
        pg_xyz = np.array([p for _, p in pose_graph_samples])
        R_pg, t_pg = _fit_alignment(
            pg_t, pg_xyz, "pose_graph", min_samples=3, fallback=(R_son, t_son))
        pose_graph_world = (pg_xyz @ R_pg.T + t_pg) - origin_xyz
        rr.log("world/pose_graph/path",
               rr.LineStrips3D([pose_graph_world],
                               colors=POSE_GRAPH_COLOR, radii=0.55),
               static=True)

    if pose_graph_world is not None and loop_constraints:
        loop_segments = loop_edge_segments(pose_graph_world, loop_constraints)
        if loop_segments:
            rr.log("world/pose_graph/loop_edges",
                   rr.LineStrips3D(loop_segments, colors=LOOP_COLOR, radii=0.8),
                   static=True)

    print(f"logged {fans_logged // args.fan_stride} fans (stride={args.fan_stride})")
    print(f"wrote {args.out} ({args.out.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
