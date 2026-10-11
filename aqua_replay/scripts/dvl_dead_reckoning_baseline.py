#!/usr/bin/env python3
"""Classical DVL + attitude dead reckoning, as a baseline for aqua_graph.

What a commercial DVL navigator does without a filter: rotate each DVL
bottom-track velocity into the world with the current attitude and integrate
it. Roll and pitch come from an AHRS (orientation of an Imu topic); yaw
(--yaw-source) from a heading topic (e.g. a GNSS compass), from the AHRS
itself, or from integrating the gyro's world-frame yaw rate (no absolute
heading at all, as on a vehicle without a compass). The lever-arm term
w x r uses the AHRS angular velocity. Depth is held at zero
(surface vessel), matching aqua_graph's depth-hold profile.

The trajectory is written as nav_msgs/Odometry of the IMU origin into a
small rosbag2, so evaluate_replay_trajectories.py can score it next to an
offline_replay output (pass both with --bag):

    dvl_dead_reckoning_baseline.py --bag beach_pond_ros2 --bag beach_pond_aux \\
        --start-offset-s 855 --dst /tmp/beach_pond_dvl_dr
"""

import argparse
import inspect
import shutil
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch),
                              np.cos(yaw), np.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(r):
    w = np.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = np.copysign(np.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    return np.array([x, y, z, w])


def heading_of_x(r):
    return np.arctan2(r[1, 0], r[0, 0])


def stamp(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def interp_angle(t, ts, angles):
    return np.angle(np.interp(t, ts, np.cos(angles)) + 1j * np.interp(t, ts, np.sin(angles)))


def run(args) -> int:
    dst = Path(args.dst)
    if dst.exists():
        if not args.overwrite:
            print(f"refusing to overwrite {dst}; pass --overwrite", file=sys.stderr)
            return 2
        shutil.rmtree(dst)

    ahrs, gyro, dvl, heading = [], [], [], []
    with AnyReader([Path(b) for b in args.bag]) as reader:
        use_heading = args.yaw_source == "heading"
        topics = {args.ahrs_topic, args.dvl_topic} | ({args.heading_topic} if use_heading else set())
        conns = [c for c in reader.connections if c.topic in topics]
        missing = topics - {c.topic for c in conns}
        if missing:
            raise SystemExit(f"input bags lack {sorted(missing)}")
        t_first = None
        for conn, t_ns, raw in reader.messages(connections=conns):
            t_first = t_ns if t_first is None else t_first
            if (t_ns - t_first) * 1e-9 < args.start_offset_s:
                continue
            if args.duration_s > 0 and (t_ns - t_first) * 1e-9 > args.start_offset_s + args.duration_s:
                break
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == args.ahrs_topic:
                q = msg.orientation
                ahrs.append((stamp(msg), q.x, q.y, q.z, q.w))
                w = msg.angular_velocity
                gyro.append((w.x, w.y, w.z))
            elif conn.topic == args.dvl_topic:
                v = msg.twist.twist.linear if hasattr(msg.twist, "twist") else msg.twist.linear
                dvl.append((stamp(msg), v.x, v.y, v.z))
            else:
                q = msg.orientation
                heading.append((stamp(msg), heading_of_x(quat_to_matrix((q.x, q.y, q.z, q.w)))))

    ahrs = np.array(sorted(ahrs))
    gyro = np.array(gyro)
    dvl = np.array(sorted(dvl))
    heading = np.array(sorted(heading)) if heading else None
    r_bd = rpy_matrix(*args.dvl_rpy)
    lever = np.array(args.dvl_lever_arm)

    gyro_yaw = None
    if args.yaw_source == "gyro":
        # World-frame yaw rate (R w)_z, integrated with the trapezoid rule.
        rate = np.array([(quat_to_matrix(q) @ w)[2] for q, w in zip(ahrs[:, 1:5], gyro)])
        gyro_yaw = np.concatenate([[0.0], np.cumsum(
            0.5 * (rate[1:] + rate[:-1]) * np.diff(ahrs[:, 0]))])

    # Attitude at every DVL stamp: AHRS roll/pitch, yaw from --yaw-source.
    def attitude(t):
        i = int(np.clip(np.searchsorted(ahrs[:, 0], t), 1, len(ahrs) - 1))
        j = i if abs(ahrs[i, 0] - t) < abs(ahrs[i - 1, 0] - t) else i - 1
        r = quat_to_matrix(ahrs[j, 1:5])
        if heading is not None:
            yaw = interp_angle(t, heading[:, 0], heading[:, 1])
            r = rpy_matrix(0, 0, yaw - heading_of_x(r)) @ r
        elif gyro_yaw is not None:
            r = rpy_matrix(0, 0, np.interp(t, ahrs[:, 0], gyro_yaw) - heading_of_x(r)) @ r
        return r, gyro[j]

    rows = []
    position = np.zeros(3)
    prev_t, prev_v = None, None
    rejected = 0
    for t, vx, vy, vz in dvl:
        # Same validity gate as aqua_graph (dvl.max_speed_mps): drops the
        # Nortek "no bottom lock" sentinel (-32.768 m/s per beam). The
        # trapezoid then spans the gap, i.e. velocity is interpolated.
        if not np.isfinite([vx, vy, vz]).all() or np.linalg.norm([vx, vy, vz]) > args.max_speed_mps:
            rejected += 1
            continue
        if t < ahrs[0, 0] or t > ahrs[-1, 0]:
            continue
        if heading is not None and not heading[0, 0] <= t <= heading[-1, 0]:
            continue
        r, w = attitude(t)
        v_world = r @ (r_bd @ np.array([vx, vy, vz]) - np.cross(w, lever))
        if prev_t is not None:
            position = position + 0.5 * (prev_v + v_world) * (t - prev_t)
        position[2] = 0.0
        prev_t, prev_v = t, v_world
        rows.append((t, *position, *matrix_to_quat(r)))

    ts = None
    for name in ("ROS2_JAZZY", "ROS2_HUMBLE", "LATEST"):
        store = getattr(Stores, name, None)
        if store is not None:
            ts = get_typestore(store)
            break
    odom_type = "nav_msgs/msg/Odometry"
    Odometry = ts.types[odom_type]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    gm = ts.types
    writer_kwargs = {"version": 9} if "version" in inspect.signature(Writer).parameters else {}
    with Writer(dst, **writer_kwargs) as writer:
        conn = writer.add_connection(args.topic, odom_type, typestore=ts)
        for t, x, y, z, qx, qy, qz, qw in rows:
            ns = int(round(t * 1e9))
            msg = Odometry(
                header=Header(stamp=Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000),
                              frame_id="map"),
                child_frame_id="base_link",
                pose=gm["geometry_msgs/msg/PoseWithCovariance"](
                    pose=gm["geometry_msgs/msg/Pose"](
                        position=gm["geometry_msgs/msg/Point"](x=x, y=y, z=z),
                        orientation=gm["geometry_msgs/msg/Quaternion"](x=qx, y=qy, z=qz, w=qw)),
                    covariance=np.zeros(36)),
                twist=gm["geometry_msgs/msg/TwistWithCovariance"](
                    twist=gm["geometry_msgs/msg/Twist"](
                        linear=gm["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0),
                        angular=gm["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)),
                    covariance=np.zeros(36)))
            writer.write(conn, ns, ts.serialize_cdr(msg, odom_type))
    print(f"wrote {dst}: {len(rows)} poses from {len(dvl)} DVL samples ({rejected} rejected) "
          f"(yaw from {args.yaw_source})")
    return 0


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--bag", action="append", required=True, help="Input rosbag2 (repeatable).")
    p.add_argument("--dst", required=True)
    p.add_argument("--topic", default="/baseline/dvl_dr/odometry")
    p.add_argument("--ahrs-topic", default="/nav/processed/microstrain/imu/madgwick")
    p.add_argument("--dvl-topic", default="/nav/processed/nortek/dvl_twist")
    p.add_argument("--yaw-source", choices=("heading", "ahrs", "gyro"), default="heading")
    p.add_argument("--heading-topic", default="/nav/processed/navsat/ubx_hdg/imu",
                   help="Imu topic whose orientation gives the yaw of the AHRS x axis "
                        "(--yaw-source heading).")
    p.add_argument("--dvl-rpy", type=float, nargs=3, default=[np.pi, 0.0, -np.pi / 2],
                   help="DVL mount rotation in the AHRS frame (roll pitch yaw, rad).")
    p.add_argument("--dvl-lever-arm", type=float, nargs=3, default=[1.105, 0.093, -0.843])
    p.add_argument("--max-speed-mps", type=float, default=5.0,
                   help="Reject DVL samples faster than this (invalid / no bottom lock).")
    p.add_argument("--start-offset-s", type=float, default=0.0)
    p.add_argument("--duration-s", type=float, default=0.0)
    p.add_argument("--overwrite", action="store_true")
    return p.parse_args(argv)


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
