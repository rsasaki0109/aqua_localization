#!/usr/bin/env python3
"""Generate a synthetic underwater survey rosbag2 for end-to-end replay tests.

A vehicle at constant depth sits still briefly, smoothly speeds up, and drives
a circle over a bumpy seabed (long runs complete a lap and revisit the start). The bag contains:

- ``/imu/data`` (sensor_msgs/Imu, 100 Hz): exact specific force and body rate
  plus seeded white noise and constant biases, with the true orientation;
- ``/pressure`` (sensor_msgs/FluidPressure, 10 Hz): fresh-water pressure;
- ``/dvl/twist`` (geometry_msgs/TwistStamped, 5 Hz): body-frame velocity;
- ``/sonar/points`` (sensor_msgs/PointCloud2, 5 Hz): a 96-beam multibeam fan
  ray-cast against the seabed, in the body frame;
- ``/ground_truth/odometry`` (nav_msgs/Odometry, 20 Hz): the true pose in the
  start-anchored map frame (origin and heading of the first pose), for APE.

Frames follow the stack: world z-up, body x-forward / z-up. Sonar messages
are written with a small receive delay after their header stamp, so the
bag's receive order differs from measurement order the way real recordings
do. Everything is seeded, so the same arguments produce identical bags.
"""

import argparse
import inspect
import math
import shutil
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

GRAVITY = 9.80665
WATER_DENSITY = 1000.0
ATMOSPHERE_PA = 101325.0


def ros2_typestore():
    for name in ("ROS2_JAZZY", "ROS2_HUMBLE", "LATEST"):
        store = getattr(Stores, name, None)
        if store is not None:
            return get_typestore(store)
    raise RuntimeError("rosbags does not expose a ROS 2 typestore")


def open_writer(dst: Path):
    if "version" in inspect.signature(Writer).parameters:
        return Writer(dst, version=9)
    return Writer(dst)


class Seabed:
    def __init__(self, rng: np.random.Generator, base_z: float):
        self.base_z = base_z
        self.hills = [
            (rng.uniform(-14, 14), rng.uniform(-14, 14), rng.uniform(-1.5, 1.5),
             rng.uniform(1.5, 4.0))
            for _ in range(14)
        ]

    def height(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        z = self.base_z + 0.6 * np.sin(0.35 * x) * np.cos(0.27 * y) + 0.4 * np.sin(0.13 * x + 0.7)
        for cx, cy, amp, sigma in self.hills:
            z = z + amp * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2.0 * sigma ** 2))
        return z


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def truth(t: float, radius: float, speed: float, depth: float, static_s: float, ramp_s: float):
    """Pose and derivatives: parked for static_s, smoothstep speed-up over ramp_s, then cruise."""
    w_max = speed / radius
    tau = min(max((t - static_s) / ramp_s, 0.0), 1.0) if ramp_s > 0.0 else float(t >= static_s)
    if t <= static_s:
        angle, v, dv = 0.0, 0.0, 0.0
    elif t < static_s + ramp_s:
        angle = w_max * ramp_s * (tau ** 3 - 0.5 * tau ** 4)
        v = speed * (3.0 * tau ** 2 - 2.0 * tau ** 3)
        dv = speed * (6.0 * tau - 6.0 * tau ** 2) / ramp_s
    else:
        angle = w_max * (0.5 * ramp_s + (t - static_s - ramp_s))
        v, dv = speed, 0.0
    w = v / radius
    c, s = math.cos(angle), math.sin(angle)
    position = np.array([radius * c, radius * s, -depth])
    tangent = np.array([-s, c, 0.0])
    velocity = v * tangent
    accel = dv * tangent + (v * v / radius) * np.array([-c, -s, 0.0])
    yaw = angle + math.pi / 2.0
    return position, velocity, accel, yaw, w


def raycast_fan(seabed: Seabed, position, rotation, beam_angles, max_range, step=0.05):
    dirs_body = np.stack(
        [np.zeros_like(beam_angles), np.sin(beam_angles), -np.cos(beam_angles)], axis=1)
    dirs_world = dirs_body @ rotation.T
    ranges = np.arange(step, max_range, step)
    pts = position[None, None, :] + ranges[None, :, None] * dirs_world[:, None, :]
    below = pts[..., 2] <= seabed.height(pts[..., 0], pts[..., 1])
    hit = below.any(axis=1)
    first = np.argmax(below, axis=1)
    hit_ranges = ranges[first]
    return dirs_body[hit] * hit_ranges[hit, None], hit


def stamp(typestore, t_ns: int):
    Time = typestore.types["builtin_interfaces/msg/Time"]
    return Time(sec=t_ns // 1_000_000_000, nanosec=t_ns % 1_000_000_000)


def header(typestore, t_ns: int, frame_id: str):
    Header = typestore.types["std_msgs/msg/Header"]
    return Header(stamp=stamp(typestore, t_ns), frame_id=frame_id)


def vector3(typestore, v):
    Vector3 = typestore.types["geometry_msgs/msg/Vector3"]
    return Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def quaternion_from_yaw(typestore, yaw: float):
    Quaternion = typestore.types["geometry_msgs/msg/Quaternion"]
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def point_cloud(typestore, t_ns: int, frame_id: str, points: np.ndarray):
    PointCloud2 = typestore.types["sensor_msgs/msg/PointCloud2"]
    PointField = typestore.types["sensor_msgs/msg/PointField"]
    fields = [PointField(name=n, offset=4 * i, datatype=7, count=1) for i, n in enumerate("xyz")]
    data = points.astype(np.float32).reshape(-1).view(np.uint8).copy()
    return PointCloud2(
        header=header(typestore, t_ns, frame_id), height=1, width=points.shape[0],
        fields=fields, is_bigendian=False, point_step=12, row_step=12 * points.shape[0],
        data=data, is_dense=True)


def generate(args) -> int:
    dst = Path(args.dst)
    if dst.exists():
        if not args.overwrite:
            print(f"refusing to overwrite {dst}; pass --overwrite", file=sys.stderr)
            return 2
        shutil.rmtree(dst)

    rng = np.random.default_rng(args.seed)
    seabed = Seabed(rng, base_z=-args.depth - args.altitude)
    ts = ros2_typestore()
    Imu = ts.types["sensor_msgs/msg/Imu"]
    FluidPressure = ts.types["sensor_msgs/msg/FluidPressure"]
    TwistStamped = ts.types["geometry_msgs/msg/TwistStamped"]
    Twist = ts.types["geometry_msgs/msg/Twist"]
    Odometry = ts.types["nav_msgs/msg/Odometry"]
    PoseWithCovariance = ts.types["geometry_msgs/msg/PoseWithCovariance"]
    TwistWithCovariance = ts.types["geometry_msgs/msg/TwistWithCovariance"]
    Pose = ts.types["geometry_msgs/msg/Pose"]
    Point = ts.types["geometry_msgs/msg/Point"]

    accel_bias = np.array([0.02, -0.015, 0.01])
    gyro_bias = np.array([0.0008, -0.0005, 0.0006])
    base_ns = int(args.start_time_s * 1e9)
    beam_angles = np.radians(np.linspace(-60.0, 60.0, args.beams))
    gravity_world = np.array([0.0, 0.0, -GRAVITY])

    records = []  # (receive_ns, topic, msgtype, message)
    imu_dt = 1.0 / args.imu_rate_hz
    for i in range(int(args.duration_s * args.imu_rate_hz) + 1):
        t = i * imu_dt
        position, velocity, accel, yaw, w = truth(t, args.radius, args.speed, args.depth, args.static_s, args.ramp_s)
        R = rot_z(yaw)
        f_body = R.T @ (accel - gravity_world) + accel_bias + rng.normal(0.0, 0.02, 3)
        gyro = np.array([0.0, 0.0, w]) + gyro_bias + rng.normal(0.0, 0.002, 3)
        t_ns = base_ns + round(t * 1e9)
        msg = Imu(
            header=header(ts, t_ns, "base_link"),
            orientation=quaternion_from_yaw(ts, yaw),
            orientation_covariance=np.full(9, 0.0, dtype=np.float64),
            angular_velocity=vector3(ts, gyro),
            angular_velocity_covariance=np.full(9, 0.0, dtype=np.float64),
            linear_acceleration=vector3(ts, f_body),
            linear_acceleration_covariance=np.full(9, 0.0, dtype=np.float64))
        records.append((t_ns, "/imu/data", "sensor_msgs/msg/Imu", msg))
        if i % max(1, round(args.imu_rate_hz / 20.0)) == 0:
            # Express truth in the frame of the first pose, which is where the
            # estimators start (origin, zero yaw).
            p0, _, _, yaw0, _ = truth(0.0, args.radius, args.speed, args.depth, args.static_s,
                                      args.ramp_s)
            rel = rot_z(yaw0).T @ (position - p0)
            gt = Odometry(
                header=header(ts, t_ns, "map"), child_frame_id="base_link",
                pose=PoseWithCovariance(
                    pose=Pose(position=Point(x=float(rel[0]), y=float(rel[1]), z=float(rel[2])),
                              orientation=quaternion_from_yaw(ts, yaw - yaw0)),
                    covariance=np.zeros(36, dtype=np.float64)),
                twist=TwistWithCovariance(
                    twist=Twist(linear=vector3(ts, R.T @ velocity),
                                angular=vector3(ts, np.array([0.0, 0.0, w]))),
                    covariance=np.zeros(36, dtype=np.float64)))
            records.append((t_ns, "/ground_truth/odometry", "nav_msgs/msg/Odometry", gt))

    for i in range(1, int(args.duration_s * 10.0) + 1):
        t = i / 10.0
        position, *_ = truth(t, args.radius, args.speed, args.depth, args.static_s, args.ramp_s)
        depth = -position[2] + rng.normal(0.0, 0.01)
        t_ns = base_ns + round(t * 1e9)
        msg = FluidPressure(
            header=header(ts, t_ns, "base_link"),
            fluid_pressure=ATMOSPHERE_PA + WATER_DENSITY * GRAVITY * depth, variance=1.0)
        records.append((t_ns, "/pressure", "sensor_msgs/msg/FluidPressure", msg))

    for i in range(1, int(args.duration_s * 5.0) + 1):
        t = i / 5.0
        position, velocity, accel, yaw, w = truth(t, args.radius, args.speed, args.depth, args.static_s, args.ramp_s)
        v_body = rot_z(yaw).T @ velocity + rng.normal(0.0, 0.01, 3)
        t_ns = base_ns + round(t * 1e9)
        msg = TwistStamped(
            header=header(ts, t_ns, "base_link"),
            twist=Twist(linear=vector3(ts, v_body), angular=vector3(ts, np.zeros(3))))
        records.append((t_ns, "/dvl/twist", "geometry_msgs/msg/TwistStamped", msg))

        points, _ = raycast_fan(seabed, position, rot_z(yaw), beam_angles, args.max_range_m)
        if points.shape[0] < 10:
            continue
        points = points + rng.normal(0.0, 0.02, points.shape)
        # Sonar arrives late, as real multibeam drivers do.
        receive_ns = t_ns + round(args.sonar_delay_s * 1e9)
        records.append((receive_ns, "/sonar/points", "sensor_msgs/msg/PointCloud2",
                        point_cloud(ts, t_ns, "base_link", points)))

    records.sort(key=lambda r: r[0])
    with open_writer(dst) as writer:
        connections = {}
        for receive_ns, topic, msgtype, msg in records:
            if topic not in connections:
                connections[topic] = writer.add_connection(topic, msgtype, typestore=ts)
            writer.write(connections[topic], receive_ns, ts.serialize_cdr(msg, msgtype))
    counts = {}
    for _, topic, _, _ in records:
        counts[topic] = counts.get(topic, 0) + 1
    print(f"wrote {dst}: " + ", ".join(f"{t}={n}" for t, n in sorted(counts.items())))
    return 0


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--dst", required=True, help="Destination rosbag2 directory.")
    p.add_argument("--duration-s", type=float, default=120.0)
    p.add_argument("--radius", type=float, default=10.0)
    p.add_argument("--speed", type=float, default=0.6, help="Forward speed in m/s.")
    p.add_argument("--depth", type=float, default=2.0, help="Vehicle depth in m.")
    p.add_argument("--static-s", type=float, default=5.0,
                   help="Parked time at the start (lets static IMU bias init settle).")
    p.add_argument("--ramp-s", type=float, default=5.0, help="Smooth speed-up duration.")
    p.add_argument("--altitude", type=float, default=8.0, help="Nominal seabed altitude in m.")
    p.add_argument("--beams", type=int, default=96)
    p.add_argument("--max-range-m", type=float, default=30.0)
    p.add_argument("--imu-rate-hz", type=float, default=100.0)
    p.add_argument("--sonar-delay-s", type=float, default=0.05)
    p.add_argument("--start-time-s", type=float, default=1700000000.0)
    p.add_argument("--seed", type=int, default=11)
    p.add_argument("--overwrite", action="store_true")
    return p.parse_args(argv)


def main(argv=None):
    return generate(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
