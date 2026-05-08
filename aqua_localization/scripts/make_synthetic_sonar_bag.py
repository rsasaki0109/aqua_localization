#!/usr/bin/env python3
"""Generate a synthetic rosbag2 with sensor_msgs/PointCloud2 sonar scans.

The fictional scene is a fixed point field in the world. A robot follows a straight
linear trajectory along world +x and at each timestamp the visible portion of the
point field is transformed into the body frame and serialized as a PointCloud2 in the
robot/sonar frame. This produces a deterministic input that exercises the
`aqua_sonar_loc` preprocessor + ICP scan matcher end-to-end without any real sonar
hardware.

The generator is intentionally minimal:

- world points are drawn from a uniform random distribution over a bounded box ahead
  of the trajectory (the seed is fixed for reproducibility);
- there is no rotation in the trajectory — yaw stays at zero, only x increases;
- there is no per-scan noise unless `--xy-noise-stddev` is passed.

The body-frame point cloud is the world points expressed relative to the current
robot pose (rotation = identity, so the transform is just `body = world - robot`).
"""

import argparse
import shutil
import struct
import sys
from pathlib import Path

import numpy as np

from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore


def linear_trajectory(num_steps: int, dt: float, speed_m_s: float) -> np.ndarray:
    """Return an (N, 3) array of robot positions sampled at uniform dt."""
    return np.column_stack(
        [
            np.arange(num_steps, dtype=np.float64) * dt * speed_m_s,
            np.zeros(num_steps),
            np.zeros(num_steps),
        ]
    )


def sample_world_points(
    rng: np.random.Generator,
    num_points: int,
    x_range: tuple,
    y_range: tuple,
    z_range: tuple,
) -> np.ndarray:
    """Sample world points uniformly in the configured box."""
    return np.column_stack(
        [
            rng.uniform(*x_range, size=num_points),
            rng.uniform(*y_range, size=num_points),
            rng.uniform(*z_range, size=num_points),
        ]
    )


def transform_world_to_body(points_world: np.ndarray, robot_pos: np.ndarray) -> np.ndarray:
    """Translate the world points into the body frame (no rotation)."""
    return points_world - robot_pos[np.newaxis, :]


def filter_by_range(points_body: np.ndarray, max_range_m: float) -> np.ndarray:
    distances = np.linalg.norm(points_body, axis=1)
    return points_body[distances <= max_range_m]


def encode_xyz_float32_message(
    typestore,
    points_body: np.ndarray,
    stamp_sec: int,
    stamp_nsec: int,
    frame_id: str,
):
    """Build a sensor_msgs/PointCloud2 with three float32 xyz fields."""
    PointCloud2 = typestore.types["sensor_msgs/msg/PointCloud2"]
    PointField = typestore.types["sensor_msgs/msg/PointField"]
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]

    fields = [
        PointField(name="x", offset=0, datatype=7, count=1),
        PointField(name="y", offset=4, datatype=7, count=1),
        PointField(name="z", offset=8, datatype=7, count=1),
    ]
    point_step = 12
    n = points_body.shape[0]
    row_step = point_step * n

    data = np.zeros(row_step, dtype=np.uint8)
    floats = points_body.astype(np.float32)
    data_view = data.view(np.float32)
    data_view[0::3] = floats[:, 0]
    data_view[1::3] = floats[:, 1]
    data_view[2::3] = floats[:, 2]

    msg = PointCloud2(
        header=Header(stamp=Time(sec=stamp_sec, nanosec=stamp_nsec), frame_id=frame_id),
        height=1,
        width=n,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=row_step,
        data=data,
        is_dense=True,
    )
    return msg


def generate_bag(args) -> int:
    rng = np.random.default_rng(args.seed)
    world_points = sample_world_points(
        rng,
        args.num_points,
        x_range=tuple(args.world_x),
        y_range=tuple(args.world_y),
        z_range=tuple(args.world_z),
    )

    trajectory = linear_trajectory(args.num_steps, args.dt, args.speed_m_s)

    dst = Path(args.dst)
    if dst.exists():
        if args.overwrite:
            shutil.rmtree(dst)
        else:
            print(f"refusing to overwrite existing {dst}; pass --overwrite", file=sys.stderr)
            return 2

    typestore = get_typestore(Stores.ROS2_JAZZY)

    base_ns = int(args.start_time_s * 1_000_000_000)
    dt_ns = int(args.dt * 1_000_000_000)

    with Writer(dst, version=9) as writer:
        conn = writer.add_connection(
            topic=args.topic,
            msgtype="sensor_msgs/msg/PointCloud2",
            typestore=typestore,
        )

        emitted = 0
        for step, pose in enumerate(trajectory):
            body_points = transform_world_to_body(world_points, pose)
            body_points = filter_by_range(body_points, args.max_range_m)
            if args.xy_noise_stddev > 0.0 and body_points.size > 0:
                noise = rng.normal(0.0, args.xy_noise_stddev, size=body_points.shape)
                body_points = body_points + noise
            if body_points.shape[0] < args.min_points:
                continue
            t_ns = base_ns + step * dt_ns
            sec = t_ns // 1_000_000_000
            nanosec = t_ns % 1_000_000_000
            msg = encode_xyz_float32_message(
                typestore, body_points, sec, nanosec, args.frame_id
            )
            writer.write(
                conn,
                t_ns,
                typestore.serialize_cdr(msg, "sensor_msgs/msg/PointCloud2"),
            )
            emitted += 1

        print(
            f"wrote {emitted} PointCloud2 messages on {args.topic} to {dst} "
            f"(world points: {args.num_points}, steps: {args.num_steps})"
        )
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Generate a synthetic sonar PointCloud2 rosbag2 for aqua_sonar_loc testing."
    )
    parser.add_argument("--dst", required=True, help="Destination rosbag2 directory.")
    parser.add_argument("--topic", default="/sonar/points",
                        help="PointCloud2 topic (default: /sonar/points).")
    parser.add_argument("--frame-id", default="sonar_link",
                        help="header.frame_id of each PointCloud2 (default: sonar_link).")
    parser.add_argument("--num-points", type=int, default=400,
                        help="World point count (default: 400).")
    parser.add_argument("--num-steps", type=int, default=120,
                        help="Number of scans / time steps (default: 120).")
    parser.add_argument("--dt", type=float, default=0.1, help="Step duration in seconds.")
    parser.add_argument("--speed-m-s", type=float, default=0.5,
                        help="Forward speed along world +x (default: 0.5 m/s).")
    parser.add_argument("--max-range-m", type=float, default=40.0,
                        help="Range filter applied per scan (default: 40 m).")
    parser.add_argument("--world-x", type=float, nargs=2, default=[5.0, 60.0])
    parser.add_argument("--world-y", type=float, nargs=2, default=[-12.0, 12.0])
    parser.add_argument("--world-z", type=float, nargs=2, default=[-3.0, 3.0])
    parser.add_argument("--xy-noise-stddev", type=float, default=0.0,
                        help="Per-point Gaussian noise stddev (default: 0).")
    parser.add_argument("--start-time-s", type=float, default=1700000000.0,
                        help="Base ROS time (sec).")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--min-points", type=int, default=30,
                        help="Skip scans with fewer than this many in-range points.")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    return generate_bag(args)


if __name__ == "__main__":
    sys.exit(main())
