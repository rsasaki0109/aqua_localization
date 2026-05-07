#!/usr/bin/env python3
"""Compare gyro-integrated yaw with AHRS-quaternion yaw delta from a rosbag2.

For each `sensor_msgs/Imu` message in the chosen topic this tool:

- integrates `angular_velocity.z` over time to produce a body-frame gyro yaw
- extracts yaw from `orientation` and tracks unwrapped delta from the first sample

Both signals are written to a CSV with columns:

    t,gyro_yaw_rad,ahrs_yaw_delta_rad,diff_rad

A summary line on stdout reports the slope (least-squares) of `ahrs_yaw_delta` against
`gyro_yaw`. Slope ~+1 means the two body-frame yaw conventions agree. Slope ~-1 means
the AHRS publishes yaw with the opposite sign (e.g. NED vs ENU body frame). Anything
else points at a scale issue, bias, or unrelated frame.

This is pure numpy + rosbags; no rclpy.
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return the yaw component of a quaternion in radians (ZYX Euler convention)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_yaw_delta(prev_unwrapped: float, prev_raw: float, current_raw: float) -> float:
    """Continue an unwrapped yaw track given previous wrapped/unwrapped pair."""
    step = normalize_angle(current_raw - prev_raw)
    return prev_unwrapped + step


def least_squares_slope(x: np.ndarray, y: np.ndarray) -> float:
    """Return slope of y ≈ slope * x (origin-anchored)."""
    denom = float((x * x).sum())
    if denom <= 0.0:
        return float("nan")
    return float((x * y).sum() / denom)


def diagnose(
    bag_path: Path,
    imu_topic: str,
    csv_out: Path,
    sample_limit: int,
    gyro_bias_rad_s: float,
):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    typestore = get_typestore(Stores.ROS2_JAZZY)

    rows = []
    integrated_gyro_yaw = 0.0
    last_t = None
    initial_ahrs_yaw = None
    last_ahrs_yaw_raw = None
    unwrapped_ahrs_yaw_delta = 0.0

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == imu_topic]
        if not connections:
            available = sorted({c.topic for c in reader.connections})
            raise SystemExit(
                f"topic {imu_topic} not found in {bag_path}. "
                f"available: {available}"
            )
        for conn, t_ns, raw in reader.messages(connections=connections):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            t = t_ns * 1.0e-9

            gyro_z = float(msg.angular_velocity.z) - gyro_bias_rad_s
            if last_t is not None:
                dt = t - last_t
                if 0.0 < dt < 0.5:
                    integrated_gyro_yaw += gyro_z * dt

            ahrs_yaw_raw = quaternion_to_yaw(
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
            )
            if initial_ahrs_yaw is None:
                initial_ahrs_yaw = ahrs_yaw_raw
                last_ahrs_yaw_raw = ahrs_yaw_raw
                unwrapped_ahrs_yaw_delta = 0.0
            else:
                unwrapped_ahrs_yaw_delta = unwrap_yaw_delta(
                    unwrapped_ahrs_yaw_delta, last_ahrs_yaw_raw, ahrs_yaw_raw
                )
                last_ahrs_yaw_raw = ahrs_yaw_raw

            rows.append(
                (t, integrated_gyro_yaw, unwrapped_ahrs_yaw_delta,
                 unwrapped_ahrs_yaw_delta - integrated_gyro_yaw)
            )
            last_t = t

            if sample_limit and len(rows) >= sample_limit:
                break

    if not rows:
        raise SystemExit(f"no messages on {imu_topic}")

    csv_out.parent.mkdir(parents=True, exist_ok=True)
    with csv_out.open("w", encoding="utf-8") as fp:
        fp.write("t,gyro_yaw_rad,ahrs_yaw_delta_rad,diff_rad\n")
        for row in rows:
            fp.write(",".join(f"{v:.9f}" for v in row) + "\n")

    arr = np.asarray(rows, dtype=np.float64)
    gyro_track = arr[:, 1] - arr[0, 1]
    ahrs_track = arr[:, 2] - arr[0, 2]
    diff = arr[:, 3] - arr[0, 3]
    slope = least_squares_slope(gyro_track, ahrs_track)
    end_gyro = float(gyro_track[-1])
    end_ahrs = float(ahrs_track[-1])
    end_diff = float(diff[-1])
    rmse_diff = float(np.sqrt(np.mean(diff ** 2)))
    return {
        "samples": int(arr.shape[0]),
        "duration_s": float(arr[-1, 0] - arr[0, 0]),
        "end_gyro_yaw_rad": end_gyro,
        "end_ahrs_yaw_delta_rad": end_ahrs,
        "end_diff_rad": end_diff,
        "rmse_diff_rad": rmse_diff,
        "slope_ahrs_over_gyro": slope,
        "csv_out": str(csv_out),
    }


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Compare gyro-integrated yaw and AHRS yaw delta in a rosbag2."
    )
    parser.add_argument("bag_path", type=Path)
    parser.add_argument(
        "--imu-topic", default="/mavros/imu/data",
        help="Topic that publishes sensor_msgs/Imu with both gyro and orientation.",
    )
    parser.add_argument(
        "--csv-out", type=Path, default=Path("/tmp/diagnose_yaw_frame.csv"),
        help="CSV output path.",
    )
    parser.add_argument(
        "--sample-limit", type=int, default=0,
        help="Stop after this many messages (0 = all).",
    )
    parser.add_argument(
        "--gyro-bias-rad-s", type=float, default=0.0,
        help="Subtract this from angular_velocity.z before integration.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    summary = diagnose(
        args.bag_path, args.imu_topic, args.csv_out, args.sample_limit, args.gyro_bias_rad_s
    )
    print(f"samples:                  {summary['samples']}")
    print(f"duration:                 {summary['duration_s']:.2f} s")
    print(f"final gyro-integrated yaw {summary['end_gyro_yaw_rad']:+.4f} rad")
    print(f"final AHRS yaw delta:     {summary['end_ahrs_yaw_delta_rad']:+.4f} rad")
    print(f"final diff (ahrs - gyro): {summary['end_diff_rad']:+.4f} rad")
    print(f"RMSE of diff over time:   {summary['rmse_diff_rad']:.4f} rad")
    print(f"slope ahrs vs gyro:       {summary['slope_ahrs_over_gyro']:+.6f}")
    print(f"csv written to:           {summary['csv_out']}")
    print()
    print("Interpretation:")
    print("  slope ~+1.00 => same body-frame yaw convention")
    print("  slope ~-1.00 => opposite sign (NED vs ENU body, or autopilot-vs-ros frame)")
    print("  slope close to 0 or far from +/-1 => unrelated frames or large bias")
    return 0


if __name__ == "__main__":
    sys.exit(main())
