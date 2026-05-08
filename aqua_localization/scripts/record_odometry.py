#!/usr/bin/env python3
"""Subscribe to a nav_msgs/Odometry topic and append each message to a TUM or CSV file.

The TUM output is directly comparable with `fjord_1_baseline.tum` and other tools that
read the standard `timestamp tx ty tz qx qy qz qw` format (rpg_trajectory_evaluation,
evo, etc.). The CSV output also includes per-axis position covariance and twist for
quick plotting.
"""

import argparse
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


CSV_HEADER = (
    "timestamp,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw,"
    "vx,vy,vz,wx,wy,wz,cov_xx,cov_yy,cov_zz\n"
)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def format_tum_line(msg: Odometry) -> str:
    t = stamp_to_seconds(msg.header.stamp)
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"


def format_csv_line(msg: Odometry) -> str:
    t = stamp_to_seconds(msg.header.stamp)
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular
    cov = msg.pose.covariance
    return (
        f"{t:.9f},{msg.header.frame_id},{msg.child_frame_id},"
        f"{p.x:.9f},{p.y:.9f},{p.z:.9f},"
        f"{q.x:.9f},{q.y:.9f},{q.z:.9f},{q.w:.9f},"
        f"{v.x:.9f},{v.y:.9f},{v.z:.9f},"
        f"{w.x:.9f},{w.y:.9f},{w.z:.9f},"
        f"{cov[0]:.9f},{cov[7]:.9f},{cov[14]:.9f}\n"
    )


class OdometryRecorderNode(Node):
    def __init__(self, topic: str, output: Path, fmt: str, reliable: bool):
        super().__init__("aqua_odometry_recorder")
        self.fmt = fmt
        self.count = 0
        self.output = output
        self.fp = output.open("w", encoding="utf-8")
        if fmt == "csv":
            self.fp.write(CSV_HEADER)
            self.fp.flush()

        qos = QoSProfile(depth=200)
        qos.reliability = (
            ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
        )
        self.sub = self.create_subscription(Odometry, topic, self.on_odom, qos)
        self.get_logger().info(
            f"recording {topic} -> {output} as {fmt} (reliability={'reliable' if reliable else 'best_effort'})"
        )

    def on_odom(self, msg: Odometry) -> None:
        line = format_tum_line(msg) if self.fmt == "tum" else format_csv_line(msg)
        self.fp.write(line)
        self.count += 1
        if self.count % 200 == 0:
            self.fp.flush()

    def destroy_node(self) -> bool:
        try:
            self.fp.flush()
            self.fp.close()
        finally:
            self.get_logger().info(f"wrote {self.count} odometry samples to {self.output}")
        return super().destroy_node()


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Record nav_msgs/Odometry to a TUM or CSV file for offline plotting."
    )
    parser.add_argument(
        "--topic",
        default="/aqua_imu_loc/odometry",
        help="Odometry topic to subscribe to (default: /aqua_imu_loc/odometry).",
    )
    parser.add_argument(
        "--out",
        required=True,
        type=Path,
        help="Output file path. Suffix is not used to infer format; pass --format.",
    )
    parser.add_argument(
        "--format",
        choices=["tum", "csv"],
        default="tum",
        help="Output format. tum: timestamp tx ty tz qx qy qz qw. csv: extended.",
    )
    parser.add_argument(
        "--reliable",
        action="store_true",
        help="Use reliable reliability instead of best-effort.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    args.out.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = OdometryRecorderNode(args.topic, args.out, args.format, args.reliable)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
