#!/usr/bin/env python3
"""Subscribe to aqua_msgs/EstimatorStatus and append each message to a CSV file.

The CSV columns include the bias states and the AHRS-hook activity flags so users can
plot bias convergence and verify that the AHRS observations are firing on a given bag.
"""

import argparse
import sys
from pathlib import Path

import rclpy
from aqua_msgs.msg import EstimatorStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


CSV_HEADER = (
    "timestamp,frame_id,estimator_name,backend,initialized,update_count,"
    "last_prediction_dt,position_covariance_trace,orientation_covariance_trace,status,"
    "accel_bias_x,accel_bias_y,accel_bias_z,"
    "gyro_bias_x,gyro_bias_y,gyro_bias_z,"
    "ahrs_gyro_bias_z_enabled,ahrs_gyro_bias_z_active,ahrs_gyro_bias_z_last_observed\n"
)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def format_csv_line(msg: EstimatorStatus) -> str:
    t = stamp_to_seconds(msg.header.stamp)
    return (
        f"{t:.9f},{msg.header.frame_id},{msg.estimator_name},{msg.backend},"
        f"{int(msg.initialized)},{msg.update_count},"
        f"{msg.last_prediction_dt:.9f},"
        f"{msg.position_covariance_trace:.9f},{msg.orientation_covariance_trace:.9f},"
        f"{msg.status},"
        f"{msg.accel_bias[0]:.9f},{msg.accel_bias[1]:.9f},{msg.accel_bias[2]:.9f},"
        f"{msg.gyro_bias[0]:.9f},{msg.gyro_bias[1]:.9f},{msg.gyro_bias[2]:.9f},"
        f"{int(msg.ahrs_gyro_bias_z_enabled)},{int(msg.ahrs_gyro_bias_z_active)},"
        f"{msg.ahrs_gyro_bias_z_last_observed:.9f}\n"
    )


class StatusRecorderNode(Node):
    def __init__(self, topic: str, output: Path, reliable: bool):
        super().__init__("aqua_status_recorder")
        self.count = 0
        self.output = output
        self.fp = output.open("w", encoding="utf-8")
        self.fp.write(CSV_HEADER)
        self.fp.flush()

        qos = QoSProfile(depth=200)
        qos.reliability = (
            ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
        )
        self.sub = self.create_subscription(EstimatorStatus, topic, self.on_status, qos)
        self.get_logger().info(
            f"recording {topic} -> {output} as csv (reliability={'reliable' if reliable else 'best_effort'})"
        )

    def on_status(self, msg: EstimatorStatus) -> None:
        self.fp.write(format_csv_line(msg))
        self.count += 1
        if self.count % 100 == 0:
            self.fp.flush()

    def destroy_node(self) -> bool:
        try:
            self.fp.flush()
            self.fp.close()
        finally:
            self.get_logger().info(f"wrote {self.count} status samples to {self.output}")
        return super().destroy_node()


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Record aqua_msgs/EstimatorStatus messages to a CSV file."
    )
    parser.add_argument(
        "--topic", default="/aqua_imu_loc/status",
        help="EstimatorStatus topic (default: /aqua_imu_loc/status).",
    )
    parser.add_argument("--out", required=True, type=Path,
                        help="Output CSV path. Parent directories are created.")
    parser.add_argument("--reliable", action="store_true",
                        help="Use reliable reliability instead of best-effort.")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    args.out.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = StatusRecorderNode(args.topic, args.out, args.reliable)
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
