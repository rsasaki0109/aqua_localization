#!/usr/bin/env python3
"""Publish a tiny odometry chain plus one loop closure constraint.

Run this alongside:

  ros2 launch aqua_pose_graph pose_graph.launch.py

The pose graph will first receive five odometry poses along x, then a loop
constraint saying keyframe 4 returns to keyframe 0. The published
`/aqua_pose_graph/path` should bend after the constraint is accepted.
"""

import argparse
import math
import time

import rclpy
from aqua_msgs.msg import PoseGraphLoopConstraint
from nav_msgs.msg import Odometry
from rclpy.node import Node


def make_information(
    translation_sigma_m: float,
    rotation_sigma_rad: float,
) -> list[float]:
    info = [0.0] * 36
    trans_info = 1.0 / (translation_sigma_m * translation_sigma_m)
    rot_info = 1.0 / (rotation_sigma_rad * rotation_sigma_rad)
    for i in range(3):
        info[i * 6 + i] = trans_info
    for i in range(3, 6):
        info[i * 6 + i] = rot_info
    return info


class PoseGraphLoopDemo(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pose_graph_loop_demo")
        self.args = args
        self.odom_pub = self.create_publisher(Odometry, args.odometry_topic, 10)
        self.loop_pub = self.create_publisher(
            PoseGraphLoopConstraint, args.loop_constraint_topic, 10
        )

    def publish_odometry(self, index: int) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.child_frame_id = self.args.child_frame_id
        msg.pose.pose.position.x = float(index) * self.args.step_m
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        for i in range(6):
            msg.pose.covariance[i * 6 + i] = self.args.odom_variance
        self.odom_pub.publish(msg)
        self.get_logger().info(
            f"published odometry sample {index}: x={msg.pose.pose.position.x:.2f}"
        )

    def publish_loop(self) -> None:
        msg = PoseGraphLoopConstraint()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.from_id = 0
        msg.to_id = self.args.keyframes - 1
        msg.relative_pose.orientation.w = 1.0
        msg.information = make_information(
            self.args.loop_translation_sigma_m,
            self.args.loop_rotation_sigma_rad,
        )
        msg.optimize_after_insert = True
        self.loop_pub.publish(msg)
        self.get_logger().info(
            f"published loop constraint {msg.from_id} -> {msg.to_id}"
        )

    def run(self) -> None:
        time.sleep(self.args.startup_delay_s)
        for i in range(self.args.keyframes):
            self.publish_odometry(i)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.args.sample_period_s)
        self.publish_loop()
        end_time = time.monotonic() + self.args.hold_s
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--odometry-topic", default="/aqua_imu_loc/odometry")
    parser.add_argument(
        "--loop-constraint-topic", default="/aqua_pose_graph/loop_constraint"
    )
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--child-frame-id", default="base_link")
    parser.add_argument("--keyframes", type=int, default=5)
    parser.add_argument("--step-m", type=float, default=1.0)
    parser.add_argument("--odom-variance", type=float, default=1.0)
    parser.add_argument("--loop-translation-sigma-m", type=float, default=0.05)
    parser.add_argument(
        "--loop-rotation-sigma-rad", type=float, default=math.radians(2.0)
    )
    parser.add_argument("--startup-delay-s", type=float, default=0.5)
    parser.add_argument("--sample-period-s", type=float, default=0.2)
    parser.add_argument("--hold-s", type=float, default=1.0)
    args = parser.parse_args()
    if args.keyframes < 2:
        parser.error("--keyframes must be at least 2")
    if args.loop_translation_sigma_m <= 0.0:
        parser.error("--loop-translation-sigma-m must be positive")
    if args.loop_rotation_sigma_rad <= 0.0:
        parser.error("--loop-rotation-sigma-rad must be positive")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = PoseGraphLoopDemo(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
