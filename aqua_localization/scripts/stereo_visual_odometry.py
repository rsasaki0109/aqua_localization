#!/usr/bin/env python3
"""Lightweight stereo visual odometry frontend for underwater replay bags.

The node is intentionally small and dependency-light: it consumes synchronized
left/right `sensor_msgs/CompressedImage` frames, triangulates ORB features from
stereo disparity, estimates frame-to-frame motion with PnP/RANSAC, and publishes
a `nav_msgs/Odometry` trajectory on `/aqua_visual_frontend/odometry`.

It is a visual frontend, not a full SLAM system: there is no loop closure, local
bundle adjustment, relocalization, or IMU coupling. The output is useful as the
first ROS 2 camera-based baseline and as an input to future fusion work.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class StereoCamera:
    fx: float
    fy: float
    cx: float
    cy: float
    bf: float

    @property
    def matrix(self) -> np.ndarray:
        return np.asarray(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )


@dataclass(frozen=True)
class VisualFrontendConfig:
    max_stereo_y_diff_px: float = 2.0
    max_stereo_descriptor_distance: float = 96.0
    min_disparity_px: float = 1.0
    max_depth_m: float = 12.0
    max_temporal_descriptor_distance: float = 96.0
    min_temporal_matches: int = 20
    min_pnp_inliers: int = 12
    min_inlier_ratio: float = 0.25
    ransac_iterations: int = 100
    ransac_reprojection_error_px: float = 3.0
    ransac_confidence: float = 0.99
    max_step_translation_m: float = 2.0
    translation_scale: float = 1.0
    position_variance_floor_m2: float = 0.04
    rotation_variance_floor_rad2: float = 0.05


@dataclass(frozen=True)
class VisualExtrinsics:
    base_from_camera_x_m: float = 0.0
    base_from_camera_y_m: float = 0.0
    base_from_camera_z_m: float = 0.0
    base_from_camera_roll_rad: float = 0.0
    base_from_camera_pitch_rad: float = 0.0
    base_from_camera_yaw_rad: float = 0.0

    def is_identity(self) -> bool:
        values = (
            self.base_from_camera_x_m,
            self.base_from_camera_y_m,
            self.base_from_camera_z_m,
            self.base_from_camera_roll_rad,
            self.base_from_camera_pitch_rad,
            self.base_from_camera_yaw_rad,
        )
        return all(abs(value) < 1.0e-12 for value in values)


@dataclass
class VisualFrame:
    stamp_s: float
    points3d: np.ndarray
    keypoints_xy: np.ndarray
    descriptors: np.ndarray
    disparities_px: np.ndarray
    left_features: int = 0
    right_features: int = 0
    stereo_matches: int = 0


@dataclass
class MotionEstimate:
    success: bool
    rotation_prev_to_curr: np.ndarray
    translation_prev_to_curr: np.ndarray
    inliers: int
    matches: int
    reason: str = ""


@dataclass(frozen=True)
class VisualFrontendStatus:
    stamp_s: float
    right_stamp_s: float
    stereo_sync_delta_ms: float
    frame_index: int
    accepted_count: int
    rejected_count: int
    left_features: int
    right_features: int
    stereo_matches: int
    stereo_points: int
    disparity_min_px: float
    disparity_median_px: float
    disparity_p95_px: float
    depth_min_m: float
    depth_median_m: float
    depth_p95_m: float
    temporal_matches: int
    pnp_inliers: int
    inlier_ratio: float
    step_translation_m: float
    decode_time_ms: float
    stereo_time_ms: float
    tracking_time_ms: float
    total_time_ms: float
    accepted: bool
    status: str


STATUS_CSV_FIELDS = [
    "timestamp",
    "right_timestamp",
    "stereo_sync_delta_ms",
    "frame_index",
    "accepted_count",
    "rejected_count",
    "left_features",
    "right_features",
    "stereo_matches",
    "stereo_points",
    "disparity_min_px",
    "disparity_median_px",
    "disparity_p95_px",
    "depth_min_m",
    "depth_median_m",
    "depth_p95_m",
    "temporal_matches",
    "pnp_inliers",
    "inlier_ratio",
    "step_translation_m",
    "decode_time_ms",
    "stereo_time_ms",
    "tracking_time_ms",
    "total_time_ms",
    "accepted",
    "status",
]


@dataclass(frozen=True)
class ProcessingTimes:
    decode_time_ms: float = 0.0
    stereo_time_ms: float = 0.0
    tracking_time_ms: float = 0.0
    total_time_ms: float = 0.0


def decode_compressed_image(data: bytes) -> np.ndarray:
    encoded = np.frombuffer(data, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError("failed to decode compressed image")
    return image


def filter_descriptor_matches(matches: Iterable[cv2.DMatch], max_distance: float) -> list[cv2.DMatch]:
    if max_distance <= 0.0:
        return list(matches)
    return [match for match in matches if float(match.distance) <= max_distance]


def triangulate_stereo_features(
    left_image: np.ndarray,
    right_image: np.ndarray,
    camera: StereoCamera,
    config: VisualFrontendConfig,
    orb: cv2.ORB,
    matcher: cv2.BFMatcher,
) -> VisualFrame:
    left_keypoints, left_desc = orb.detectAndCompute(left_image, None)
    right_keypoints, right_desc = orb.detectAndCompute(right_image, None)
    left_count = len(left_keypoints)
    right_count = len(right_keypoints)
    if left_desc is None or right_desc is None:
        return VisualFrame(
            0.0,
            empty_points3d(),
            empty_keypoints(),
            empty_descriptors(),
            empty_disparities(),
            left_features=left_count,
            right_features=right_count,
        )

    raw_matches = matcher.match(left_desc, right_desc)
    matches = filter_descriptor_matches(raw_matches, config.max_stereo_descriptor_distance)
    points = []
    keypoints = []
    descriptors = []
    disparities = []
    for match in matches:
        left_pt = left_keypoints[match.queryIdx].pt
        right_pt = right_keypoints[match.trainIdx].pt
        if abs(left_pt[1] - right_pt[1]) > config.max_stereo_y_diff_px:
            continue
        disparity = left_pt[0] - right_pt[0]
        if disparity < config.min_disparity_px:
            continue
        z = camera.bf / disparity
        if not math.isfinite(z) or z <= 0.0 or z > config.max_depth_m:
            continue
        x = (left_pt[0] - camera.cx) * z / camera.fx
        y = (left_pt[1] - camera.cy) * z / camera.fy
        points.append((x, y, z))
        keypoints.append(left_pt)
        descriptors.append(left_desc[match.queryIdx])
        disparities.append(disparity)

    if not points:
        return VisualFrame(
            0.0,
            empty_points3d(),
            empty_keypoints(),
            empty_descriptors(),
            empty_disparities(),
            left_features=left_count,
            right_features=right_count,
            stereo_matches=len(matches),
        )
    return VisualFrame(
        0.0,
        np.asarray(points, dtype=np.float32),
        np.asarray(keypoints, dtype=np.float32),
        np.asarray(descriptors, dtype=np.uint8),
        np.asarray(disparities, dtype=np.float32),
        left_features=left_count,
        right_features=right_count,
        stereo_matches=len(matches),
    )


def estimate_motion_pnp(
    previous: VisualFrame,
    current: VisualFrame,
    camera: StereoCamera,
    config: VisualFrontendConfig,
    matcher: cv2.BFMatcher,
) -> MotionEstimate:
    if previous.descriptors.shape[0] < config.min_temporal_matches:
        return failed_motion("not enough previous stereo features")
    if current.descriptors.shape[0] < config.min_temporal_matches:
        return failed_motion("not enough current stereo features")

    raw_matches = matcher.match(previous.descriptors, current.descriptors)
    matches = filter_descriptor_matches(raw_matches, config.max_temporal_descriptor_distance)
    if len(matches) < config.min_temporal_matches:
        return failed_motion("not enough temporal matches", matches=len(matches))

    object_points = np.asarray(
        [previous.points3d[match.queryIdx] for match in matches], dtype=np.float32
    )
    image_points = np.asarray(
        [current.keypoints_xy[match.trainIdx] for match in matches], dtype=np.float32
    )
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        camera.matrix,
        np.zeros((4, 1), dtype=np.float64),
        iterationsCount=config.ransac_iterations,
        reprojectionError=config.ransac_reprojection_error_px,
        confidence=config.ransac_confidence,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok or inliers is None:
        return failed_motion("pnp failed", matches=len(matches))

    inlier_count = int(inliers.shape[0])
    if inlier_count < config.min_pnp_inliers:
        return failed_motion("too few pnp inliers", matches=len(matches), inliers=inlier_count)
    inlier_ratio = inlier_count / max(1, len(matches))
    if inlier_ratio < config.min_inlier_ratio:
        return failed_motion("low pnp inlier ratio", matches=len(matches), inliers=inlier_count)

    rotation, _ = cv2.Rodrigues(rvec)
    translation = tvec.reshape(3).astype(np.float64) * config.translation_scale
    if float(np.linalg.norm(translation)) > config.max_step_translation_m:
        return failed_motion("translation gate rejected step", matches=len(matches), inliers=inlier_count)

    return MotionEstimate(True, rotation, translation, inlier_count, len(matches))


def update_world_pose(
    world_from_previous: np.ndarray,
    rotation_prev_to_curr: np.ndarray,
    translation_prev_to_curr: np.ndarray,
) -> np.ndarray:
    """Compose a camera pose from a PnP transform.

    OpenCV PnP returns X_curr = R * X_prev + t. The published pose is
    world_from_camera, so the frame increment is the inverse transform.
    """
    curr_from_prev = np.eye(4, dtype=np.float64)
    curr_from_prev[:3, :3] = rotation_prev_to_curr
    curr_from_prev[:3, 3] = translation_prev_to_curr
    prev_from_curr = np.linalg.inv(curr_from_prev)
    return world_from_previous @ prev_from_curr


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    rot_x = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rot_y = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rot_z = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rot_z @ rot_y @ rot_x


def base_from_camera_transform(extrinsics: VisualExtrinsics) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rpy_to_matrix(
        extrinsics.base_from_camera_roll_rad,
        extrinsics.base_from_camera_pitch_rad,
        extrinsics.base_from_camera_yaw_rad,
    )
    transform[:3, 3] = [
        extrinsics.base_from_camera_x_m,
        extrinsics.base_from_camera_y_m,
        extrinsics.base_from_camera_z_m,
    ]
    return transform


def world_from_base_pose(
    world_from_camera: np.ndarray,
    base_from_camera: np.ndarray,
    publish_base_pose: bool,
) -> np.ndarray:
    if not publish_base_pose:
        return world_from_camera
    return world_from_camera @ np.linalg.inv(base_from_camera)


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


def covariance_diagonal(config: VisualFrontendConfig, inliers: int) -> list[float]:
    inliers = max(1, int(inliers))
    pos_var = max(config.position_variance_floor_m2, 2.0 / inliers)
    rot_var = max(config.rotation_variance_floor_rad2, 1.0 / inliers)
    return [pos_var, pos_var, pos_var, rot_var, rot_var, rot_var]


def empty_points3d() -> np.ndarray:
    return np.empty((0, 3), dtype=np.float32)


def empty_keypoints() -> np.ndarray:
    return np.empty((0, 2), dtype=np.float32)


def empty_descriptors() -> np.ndarray:
    return np.empty((0, 32), dtype=np.uint8)


def empty_disparities() -> np.ndarray:
    return np.empty((0,), dtype=np.float32)


def make_warmup_image(width: int = 640, height: int = 512) -> np.ndarray:
    image = np.zeros((height, width), dtype=np.uint8)
    for y in range(0, height, 32):
        color = 80 if (y // 32) % 2 == 0 else 180
        image[y:y + 16, :] = color
    for x in range(0, width, 32):
        image[:, x:x + 16] = np.maximum(image[:, x:x + 16], 120)
    cv2.circle(image, (width // 3, height // 3), 70, 220, 3)
    cv2.rectangle(image, (width // 2, height // 2), (width - 80, height - 60), 200, 2)
    return image


def warm_up_feature_pipeline(orb: cv2.ORB, matcher: cv2.BFMatcher, iterations: int = 3) -> float:
    left = make_warmup_image()
    right = np.roll(left, shift=-4, axis=1)
    start = time.perf_counter()
    for _ in range(max(1, iterations)):
        left_keypoints, left_desc = orb.detectAndCompute(left, None)
        right_keypoints, right_desc = orb.detectAndCompute(right, None)
        if left_desc is not None and right_desc is not None and left_keypoints and right_keypoints:
            matcher.match(left_desc, right_desc)
    return (time.perf_counter() - start) * 1000.0


def failed_motion(reason: str, matches: int = 0, inliers: int = 0) -> MotionEstimate:
    return MotionEstimate(
        False,
        np.eye(3, dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        inliers,
        matches,
        reason,
    )


def finite_percentile(values: np.ndarray, q: float) -> float:
    finite = np.asarray(values, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if finite.size == 0:
        return math.nan
    return float(np.percentile(finite, q))


def make_status(
    stamp_s: float,
    right_stamp_s: float,
    frame_index: int,
    accepted_count: int,
    rejected_count: int,
    frame: VisualFrame,
    estimate: MotionEstimate,
    processing_times: ProcessingTimes = ProcessingTimes(),
) -> VisualFrontendStatus:
    inlier_ratio = estimate.inliers / max(1, estimate.matches)
    step_translation_m = float(np.linalg.norm(estimate.translation_prev_to_curr))
    depths = frame.points3d[:, 2] if frame.points3d.size else np.asarray([], dtype=np.float32)
    return VisualFrontendStatus(
        stamp_s=stamp_s,
        right_stamp_s=right_stamp_s,
        stereo_sync_delta_ms=abs(right_stamp_s - stamp_s) * 1000.0,
        frame_index=frame_index,
        accepted_count=accepted_count,
        rejected_count=rejected_count,
        left_features=frame.left_features,
        right_features=frame.right_features,
        stereo_matches=frame.stereo_matches,
        stereo_points=int(frame.points3d.shape[0]),
        disparity_min_px=finite_percentile(frame.disparities_px, 0.0),
        disparity_median_px=finite_percentile(frame.disparities_px, 50.0),
        disparity_p95_px=finite_percentile(frame.disparities_px, 95.0),
        depth_min_m=finite_percentile(depths, 0.0),
        depth_median_m=finite_percentile(depths, 50.0),
        depth_p95_m=finite_percentile(depths, 95.0),
        temporal_matches=estimate.matches,
        pnp_inliers=estimate.inliers,
        inlier_ratio=float(inlier_ratio),
        step_translation_m=step_translation_m,
        decode_time_ms=processing_times.decode_time_ms,
        stereo_time_ms=processing_times.stereo_time_ms,
        tracking_time_ms=processing_times.tracking_time_ms,
        total_time_ms=processing_times.total_time_ms,
        accepted=estimate.success,
        status=estimate.reason if estimate.reason else "accepted",
    )


def status_to_dict(status: VisualFrontendStatus) -> dict:
    return {
        "timestamp": status.stamp_s,
        "right_timestamp": status.right_stamp_s,
        "stereo_sync_delta_ms": status.stereo_sync_delta_ms,
        "frame_index": status.frame_index,
        "accepted_count": status.accepted_count,
        "rejected_count": status.rejected_count,
        "left_features": status.left_features,
        "right_features": status.right_features,
        "stereo_matches": status.stereo_matches,
        "stereo_points": status.stereo_points,
        "disparity_min_px": status.disparity_min_px,
        "disparity_median_px": status.disparity_median_px,
        "disparity_p95_px": status.disparity_p95_px,
        "depth_min_m": status.depth_min_m,
        "depth_median_m": status.depth_median_m,
        "depth_p95_m": status.depth_p95_m,
        "temporal_matches": status.temporal_matches,
        "pnp_inliers": status.pnp_inliers,
        "inlier_ratio": status.inlier_ratio,
        "step_translation_m": status.step_translation_m,
        "decode_time_ms": status.decode_time_ms,
        "stereo_time_ms": status.stereo_time_ms,
        "tracking_time_ms": status.tracking_time_ms,
        "total_time_ms": status.total_time_ms,
        "accepted": status.accepted,
        "status": status.status,
    }


def status_to_json(status: VisualFrontendStatus) -> str:
    return json.dumps(status_to_dict(status), sort_keys=True, separators=(",", ":"))


def write_status_csv_header(fp):
    writer = csv.DictWriter(fp, fieldnames=STATUS_CSV_FIELDS)
    writer.writeheader()


def write_status_csv_row(fp, status: VisualFrontendStatus):
    row = status_to_dict(status)
    row["timestamp"] = f"{status.stamp_s:.9f}"
    row["right_timestamp"] = f"{status.right_stamp_s:.9f}"
    row["stereo_sync_delta_ms"] = f"{status.stereo_sync_delta_ms:.6f}"
    row["inlier_ratio"] = f"{status.inlier_ratio:.9f}"
    row["step_translation_m"] = f"{status.step_translation_m:.9f}"
    for field in (
        "decode_time_ms",
        "stereo_time_ms",
        "tracking_time_ms",
        "total_time_ms",
    ):
        row[field] = f"{float(row[field]):.3f}"
    for field in (
        "disparity_min_px",
        "disparity_median_px",
        "disparity_p95_px",
        "depth_min_m",
        "depth_median_m",
        "depth_p95_m",
    ):
        value = row[field]
        row[field] = f"{value:.9f}" if math.isfinite(float(value)) else ""
    row["accepted"] = int(status.accepted)
    writer = csv.DictWriter(fp, fieldnames=STATUS_CSV_FIELDS)
    writer.writerow(row)


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def run_ros_node(argv=None) -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    class StereoVisualOdometryNode(Node):
        def __init__(self):
            super().__init__("aqua_stereo_visual_odometry")
            self.left_topic = self.declare_parameter(
                "topics.left_image", "/camera/left/image_dehazed/compressed"
            ).value
            self.right_topic = self.declare_parameter(
                "topics.right_image", "/camera/right/image_dehazed/compressed"
            ).value
            self.odom_topic = self.declare_parameter(
                "topics.odometry", "/aqua_visual_frontend/odometry"
            ).value
            self.status_topic = self.declare_parameter(
                "topics.status", "/aqua_visual_frontend/status"
            ).value
            self.frame_id = self.declare_parameter("frames.odom", "visual_odom").value
            self.camera_frame_id = self.declare_parameter("frames.camera", "camera_left").value
            self.base_frame_id = self.declare_parameter("frames.base", "base_link").value
            self.sync_slop_s = float(self.declare_parameter("sync.slop_s", 0.02).value)
            self.buffer_size = int(self.declare_parameter("sync.buffer_size", 20).value)
            self.status_csv_path = str(
                self.declare_parameter("diagnostics.status_csv_path", "").value
            )
            self.extrinsics = VisualExtrinsics(
                base_from_camera_x_m=float(
                    self.declare_parameter("extrinsics.base_from_camera.x_m", 0.0).value
                ),
                base_from_camera_y_m=float(
                    self.declare_parameter("extrinsics.base_from_camera.y_m", 0.0).value
                ),
                base_from_camera_z_m=float(
                    self.declare_parameter("extrinsics.base_from_camera.z_m", 0.0).value
                ),
                base_from_camera_roll_rad=float(
                    self.declare_parameter("extrinsics.base_from_camera.roll_rad", 0.0).value
                ),
                base_from_camera_pitch_rad=float(
                    self.declare_parameter("extrinsics.base_from_camera.pitch_rad", 0.0).value
                ),
                base_from_camera_yaw_rad=float(
                    self.declare_parameter("extrinsics.base_from_camera.yaw_rad", 0.0).value
                ),
            )
            self.publish_base_pose = bool(
                self.declare_parameter("extrinsics.publish_base_pose", False).value
            ) or not self.extrinsics.is_identity()
            self.child_frame_id = self.base_frame_id if self.publish_base_pose else self.camera_frame_id
            self.base_from_camera = base_from_camera_transform(self.extrinsics)

            self.camera = StereoCamera(
                fx=float(self.declare_parameter("camera.fx", 655.0).value),
                fy=float(self.declare_parameter("camera.fy", 655.0).value),
                cx=float(self.declare_parameter("camera.cx", 306.0).value),
                cy=float(self.declare_parameter("camera.cy", 256.0).value),
                bf=float(self.declare_parameter("camera.bf", 78.89165891925023).value),
            )
            self.config = VisualFrontendConfig(
                max_stereo_y_diff_px=float(
                    self.declare_parameter("stereo.max_y_diff_px", 2.0).value
                ),
                max_stereo_descriptor_distance=float(
                    self.declare_parameter("matching.max_stereo_descriptor_distance", 96.0).value
                ),
                min_disparity_px=float(self.declare_parameter("stereo.min_disparity_px", 1.0).value),
                max_depth_m=float(self.declare_parameter("stereo.max_depth_m", 12.0).value),
                max_temporal_descriptor_distance=float(
                    self.declare_parameter("matching.max_temporal_descriptor_distance", 96.0).value
                ),
                min_temporal_matches=int(
                    self.declare_parameter("tracking.min_temporal_matches", 20).value
                ),
                min_pnp_inliers=int(self.declare_parameter("tracking.min_pnp_inliers", 12).value),
                min_inlier_ratio=float(
                    self.declare_parameter("tracking.min_inlier_ratio", 0.25).value
                ),
                ransac_iterations=int(
                    self.declare_parameter("tracking.ransac_iterations", 100).value
                ),
                ransac_reprojection_error_px=float(
                    self.declare_parameter("tracking.ransac_reprojection_error_px", 3.0).value
                ),
                ransac_confidence=float(
                    self.declare_parameter("tracking.ransac_confidence", 0.99).value
                ),
                max_step_translation_m=float(
                    self.declare_parameter("tracking.max_step_translation_m", 2.0).value
                ),
                translation_scale=float(
                    self.declare_parameter("tracking.translation_scale", 1.0).value
                ),
                position_variance_floor_m2=float(
                    self.declare_parameter("covariance.position_floor_m2", 0.04).value
                ),
                rotation_variance_floor_rad2=float(
                    self.declare_parameter("covariance.rotation_floor_rad2", 0.05).value
                ),
            )

            nfeatures = int(self.declare_parameter("orb.n_features", 1000).value)
            fast_threshold = int(self.declare_parameter("orb.fast_threshold", 12).value)
            opencv_threads = int(self.declare_parameter("opencv.threads", 0).value)
            if opencv_threads > 0:
                cv2.setNumThreads(opencv_threads)
            self.orb = cv2.ORB_create(nfeatures=nfeatures, fastThreshold=fast_threshold)
            self.stereo_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            self.temporal_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            warmup_enabled = bool(self.declare_parameter("diagnostics.warmup", True).value)
            warmup_time_ms = 0.0
            if warmup_enabled:
                warmup_time_ms = warm_up_feature_pipeline(self.orb, self.stereo_matcher)

            qos = QoSProfile(depth=20)
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.left_sub = self.create_subscription(
                CompressedImage, self.left_topic, self.on_left, qos
            )
            self.right_sub = self.create_subscription(
                CompressedImage, self.right_topic, self.on_right, qos
            )
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
            self.status_pub = self.create_publisher(String, self.status_topic, 10)
            self.status_csv_fp = None
            if self.status_csv_path:
                path = Path(self.status_csv_path)
                path.parent.mkdir(parents=True, exist_ok=True)
                self.status_csv_fp = path.open("w", newline="", encoding="utf-8")
                write_status_csv_header(self.status_csv_fp)
                self.status_csv_fp.flush()

            self.left_buffer = []
            self.right_buffer = []
            self.previous_frame: Optional[VisualFrame] = None
            self.world_from_camera = np.eye(4, dtype=np.float64)
            self.frames = 0
            self.accepted = 0
            self.rejected = 0
            self.get_logger().info(
                f"stereo visual odometry started: left={self.left_topic} right={self.right_topic} "
                f"odom={self.odom_topic} status={self.status_topic} child={self.child_frame_id} "
                f"orb_n_features={nfeatures} orb_fast_threshold={fast_threshold} "
                f"opencv_threads={cv2.getNumThreads()} warmup_ms={warmup_time_ms:.1f}"
            )

        def on_left(self, msg):
            self.left_buffer.append(msg)
            self.left_buffer = self.left_buffer[-self.buffer_size :]
            self.try_process()

        def on_right(self, msg):
            self.right_buffer.append(msg)
            self.right_buffer = self.right_buffer[-self.buffer_size :]
            self.try_process()

        def try_process(self):
            if not self.left_buffer or not self.right_buffer:
                return
            left_msg = self.left_buffer[-1]
            left_t = stamp_to_seconds(left_msg.header.stamp)
            right_msg = min(
                self.right_buffer,
                key=lambda msg: abs(stamp_to_seconds(msg.header.stamp) - left_t),
            )
            right_t = stamp_to_seconds(right_msg.header.stamp)
            if abs(right_t - left_t) > self.sync_slop_s:
                return
            self.left_buffer.clear()
            self.right_buffer = [msg for msg in self.right_buffer if msg is not right_msg]
            self.process_pair(left_msg, right_msg)

        def process_pair(self, left_msg, right_msg):
            total_start = time.perf_counter()
            try:
                decode_start = time.perf_counter()
                left = decode_compressed_image(bytes(left_msg.data))
                right = decode_compressed_image(bytes(right_msg.data))
                decode_time_ms = (time.perf_counter() - decode_start) * 1000.0
            except ValueError as exc:
                self.get_logger().warn(str(exc))
                return

            stereo_start = time.perf_counter()
            frame = triangulate_stereo_features(
                left, right, self.camera, self.config, self.orb, self.stereo_matcher
            )
            stereo_time_ms = (time.perf_counter() - stereo_start) * 1000.0
            frame.stamp_s = stamp_to_seconds(left_msg.header.stamp)
            self.frames += 1

            if self.previous_frame is None:
                total_time_ms = (time.perf_counter() - total_start) * 1000.0
                processing_times = ProcessingTimes(
                    decode_time_ms=decode_time_ms,
                    stereo_time_ms=stereo_time_ms,
                    tracking_time_ms=0.0,
                    total_time_ms=total_time_ms,
                )
                self.previous_frame = frame
                self.publish_odometry(left_msg.header.stamp, 0, 0)
                init_estimate = MotionEstimate(
                    True,
                    np.eye(3, dtype=np.float64),
                    np.zeros(3, dtype=np.float64),
                    0,
                    0,
                    "initialized",
                )
                self.publish_status(
                    frame,
                    init_estimate,
                    processing_times,
                    stamp_to_seconds(right_msg.header.stamp),
                )
                return

            tracking_start = time.perf_counter()
            estimate = estimate_motion_pnp(
                self.previous_frame, frame, self.camera, self.config, self.temporal_matcher
            )
            tracking_time_ms = (time.perf_counter() - tracking_start) * 1000.0
            if estimate.success:
                self.world_from_camera = update_world_pose(
                    self.world_from_camera,
                    estimate.rotation_prev_to_curr,
                    estimate.translation_prev_to_curr,
                )
                self.previous_frame = frame
                self.accepted += 1
                self.publish_odometry(left_msg.header.stamp, estimate.inliers, estimate.matches)
            else:
                self.rejected += 1
                self.previous_frame = frame
                self.get_logger().warn(
                    f"visual odometry rejected frame {self.frames}: {estimate.reason}",
                    throttle_duration_sec=2.0,
                )
            total_time_ms = (time.perf_counter() - total_start) * 1000.0
            self.publish_status(
                frame,
                estimate,
                ProcessingTimes(
                    decode_time_ms=decode_time_ms,
                    stereo_time_ms=stereo_time_ms,
                    tracking_time_ms=tracking_time_ms,
                    total_time_ms=total_time_ms,
                ),
                stamp_to_seconds(right_msg.header.stamp),
            )

        def publish_odometry(self, stamp, inliers: int, matches: int):
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.child_frame_id = self.child_frame_id
            publish_pose = world_from_base_pose(
                self.world_from_camera,
                self.base_from_camera,
                self.publish_base_pose,
            )
            position = publish_pose[:3, 3]
            msg.pose.pose.position.x = float(position[0])
            msg.pose.pose.position.y = float(position[1])
            msg.pose.pose.position.z = float(position[2])
            qx, qy, qz, qw = rotation_matrix_to_quaternion(publish_pose[:3, :3])
            msg.pose.pose.orientation.x = qx
            msg.pose.pose.orientation.y = qy
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            diag = covariance_diagonal(self.config, inliers)
            for i, value in enumerate(diag):
                msg.pose.covariance[i * 6 + i] = float(value)
            if matches > 0:
                msg.twist.covariance[0] = float(matches)
                msg.twist.covariance[1] = float(inliers)
            self.odom_pub.publish(msg)

        def publish_status(
            self,
            frame: VisualFrame,
            estimate: MotionEstimate,
            processing_times: ProcessingTimes,
            right_stamp_s: float,
        ):
            status = make_status(
                frame.stamp_s,
                right_stamp_s,
                self.frames,
                self.accepted,
                self.rejected,
                frame,
                estimate,
                processing_times,
            )
            msg = String()
            msg.data = status_to_json(status)
            self.status_pub.publish(msg)
            if self.status_csv_fp is not None:
                write_status_csv_row(self.status_csv_fp, status)
                self.status_csv_fp.flush()

        def destroy_node(self) -> bool:
            try:
                if self.status_csv_fp is not None:
                    self.status_csv_fp.flush()
                    self.status_csv_fp.close()
                    self.status_csv_fp = None
            finally:
                return super().destroy_node()

    rclpy.init(args=argv)
    node = StereoVisualOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.get_logger().info(
                f"visual odometry stopped: frames={node.frames} "
                f"accepted={node.accepted} rejected={node.rejected}"
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Run the stereo visual odometry ROS 2 node.")
    return parser.parse_known_args(argv)[0]


def main(argv=None) -> int:
    parse_args(argv if argv is not None else sys.argv[1:])
    return run_ros_node(argv)


if __name__ == "__main__":
    sys.exit(main())
