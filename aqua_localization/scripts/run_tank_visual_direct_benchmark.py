#!/usr/bin/env python3
"""Run the Tank stereo visual frontend directly from a rosbag2 sqlite file.

This bypasses ``ros2 bag play`` for the camera topics. It is intended for
throughput diagnosis when ROS replay drops or corrupts compressed image delivery
but the bag itself can be deserialized offline.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sqlite3
import sys
import time

import cv2
import numpy as np

import run_tank_visual_benchmark
import stereo_visual_odometry as svo


DEFAULT_LEFT_TOPIC = "/camera/left/image_dehazed/compressed"
DEFAULT_RIGHT_TOPIC = "/camera/right/image_dehazed/compressed"


@dataclass(frozen=True)
class ImageRecord:
    stamp_s: float
    data: bytes


@dataclass(frozen=True)
class DirectVisualResult:
    stereo_pairs: int
    processed_frames: int
    accepted: int
    rejected: int
    decode_failures: int
    warmup_ms: float


def resolve_sqlite_db(bag: Path) -> Path:
    if bag.is_file():
        return bag
    candidates = sorted(bag.glob("*.db3"))
    if len(candidates) != 1:
        raise FileNotFoundError(f"expected exactly one sqlite .db3 in {bag}, found {len(candidates)}")
    return candidates[0]


def read_compressed_image_records(bag: Path, topic: str) -> list[ImageRecord]:
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage

    db_path = resolve_sqlite_db(bag)
    with sqlite3.connect(str(db_path)) as con:
        row = con.execute("select id from topics where name = ?", (topic,)).fetchone()
        if row is None:
            raise ValueError(f"topic not found in {db_path}: {topic}")
        topic_id = int(row[0])
        records = []
        for (raw,) in con.execute(
                "select data from messages where topic_id = ? order by timestamp",
                (topic_id,)):
            msg = deserialize_message(raw, CompressedImage)
            records.append(ImageRecord(svo.stamp_to_seconds(msg.header.stamp), bytes(msg.data)))
    return records


def pair_stereo_records(
        left_records: list[ImageRecord],
        right_records: list[ImageRecord],
        sync_slop_s: float) -> list[tuple[ImageRecord, ImageRecord]]:
    pairs = []
    right_index = 0
    for left in left_records:
        while (
                right_index + 1 < len(right_records)
                and abs(right_records[right_index + 1].stamp_s - left.stamp_s)
                <= abs(right_records[right_index].stamp_s - left.stamp_s)):
            right_index += 1
        if right_index < len(right_records):
            right = right_records[right_index]
            if abs(right.stamp_s - left.stamp_s) <= sync_slop_s:
                pairs.append((left, right))
                right_index += 1
    return pairs


def visual_config_from_args(args):
    camera = svo.StereoCamera(
        fx=args.camera_fx,
        fy=args.camera_fy,
        cx=args.camera_cx,
        cy=args.camera_cy,
        bf=args.camera_bf,
    )
    config = svo.VisualFrontendConfig(
        max_stereo_descriptor_distance=args.max_stereo_descriptor_distance,
        max_temporal_descriptor_distance=args.max_temporal_descriptor_distance,
        translation_scale=args.translation_scale,
    )
    extrinsics = svo.VisualExtrinsics(
        base_from_camera_x_m=args.base_from_camera_x_m,
        base_from_camera_y_m=args.base_from_camera_y_m,
        base_from_camera_z_m=args.base_from_camera_z_m,
        base_from_camera_roll_rad=args.base_from_camera_roll_rad,
        base_from_camera_pitch_rad=args.base_from_camera_pitch_rad,
        base_from_camera_yaw_rad=args.base_from_camera_yaw_rad,
    )
    return camera, config, extrinsics


def format_tum_pose_line(stamp_s: float, pose: np.ndarray) -> str:
    position = pose[:3, 3]
    qx, qy, qz, qw = svo.rotation_matrix_to_quaternion(pose[:3, :3])
    return (
        f"{stamp_s:.9f} {position[0]:.9f} {position[1]:.9f} {position[2]:.9f} "
        f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
    )


def process_stereo_pairs_direct(
        args,
        pairs: list[tuple[ImageRecord, ImageRecord]],
        estimate_tum: Path,
        status_csv: Path) -> DirectVisualResult:
    camera, config, extrinsics = visual_config_from_args(args)
    if args.opencv_threads > 0:
        cv2.setNumThreads(args.opencv_threads)
    orb = cv2.ORB_create(nfeatures=args.orb_n_features, fastThreshold=args.orb_fast_threshold)
    stereo_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    temporal_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    warmup_ms = svo.warm_up_feature_pipeline(orb, stereo_matcher) if args.warmup else 0.0

    base_from_camera = svo.base_from_camera_transform(extrinsics)
    publish_base_pose = args.publish_base_pose or not extrinsics.is_identity()
    world_from_camera = np.eye(4, dtype=np.float64)
    previous_frame = None
    processed = 0
    accepted = 0
    rejected = 0
    decode_failures = 0

    estimate_tum.parent.mkdir(parents=True, exist_ok=True)
    status_csv.parent.mkdir(parents=True, exist_ok=True)
    with estimate_tum.open("w", encoding="utf-8") as tum_fp, \
            status_csv.open("w", newline="", encoding="utf-8") as status_fp:
        svo.write_status_csv_header(status_fp)
        for left_record, right_record in pairs:
            total_start = time.perf_counter()
            try:
                decode_start = time.perf_counter()
                left = svo.decode_compressed_image(left_record.data)
                right = svo.decode_compressed_image(right_record.data)
                decode_time_ms = (time.perf_counter() - decode_start) * 1000.0
            except ValueError:
                decode_failures += 1
                continue

            stereo_start = time.perf_counter()
            frame = svo.triangulate_stereo_features(
                left, right, camera, config, orb, stereo_matcher
            )
            stereo_time_ms = (time.perf_counter() - stereo_start) * 1000.0
            frame.stamp_s = left_record.stamp_s
            processed += 1

            if previous_frame is None:
                estimate = svo.MotionEstimate(
                    True,
                    np.eye(3, dtype=np.float64),
                    np.zeros(3, dtype=np.float64),
                    0,
                    0,
                    "initialized",
                )
                previous_frame = frame
                publish_pose = svo.world_from_base_pose(
                    world_from_camera, base_from_camera, publish_base_pose
                )
                tum_fp.write(format_tum_pose_line(frame.stamp_s, publish_pose))
                processing_times = svo.ProcessingTimes(
                    decode_time_ms=decode_time_ms,
                    stereo_time_ms=stereo_time_ms,
                    tracking_time_ms=0.0,
                    total_time_ms=(time.perf_counter() - total_start) * 1000.0,
                )
            else:
                tracking_start = time.perf_counter()
                estimate = svo.estimate_motion_pnp(
                    previous_frame, frame, camera, config, temporal_matcher
                )
                tracking_time_ms = (time.perf_counter() - tracking_start) * 1000.0
                if estimate.success:
                    world_from_camera = svo.update_world_pose(
                        world_from_camera,
                        estimate.rotation_prev_to_curr,
                        estimate.translation_prev_to_curr,
                    )
                    accepted += 1
                    publish_pose = svo.world_from_base_pose(
                        world_from_camera, base_from_camera, publish_base_pose
                    )
                    tum_fp.write(format_tum_pose_line(frame.stamp_s, publish_pose))
                else:
                    rejected += 1
                previous_frame = frame
                processing_times = svo.ProcessingTimes(
                    decode_time_ms=decode_time_ms,
                    stereo_time_ms=stereo_time_ms,
                    tracking_time_ms=tracking_time_ms,
                    total_time_ms=(time.perf_counter() - total_start) * 1000.0,
                )

            status = svo.make_status(
                frame.stamp_s,
                processed,
                accepted,
                rejected,
                frame,
                estimate,
                processing_times,
            )
            svo.write_status_csv_row(status_fp, status)
            status_fp.flush()

    return DirectVisualResult(
        stereo_pairs=len(pairs),
        processed_frames=processed,
        accepted=accepted,
        rejected=rejected,
        decode_failures=decode_failures,
        warmup_ms=warmup_ms,
    )


def run_direct(args, paths: run_tank_visual_benchmark.BenchmarkPaths) -> DirectVisualResult:
    left_records = read_compressed_image_records(args.bag, args.left_topic)
    right_records = read_compressed_image_records(args.bag, args.right_topic)
    pairs = pair_stereo_records(left_records, right_records, args.sync_slop_s)
    return process_stereo_pairs_direct(args, pairs, paths.estimate_tum, paths.status_csv)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Run Tank stereo visual odometry directly from rosbag2 sqlite camera messages."
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_visual_direct"))
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--sequence", default="short_test")
    parser.add_argument("--system", default="aqua_visual_frontend_direct")
    parser.add_argument("--left-topic", default=DEFAULT_LEFT_TOPIC)
    parser.add_argument("--right-topic", default=DEFAULT_RIGHT_TOPIC)
    parser.add_argument("--sync-slop-s", type=float, default=0.02)
    parser.add_argument("--translation-scale", type=float, default=1.0)
    parser.add_argument("--drift-window-s", type=float, default=3.0)
    parser.add_argument("--drift-stride-s", type=float, default=1.0)
    parser.add_argument("--drift-min-samples", type=int, default=20)
    parser.add_argument("--segment-s", type=float, default=1.0)
    parser.add_argument("--segment-stride-s", type=float, default=0.5)
    parser.add_argument("--segment-min-reference-motion-m", type=float, default=0.01)
    parser.add_argument("--camera-fx", type=float, default=run_tank_visual_benchmark.DEFAULT_FX)
    parser.add_argument("--camera-fy", type=float, default=run_tank_visual_benchmark.DEFAULT_FY)
    parser.add_argument("--camera-cx", type=float, default=run_tank_visual_benchmark.DEFAULT_CX)
    parser.add_argument("--camera-cy", type=float, default=run_tank_visual_benchmark.DEFAULT_CY)
    parser.add_argument("--camera-bf", type=float, default=run_tank_visual_benchmark.DEFAULT_BF)
    parser.add_argument("--base-from-camera-x-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-y-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-z-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-roll-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-pitch-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-yaw-rad", type=float, default=0.0)
    parser.add_argument("--publish-base-pose", action="store_true")
    parser.add_argument("--max-stereo-descriptor-distance", type=float, default=96.0)
    parser.add_argument("--max-temporal-descriptor-distance", type=float, default=96.0)
    parser.add_argument("--orb-n-features", type=int, default=1000)
    parser.add_argument("--orb-fast-threshold", type=int, default=12)
    parser.add_argument("--opencv-threads", type=int, default=0)
    parser.add_argument("--no-warmup", dest="warmup", action="store_false")
    parser.set_defaults(warmup=True)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.translation_scale <= 0.0:
        raise ValueError("--translation-scale must be positive")
    if args.sync_slop_s < 0.0:
        raise ValueError("--sync-slop-s must be non-negative")
    if args.orb_n_features <= 0:
        raise ValueError("--orb-n-features must be positive")
    if args.orb_fast_threshold < 0:
        raise ValueError("--orb-fast-threshold must be non-negative")
    if args.opencv_threads < 0:
        raise ValueError("--opencv-threads must be non-negative")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    paths = run_tank_visual_benchmark.default_paths(args.out_dir, args.sequence)
    args.status_csv = paths.status_csv
    result = run_direct(args, paths)
    print(
        "direct visual replay: "
        f"pairs={result.stereo_pairs} processed={result.processed_frames} "
        f"accepted={result.accepted} rejected={result.rejected} "
        f"decode_failures={result.decode_failures} warmup_ms={result.warmup_ms:.1f}"
    )
    print(run_tank_visual_benchmark.evaluate(args, paths.estimate_tum, paths))
    return 0


if __name__ == "__main__":
    sys.exit(main())
