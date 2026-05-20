#!/usr/bin/env python3
"""Estimate a stereo visual-odometry translation scale from TUM trajectories.

This is intended for `stereo_visual_odometry.py`: run the visual frontend on a
calibration sequence, record `/aqua_visual_frontend/odometry` to TUM, and align
it against ground truth. The Sim(3) scale from Umeyama is the multiplier to apply
to the frontend's `tracking.translation_scale` parameter for a later validation
sequence.
"""

import argparse
import sys
from pathlib import Path

import compare_trajectories


def estimate_scale(reference_path: Path, estimate_path: Path, current_scale: float = 1.0) -> dict:
    if current_scale <= 0.0:
        raise ValueError("--current-scale must be positive")
    se3_stats, _ = compare_trajectories.compare(
        reference_path, estimate_path, with_scale=False, no_align=False)
    sim3_stats, _ = compare_trajectories.compare(
        reference_path, estimate_path, with_scale=True, no_align=False)
    sim3_scale = float(sim3_stats["alignment"]["scale"])
    return {
        "matched_samples": int(sim3_stats["count"]),
        "matched_seconds": float(sim3_stats["matched_seconds"]),
        "current_scale": float(current_scale),
        "sim3_alignment_scale": sim3_scale,
        "recommended_translation_scale": float(current_scale * sim3_scale),
        "se3_rmse": float(se3_stats["rmse"]),
        "sim3_rmse": float(sim3_stats["rmse"]),
    }


def format_report(result: dict, ros_args: bool) -> str:
    lines = [
        f"matched samples: {result['matched_samples']}",
        f"matched duration: {result['matched_seconds']:.2f} s",
        f"current tracking.translation_scale: {result['current_scale']:.9f}",
        f"Sim(3) alignment scale: {result['sim3_alignment_scale']:.9f}",
        f"recommended tracking.translation_scale: {result['recommended_translation_scale']:.9f}",
        f"SE(3) RMSE before scale fit: {result['se3_rmse']:.4f} m",
        f"Sim(3) RMSE after scale fit: {result['sim3_rmse']:.4f} m",
    ]
    if ros_args:
        lines.append(
            f"-p tracking.translation_scale:={result['recommended_translation_scale']:.9f}")
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Estimate tracking.translation_scale for stereo_visual_odometry.py.")
    parser.add_argument("reference", type=Path, help="Reference TUM trajectory.")
    parser.add_argument("estimate", type=Path, help="Visual odometry TUM trajectory.")
    parser.add_argument(
        "--current-scale",
        type=float,
        default=1.0,
        help="Scale used when the estimate was recorded. Default 1.0.",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=20,
        help="Fail if fewer matched samples are available.",
    )
    parser.add_argument(
        "--ros-args",
        action="store_true",
        help="Also print a copy-pastable ROS parameter override line.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    result = estimate_scale(args.reference, args.estimate, args.current_scale)
    if result["matched_samples"] < args.min_samples:
        raise ValueError(
            f"only {result['matched_samples']} matched samples; need at least {args.min_samples}")
    print(format_report(result, args.ros_args))
    return 0


if __name__ == "__main__":
    sys.exit(main())
