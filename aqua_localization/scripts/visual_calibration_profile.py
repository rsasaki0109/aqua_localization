#!/usr/bin/env python3
"""Create and read visual frontend calibration profiles.

The profile is intentionally small YAML: it records the visual scale,
camera-to-base extrinsic hypothesis, and throughput-related frontend settings
that should be reused on a held-out evaluation sequence.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml

import calibrate_visual_scale


EXTRINSIC_KEYS = {
    "base_from_camera_x_m": ("base_from_camera", "x_m"),
    "base_from_camera_y_m": ("base_from_camera", "y_m"),
    "base_from_camera_z_m": ("base_from_camera", "z_m"),
    "base_from_camera_roll_rad": ("base_from_camera", "roll_rad"),
    "base_from_camera_pitch_rad": ("base_from_camera", "pitch_rad"),
    "base_from_camera_yaw_rad": ("base_from_camera", "yaw_rad"),
}

FRONTEND_KEYS = {
    "translation_scale": ("frontend", "translation_scale"),
    "max_stereo_descriptor_distance": ("frontend", "max_stereo_descriptor_distance"),
    "max_temporal_descriptor_distance": ("frontend", "max_temporal_descriptor_distance"),
    "orb_n_features": ("frontend", "orb_n_features"),
    "orb_fast_threshold": ("frontend", "orb_fast_threshold"),
    "opencv_threads": ("frontend", "opencv_threads"),
}

FUSION_KEYS = {
    "visual_position_variance_floor": ("fusion", "visual_position_variance_floor"),
}


def nested_get(data: dict, path: tuple[str, str], default=None):
    section, key = path
    section_value = data.get(section, {})
    if not isinstance(section_value, dict):
        return default
    return section_value.get(key, default)


def load_profile(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fp:
        data = yaml.safe_load(fp) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a YAML mapping")
    return data


def profile_arg_defaults(path: Path) -> dict:
    profile = load_profile(path)
    defaults = {}
    for arg_name, yaml_path in {
        **FRONTEND_KEYS,
        **EXTRINSIC_KEYS,
        **FUSION_KEYS,
    }.items():
        value = nested_get(profile, yaml_path)
        if value is not None:
            defaults[arg_name] = value
    return defaults


def profile_label(path: Path, profile: dict) -> str:
    name = profile.get("name")
    if name:
        return str(name)
    metadata = profile.get("metadata", {})
    if isinstance(metadata, dict):
        calibration_sequence = metadata.get("calibration_sequence")
        if calibration_sequence:
            return str(calibration_sequence)
    return path.stem


def build_profile(args) -> dict:
    if args.translation_scale is None:
        if args.calibration_reference is None or args.calibration_estimate is None:
            raise ValueError(
                "provide --translation-scale or both --calibration-reference and --calibration-estimate"
            )
        calibration = calibrate_visual_scale.estimate_scale(
            args.calibration_reference,
            args.calibration_estimate,
            args.calibration_current_scale,
        )
        translation_scale = calibration["recommended_translation_scale"]
    else:
        if args.translation_scale <= 0.0:
            raise ValueError("--translation-scale must be positive")
        translation_scale = args.translation_scale
        calibration = None

    profile = {
        "format_version": 1,
        "name": args.name,
        "metadata": {
            "dataset": args.dataset,
            "calibration_sequence": args.calibration_sequence,
            "validation_sequence": args.validation_sequence,
            "note": args.note,
        },
        "frontend": {
            "translation_scale": float(translation_scale),
            "max_stereo_descriptor_distance": float(args.max_stereo_descriptor_distance),
            "max_temporal_descriptor_distance": float(args.max_temporal_descriptor_distance),
            "orb_n_features": int(args.orb_n_features),
            "orb_fast_threshold": int(args.orb_fast_threshold),
            "opencv_threads": int(args.opencv_threads),
        },
        "base_from_camera": {
            "x_m": float(args.base_from_camera_x_m),
            "y_m": float(args.base_from_camera_y_m),
            "z_m": float(args.base_from_camera_z_m),
            "roll_rad": float(args.base_from_camera_roll_rad),
            "pitch_rad": float(args.base_from_camera_pitch_rad),
            "yaw_rad": float(args.base_from_camera_yaw_rad),
        },
        "fusion": {
            "visual_position_variance_floor": float(args.visual_position_variance_floor),
        },
    }
    if calibration is not None:
        profile["scale_calibration"] = {
            "reference": str(args.calibration_reference),
            "estimate": str(args.calibration_estimate),
            "current_scale": float(args.calibration_current_scale),
            "matched_samples": int(calibration["matched_samples"]),
            "matched_seconds": float(calibration["matched_seconds"]),
            "sim3_alignment_scale": float(calibration["sim3_alignment_scale"]),
            "recommended_translation_scale": float(calibration["recommended_translation_scale"]),
            "se3_rmse": float(calibration["se3_rmse"]),
            "sim3_rmse": float(calibration["sim3_rmse"]),
        }
    return profile


def write_profile(path: Path, profile: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(profile, sort_keys=False, default_flow_style=False)
    path.write_text(text, encoding="utf-8")


def format_summary(path: Path, profile: dict) -> str:
    frontend = profile["frontend"]
    extrinsics = profile["base_from_camera"]
    fusion = profile["fusion"]
    lines = [
        f"wrote visual calibration profile: {path}",
        f"tracking.translation_scale: {frontend['translation_scale']:.9f}",
        (
            "base_from_camera: "
            f"({extrinsics['x_m']:g},{extrinsics['y_m']:g},{extrinsics['z_m']:g}) m"
        ),
        (
            "frontend: "
            f"stereo_dist={frontend['max_stereo_descriptor_distance']:g}, "
            f"temporal_dist={frontend['max_temporal_descriptor_distance']:g}, "
            f"orb_n_features={frontend['orb_n_features']}, "
            f"orb_fast_threshold={frontend['orb_fast_threshold']}, "
            f"opencv_threads={frontend['opencv_threads']}"
        ),
        f"visual_position_variance_floor: {fusion['visual_position_variance_floor']:g}",
    ]
    if "scale_calibration" in profile:
        cal = profile["scale_calibration"]
        lines.append(
            f"scale calibration: {cal['matched_samples']} samples, "
            f"{cal['matched_seconds']:.2f} s, Sim(3) RMSE {cal['sim3_rmse']:.4f} m"
        )
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Write a reusable YAML profile for visual frontend calibration."
    )
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--name", default="visual_calibration")
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--calibration-sequence", default="calibration")
    parser.add_argument("--validation-sequence", default="")
    parser.add_argument("--note", default="")
    parser.add_argument("--calibration-reference", type=Path)
    parser.add_argument("--calibration-estimate", type=Path)
    parser.add_argument("--calibration-current-scale", type=float, default=1.0)
    parser.add_argument("--translation-scale", type=float, default=None)
    parser.add_argument("--base-from-camera-x-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-y-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-z-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-roll-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-pitch-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-yaw-rad", type=float, default=0.0)
    parser.add_argument("--max-stereo-descriptor-distance", type=float, default=64.0)
    parser.add_argument("--max-temporal-descriptor-distance", type=float, default=64.0)
    parser.add_argument("--orb-n-features", type=int, default=700)
    parser.add_argument("--orb-fast-threshold", type=int, default=16)
    parser.add_argument("--opencv-threads", type=int, default=2)
    parser.add_argument("--visual-position-variance-floor", type=float, default=0.01)
    return parser.parse_args(argv)


def validate_args(args):
    if args.calibration_current_scale <= 0.0:
        raise ValueError("--calibration-current-scale must be positive")
    if args.orb_n_features <= 0:
        raise ValueError("--orb-n-features must be positive")
    if args.orb_fast_threshold < 0:
        raise ValueError("--orb-fast-threshold must be non-negative")
    if args.opencv_threads < 0:
        raise ValueError("--opencv-threads must be non-negative")
    if args.visual_position_variance_floor <= 0.0:
        raise ValueError("--visual-position-variance-floor must be positive")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    validate_args(args)
    profile = build_profile(args)
    write_profile(args.out, profile)
    print(format_summary(args.out, profile))
    return 0


if __name__ == "__main__":
    sys.exit(main())
