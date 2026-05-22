#!/usr/bin/env python3
"""Create and read reusable Tank DVL motion-prior profiles."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import yaml


PROFILE_KEYS = {
    "dvl_yaw_mode": ("prior", "dvl_yaw_mode"),
    "dvl_frame_yaw_offset_deg": ("prior", "dvl_frame_yaw_offset_deg"),
    "imu_yaw_offset_deg": ("prior", "imu_yaw_offset_deg"),
    "prior_scale": ("prior", "prior_scale"),
    "mode": ("application", "mode"),
    "blend_alpha": ("application", "blend_alpha"),
    "min_prior_step_m": ("application", "min_prior_step_m"),
    "min_length_ratio": ("application", "min_length_ratio"),
    "max_length_ratio": ("application", "max_length_ratio"),
    "min_direction_cosine": ("application", "min_direction_cosine"),
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
    for arg_name, yaml_path in PROFILE_KEYS.items():
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
        validation_sequence = metadata.get("validation_sequence")
        if calibration_sequence and validation_sequence:
            return f"{calibration_sequence}_to_{validation_sequence}"
        if calibration_sequence:
            return str(calibration_sequence)
    return path.stem


def build_profile(args) -> dict:
    return {
        "format_version": 1,
        "name": args.name,
        "metadata": {
            "dataset": args.dataset,
            "calibration_sequence": args.calibration_sequence,
            "validation_sequence": args.validation_sequence,
            "calibration_bag": str(args.calibration_bag) if args.calibration_bag else "",
            "calibration_reference": str(args.calibration_reference) if args.calibration_reference else "",
            "calibration_visual": str(args.calibration_visual) if args.calibration_visual else "",
            "note": args.note,
        },
        "prior": {
            "dvl_yaw_mode": args.dvl_yaw_mode,
            "dvl_frame_yaw_offset_deg": float(args.dvl_frame_yaw_offset_deg),
            "imu_yaw_offset_deg": float(args.imu_yaw_offset_deg),
            "prior_scale": float(args.prior_scale),
        },
        "application": {
            "mode": args.mode,
            "blend_alpha": float(args.blend_alpha),
            "min_prior_step_m": float(args.min_prior_step_m),
            "min_length_ratio": float(args.min_length_ratio),
            "max_length_ratio": float(args.max_length_ratio),
            "min_direction_cosine": float(args.min_direction_cosine),
        },
    }


def write_profile(path: Path, profile: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(profile, sort_keys=False, default_flow_style=False)
    path.write_text(text, encoding="utf-8")


def format_summary(path: Path, profile: dict) -> str:
    prior = profile["prior"]
    application = profile["application"]
    metadata = profile["metadata"]
    return "\n".join([
        f"wrote Tank DVL prior profile: {path}",
        f"name: {profile['name']}",
        f"calibration_sequence: {metadata['calibration_sequence']}",
        f"validation_sequence: {metadata['validation_sequence']}",
        (
            "prior: "
            f"mode={prior['dvl_yaw_mode']}, "
            f"dvl_yaw_offset={prior['dvl_frame_yaw_offset_deg']:g} deg, "
            f"imu_yaw_offset={prior['imu_yaw_offset_deg']:g} deg, "
            f"scale={prior['prior_scale']:g}"
        ),
        (
            "application: "
            f"mode={application['mode']}, "
            f"blend_alpha={application['blend_alpha']:g}, "
            f"length_ratio=[{application['min_length_ratio']:g},"
            f"{application['max_length_ratio']:g}], "
            f"min_direction_cosine={application['min_direction_cosine']:g}"
        ),
    ])


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Write a reusable YAML profile for Tank DVL motion-prior calibration."
    )
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--name", default="tank_dvl_prior")
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--calibration-sequence", default="calibration")
    parser.add_argument("--validation-sequence", default="")
    parser.add_argument("--calibration-bag", type=Path)
    parser.add_argument("--calibration-reference", type=Path)
    parser.add_argument("--calibration-visual", type=Path)
    parser.add_argument("--note", default="")
    parser.add_argument("--dvl-yaw-mode", choices=["body_raw", "gt_yaw", "imu_yaw"], default="imu_yaw")
    parser.add_argument("--dvl-frame-yaw-offset-deg", type=float, default=-90.0)
    parser.add_argument("--imu-yaw-offset-deg", type=float, default=115.0)
    parser.add_argument("--prior-scale", type=float, default=1.25375)
    parser.add_argument(
        "--mode",
        choices=["replace-outliers", "blend-outliers", "blend-all"],
        default="replace-outliers",
    )
    parser.add_argument("--blend-alpha", type=float, default=0.5)
    parser.add_argument("--min-prior-step-m", type=float, default=1.0e-4)
    parser.add_argument("--min-length-ratio", type=float, default=0.5)
    parser.add_argument("--max-length-ratio", type=float, default=1.5)
    parser.add_argument("--min-direction-cosine", type=float, default=0.5)
    return parser.parse_args(argv)


def validate_args(args) -> None:
    if args.prior_scale <= 0.0:
        raise ValueError("--prior-scale must be positive")
    if not 0.0 <= args.blend_alpha <= 1.0:
        raise ValueError("--blend-alpha must be in [0, 1]")
    if args.min_prior_step_m < 0.0:
        raise ValueError("--min-prior-step-m must be non-negative")
    if args.min_length_ratio < 0.0:
        raise ValueError("--min-length-ratio must be non-negative")
    if args.max_length_ratio < args.min_length_ratio:
        raise ValueError("--max-length-ratio must be >= --min-length-ratio")
    if not -1.0 <= args.min_direction_cosine <= 1.0:
        raise ValueError("--min-direction-cosine must be in [-1, 1]")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    validate_args(args)
    profile = build_profile(args)
    write_profile(args.out, profile)
    print(format_summary(args.out, profile))
    return 0


if __name__ == "__main__":
    sys.exit(main())
