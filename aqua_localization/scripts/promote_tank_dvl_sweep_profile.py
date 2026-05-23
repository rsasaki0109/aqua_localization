#!/usr/bin/env python3
"""Promote one Tank DVL prior gate-sweep row into a reusable profile."""

from __future__ import annotations

import argparse
import csv
from copy import deepcopy
from pathlib import Path
import sys

import tank_dvl_prior_profile


SWEEP_TO_PROFILE_KEYS = {
    "mode": ("application", "mode", str),
    "blend_alpha": ("application", "blend_alpha", float),
    "prior_scale": ("prior", "prior_scale", float),
    "min_length_ratio": ("application", "min_length_ratio", float),
    "max_length_ratio": ("application", "max_length_ratio", float),
    "min_direction_cosine": ("application", "min_direction_cosine", float),
}


def read_sweep_rows(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as fp:
        rows = list(csv.DictReader(fp))
    if not rows:
        raise ValueError(f"{path}: no sweep rows")
    return rows


def row_rank(row: dict) -> int:
    try:
        return int(row["rank"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("sweep CSV must contain integer rank column") from exc


def select_sweep_row(rows: list[dict], rank: int) -> dict:
    for row in rows:
        if row_rank(row) == rank:
            return row
    available = ", ".join(str(row_rank(row)) for row in rows[:10])
    raise ValueError(f"rank {rank} not found in sweep CSV; available ranks start with: {available}")


def set_nested(data: dict, section: str, key: str, value) -> None:
    section_data = data.setdefault(section, {})
    if not isinstance(section_data, dict):
        raise ValueError(f"profile section {section!r} must be a mapping")
    section_data[key] = value


def promote_profile(base_profile: dict, sweep_row: dict, args) -> dict:
    profile = deepcopy(base_profile)
    if args.name:
        profile["name"] = args.name
    else:
        profile["name"] = f"{tank_dvl_prior_profile.profile_label(args.base_profile, base_profile)}_rank{args.rank}"

    metadata = profile.setdefault("metadata", {})
    if not isinstance(metadata, dict):
        raise ValueError("profile metadata must be a mapping")
    metadata["note"] = args.note or (
        f"promoted from Tank DVL gate sweep rank {args.rank}; "
        f"source RMSE={float(sweep_row['corrected_rmse_m']):.4f} m"
    )
    metadata["source_sweep_csv"] = str(args.sweep_csv)
    metadata["source_sweep_rank"] = int(args.rank)
    metadata["source_sweep_corrected_rmse_m"] = float(sweep_row["corrected_rmse_m"])
    if sweep_row.get("gap_to_baseline_x"):
        metadata["source_sweep_gap_to_baseline_x"] = float(sweep_row["gap_to_baseline_x"])
    if args.validation_sequence:
        metadata["validation_sequence"] = args.validation_sequence

    for sweep_key, (section, profile_key, caster) in SWEEP_TO_PROFILE_KEYS.items():
        if sweep_key not in sweep_row or sweep_row[sweep_key] == "":
            raise ValueError(f"sweep CSV missing required column {sweep_key!r}")
        set_nested(profile, section, profile_key, caster(sweep_row[sweep_key]))
    return profile


def format_summary(path: Path, profile: dict, row: dict) -> str:
    prior = profile["prior"]
    application = profile["application"]
    metadata = profile["metadata"]
    return "\n".join([
        f"wrote promoted Tank DVL prior profile: {path}",
        f"name: {profile['name']}",
        f"calibration_sequence: {metadata.get('calibration_sequence', '')}",
        f"validation_sequence: {metadata.get('validation_sequence', '')}",
        f"source_sweep_rank: {metadata['source_sweep_rank']}",
        f"source_sweep_rmse: {float(row['corrected_rmse_m']):.4f} m",
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
        description="Promote a Tank DVL gate-sweep row into a reusable profile."
    )
    parser.add_argument("--base-profile", required=True, type=Path)
    parser.add_argument("--sweep-csv", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--rank", type=int, default=1)
    parser.add_argument("--name", default="")
    parser.add_argument("--validation-sequence", default="")
    parser.add_argument("--note", default="")
    return parser.parse_args(argv)


def validate_args(args) -> None:
    if args.rank <= 0:
        raise ValueError("--rank must be positive")


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        validate_args(args)
        base_profile = tank_dvl_prior_profile.load_profile(args.base_profile)
        row = select_sweep_row(read_sweep_rows(args.sweep_csv), args.rank)
        profile = promote_profile(base_profile, row, args)
        tank_dvl_prior_profile.write_profile(args.out, profile)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(format_summary(args.out, profile, row))
    return 0


if __name__ == "__main__":
    sys.exit(main())
