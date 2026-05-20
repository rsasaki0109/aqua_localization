#!/usr/bin/env python3
"""Sweep camera-to-base extrinsic hypotheses for a visual TUM trajectory.

The stereo frontend publishes the left-camera pose. For fusion and fair
body-frame comparison we often need the base pose:

    T_world_base = T_world_camera * inverse(T_base_camera)

This tool applies that transform to an existing TUM trajectory and evaluates
each candidate against a reference trajectory without replaying a ROS bag.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import itertools
import math
from pathlib import Path
import sys

import numpy as np

import compare_trajectories
import run_tank_visual_benchmark


@dataclass(frozen=True)
class ExtrinsicCandidate:
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float

    @property
    def label(self) -> str:
        parts = [
            ("x", self.x_m),
            ("y", self.y_m),
            ("z", self.z_m),
            ("r", math.degrees(self.roll_rad)),
            ("p", math.degrees(self.pitch_rad)),
            ("yw", math.degrees(self.yaw_rad)),
        ]
        text = "__".join(f"{name}_{format_label_number(value)}" for name, value in parts)
        return run_tank_visual_benchmark.sanitize_name(text)


@dataclass(frozen=True)
class ExtrinsicResult:
    candidate: ExtrinsicCandidate
    transformed_tum: Path
    rmse_m: float
    matched_seconds: float
    count: int
    mean_m: float
    median_m: float
    max_m: float


def parse_float_list(value: str) -> list[float]:
    values = [float(part.strip()) for part in value.split(",") if part.strip()]
    if not values:
        raise ValueError("candidate list is empty")
    return values


def parse_degrees_list(value: str) -> list[float]:
    return [math.radians(v) for v in parse_float_list(value)]


def format_label_number(value: float) -> str:
    if abs(value) < 1.0e-12:
        value = 0.0
    text = f"{value:g}".replace("-", "m").replace(".", "p")
    return text


def format_float(value: float, precision: int = 4) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise ValueError("zero-norm quaternion")
    x, y, z, w = q / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
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


def transform_from_xyz_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rpy_to_matrix(roll, pitch, yaw)
    transform[:3, 3] = [x, y, z]
    return transform


def transform_from_tum_row(row: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_matrix(row[4], row[5], row[6], row[7])
    transform[:3, 3] = row[1:4]
    return transform


def tum_row_from_transform(stamp_s: float, transform: np.ndarray) -> list[float]:
    qx, qy, qz, qw = matrix_to_quaternion(transform[:3, :3])
    x, y, z = transform[:3, 3]
    return [stamp_s, x, y, z, qx, qy, qz, qw]


def transform_camera_to_base(traj: np.ndarray, candidate: ExtrinsicCandidate) -> np.ndarray:
    base_from_camera = transform_from_xyz_rpy(
        candidate.x_m,
        candidate.y_m,
        candidate.z_m,
        candidate.roll_rad,
        candidate.pitch_rad,
        candidate.yaw_rad,
    )
    camera_from_base = np.linalg.inv(base_from_camera)
    rows = []
    for row in traj:
        world_from_camera = transform_from_tum_row(row)
        world_from_base = world_from_camera @ camera_from_base
        rows.append(tum_row_from_transform(float(row[0]), world_from_base))
    return np.asarray(rows, dtype=np.float64)


def write_tum(path: Path, traj: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fp:
        for row in traj:
            fp.write(
                f"{row[0]:.9f} {row[1]:.9f} {row[2]:.9f} {row[3]:.9f} "
                f"{row[4]:.9f} {row[5]:.9f} {row[6]:.9f} {row[7]:.9f}\n"
            )


def build_candidates(args) -> list[ExtrinsicCandidate]:
    return [
        ExtrinsicCandidate(x, y, z, roll, pitch, yaw)
        for x, y, z, roll, pitch, yaw in itertools.product(
            parse_float_list(args.x_m),
            parse_float_list(args.y_m),
            parse_float_list(args.z_m),
            parse_degrees_list(args.roll_deg),
            parse_degrees_list(args.pitch_deg),
            parse_degrees_list(args.yaw_deg),
        )
    ]


def evaluate_candidate(args, source: np.ndarray, candidate: ExtrinsicCandidate) -> ExtrinsicResult:
    transformed = transform_camera_to_base(source, candidate)
    path = args.out_dir / f"{candidate.label}.tum"
    write_tum(path, transformed)
    stats, _ = compare_trajectories.compare(
        args.reference,
        path,
        with_scale=args.scale,
        no_align=args.no_align,
    )
    return ExtrinsicResult(
        candidate=candidate,
        transformed_tum=path,
        rmse_m=float(stats["rmse"]),
        matched_seconds=float(stats["matched_seconds"]),
        count=int(stats["count"]),
        mean_m=float(stats["mean"]),
        median_m=float(stats["median"]),
        max_m=float(stats["max"]),
    )


def run_sweep(args) -> list[ExtrinsicResult]:
    source = compare_trajectories.load_tum(args.estimate)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    return [evaluate_candidate(args, source, candidate) for candidate in build_candidates(args)]


def format_markdown(results: list[ExtrinsicResult], args) -> str:
    best = min(results, key=lambda result: result.rmse_m) if results else None
    lines = [
        "# Visual Extrinsic Sweep",
        "",
        f"Sequence: `{args.sequence}`",
        f"Reference: `{args.reference}`",
        f"Estimate: `{args.estimate}`",
        f"Alignment: `{'Sim(3)' if args.scale else 'SE(3)' if not args.no_align else 'none'}`",
        "",
        "| x m | y m | z m | roll deg | pitch deg | yaw deg | Status | RMSE m | Matched s | Samples | Output |",
        "|----:|----:|----:|---------:|----------:|--------:|--------|-------:|----------:|--------:|--------|",
    ]
    for result in results:
        c = result.candidate
        marker = "best" if best is result else "ok"
        lines.append(
            "| "
            + " | ".join(
                [
                    format_float(c.x_m, 3),
                    format_float(c.y_m, 3),
                    format_float(c.z_m, 3),
                    format_float(math.degrees(c.roll_rad), 2),
                    format_float(math.degrees(c.pitch_rad), 2),
                    format_float(math.degrees(c.yaw_rad), 2),
                    marker,
                    format_float(result.rmse_m),
                    format_float(result.matched_seconds, 2),
                    str(result.count),
                    f"`{result.transformed_tum}`",
                ]
            )
            + " |"
        )
    if best is not None:
        c = best.candidate
        lines.extend(
            [
                "",
                "## Readout",
                "",
                (
                    f"Best RMSE: `{format_float(best.rmse_m)}` m at "
                    f"`x={format_float(c.x_m, 3)} y={format_float(c.y_m, 3)} "
                    f"z={format_float(c.z_m, 3)} roll={format_float(math.degrees(c.roll_rad), 2)}deg "
                    f"pitch={format_float(math.degrees(c.pitch_rad), 2)}deg "
                    f"yaw={format_float(math.degrees(c.yaw_rad), 2)}deg`."
                ),
            ]
        )
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Sweep camera-to-base extrinsics for an existing visual TUM trajectory."
    )
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--estimate", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_visual_extrinsic_sweep"))
    parser.add_argument("--summary-out", type=Path, default=None)
    parser.add_argument("--sequence", default="short_test")
    parser.add_argument("--x-m", default="0.0")
    parser.add_argument("--y-m", default="0.0")
    parser.add_argument("--z-m", default="0.0")
    parser.add_argument("--roll-deg", default="0.0")
    parser.add_argument("--pitch-deg", default="0.0")
    parser.add_argument("--yaw-deg", default="0.0")
    parser.add_argument("--scale", action="store_true", help="Evaluate with Sim(3) alignment.")
    parser.add_argument("--no-align", action="store_true", help="Evaluate raw positions.")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    results = run_sweep(args)
    summary = format_markdown(results, args)
    summary_out = args.summary_out or (args.out_dir / "visual_extrinsic_sweep.md")
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    summary_out.write_text(summary + "\n", encoding="utf-8")
    print(f"wrote extrinsic sweep summary: {summary_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
