#!/usr/bin/env python3
"""Analyze Tank DVL velocity as a visual-odometry motion prior."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import importlib.util
import math
from pathlib import Path
import sqlite3
import sys

import numpy as np


DEFAULT_DVL_TOPIC = "/dvl/twist"


def load_compare_module():
    script_path = Path(__file__).resolve().parent / "compare_trajectories.py"
    spec = importlib.util.spec_from_file_location("compare_trajectories", script_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class DvlRecord:
    stamp_s: float
    velocity_mps: np.ndarray


@dataclass(frozen=True)
class DvlPriorStep:
    start_stamp_s: float
    end_stamp_s: float
    offset_s: float
    dt_s: float
    dvl_step_m: float
    reference_step_m: float
    length_ratio: float
    direction_cosine: float
    heading_error_deg: float
    dvl_cumulative_m: float
    reference_cumulative_m: float
    dvl_samples: int
    covered: bool
    score: float


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def resolve_sqlite_db(bag: Path) -> Path:
    if bag.is_file():
        return bag
    candidates = sorted(bag.glob("*.db3"))
    if len(candidates) != 1:
        raise FileNotFoundError(f"expected exactly one sqlite .db3 in {bag}, found {len(candidates)}")
    return candidates[0]


def read_dvl_records(bag: Path, topic: str) -> list[DvlRecord]:
    from geometry_msgs.msg import TwistStamped
    from rclpy.serialization import deserialize_message

    db_path = resolve_sqlite_db(bag)
    with sqlite3.connect(str(db_path)) as con:
        row = con.execute("select id, type from topics where name = ?", (topic,)).fetchone()
        if row is None:
            raise ValueError(f"topic not found in {db_path}: {topic}")
        topic_id = int(row[0])
        msg_type = str(row[1])
        if msg_type != "geometry_msgs/msg/TwistStamped":
            raise ValueError(f"{topic}: expected geometry_msgs/msg/TwistStamped, got {msg_type}")
        records = []
        for (raw,) in con.execute(
            "select data from messages where topic_id = ? order by timestamp",
            (topic_id,),
        ):
            msg = deserialize_message(raw, TwistStamped)
            records.append(DvlRecord(
                stamp_s=stamp_to_seconds(msg.header.stamp),
                velocity_mps=np.asarray(
                    [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                    dtype=np.float64,
                ),
            ))
    if not records:
        raise ValueError(f"no DVL records found on {topic}")
    return records


def yaw_from_quaternion_rows(traj: np.ndarray) -> np.ndarray:
    qx = traj[:, 4]
    qy = traj[:, 5]
    qz = traj[:, 6]
    qw = traj[:, 7]
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.unwrap(np.arctan2(siny_cosp, cosy_cosp))


def interpolate_series(times: np.ndarray, values: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    out = np.full((query_times.shape[0], values.shape[1]), np.nan, dtype=np.float64)
    in_range = (query_times >= times[0]) & (query_times <= times[-1])
    if not np.any(in_range):
        return out
    for axis in range(values.shape[1]):
        out[in_range, axis] = np.interp(query_times[in_range], times, values[:, axis])
    return out


def interpolate_yaw(reference_tum: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    times = reference_tum[:, 0]
    yaw = yaw_from_quaternion_rows(reference_tum)
    out = np.full(query_times.shape[0], np.nan, dtype=np.float64)
    in_range = (query_times >= times[0]) & (query_times <= times[-1])
    if np.any(in_range):
        out[in_range] = np.interp(query_times[in_range], times, yaw)
    return out


def heading(delta_xyz: np.ndarray) -> float:
    if float(np.linalg.norm(delta_xyz[:2])) <= 1.0e-12:
        return math.nan
    return math.atan2(float(delta_xyz[1]), float(delta_xyz[0]))


def heading_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    yaw_a = heading(a)
    yaw_b = heading(b)
    if not math.isfinite(yaw_a) or not math.isfinite(yaw_b):
        return math.nan
    diff = (yaw_a - yaw_b + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(diff)


def direction_cosine(a: np.ndarray, b: np.ndarray) -> float:
    norm_a = float(np.linalg.norm(a))
    norm_b = float(np.linalg.norm(b))
    if norm_a <= 1.0e-12 or norm_b <= 1.0e-12:
        return math.nan
    return float(np.dot(a, b) / (norm_a * norm_b))


def rotate_body_velocity_by_yaw(velocity_body: np.ndarray, yaw_rad: np.ndarray) -> np.ndarray:
    rotated = velocity_body.copy()
    c = np.cos(yaw_rad)
    s = np.sin(yaw_rad)
    x = velocity_body[:, 0]
    y = velocity_body[:, 1]
    rotated[:, 0] = c * x - s * y
    rotated[:, 1] = s * x + c * y
    return rotated


def integrate_dvl_step(
    start_s: float,
    end_s: float,
    dvl_times: np.ndarray,
    dvl_velocities: np.ndarray,
    reference_tum: np.ndarray,
    mode: str,
    dvl_frame_yaw_offset_rad: float = 0.0,
) -> tuple[np.ndarray, bool, int]:
    if end_s <= start_s:
        return np.zeros(3, dtype=np.float64), False, 0
    if start_s < dvl_times[0] or end_s > dvl_times[-1]:
        return np.zeros(3, dtype=np.float64), False, 0
    inside = dvl_times[(dvl_times > start_s) & (dvl_times < end_s)]
    sample_times = np.concatenate(([start_s], inside, [end_s]))
    velocities = interpolate_series(dvl_times, dvl_velocities, sample_times)
    if np.isnan(velocities).any():
        return np.zeros(3, dtype=np.float64), False, int(inside.shape[0])
    if mode == "gt_yaw":
        yaw = interpolate_yaw(reference_tum, sample_times)
        if np.isnan(yaw).any():
            return np.zeros(3, dtype=np.float64), False, int(inside.shape[0])
        velocities = rotate_body_velocity_by_yaw(velocities, yaw + dvl_frame_yaw_offset_rad)
    elif mode != "body_raw":
        raise ValueError(f"unsupported mode: {mode}")
    elif dvl_frame_yaw_offset_rad != 0.0:
        velocities = rotate_body_velocity_by_yaw(
            velocities,
            np.full(sample_times.shape[0], dvl_frame_yaw_offset_rad, dtype=np.float64),
        )
    delta = np.trapezoid(velocities, sample_times, axis=0)
    return np.asarray(delta, dtype=np.float64), True, int(sample_times.shape[0])


def build_dvl_prior_steps(
    visual_times: np.ndarray,
    reference_xyz: np.ndarray,
    dvl_records: list[DvlRecord],
    reference_tum: np.ndarray,
    mode: str,
    min_reference_step_m: float,
    dvl_frame_yaw_offset_rad: float = 0.0,
) -> list[DvlPriorStep]:
    if min_reference_step_m < 0.0:
        raise ValueError("min_reference_step_m must be non-negative")
    dvl_times = np.asarray([record.stamp_s for record in dvl_records], dtype=np.float64)
    dvl_velocities = np.asarray([record.velocity_mps for record in dvl_records], dtype=np.float64)
    steps = []
    dvl_cumulative = 0.0
    reference_cumulative = 0.0
    t0 = float(visual_times[0])
    for index in range(1, visual_times.shape[0]):
        start_s = float(visual_times[index - 1])
        end_s = float(visual_times[index])
        reference_delta = reference_xyz[index] - reference_xyz[index - 1]
        reference_step = float(np.linalg.norm(reference_delta))
        dvl_delta, covered, dvl_samples = integrate_dvl_step(
            start_s,
            end_s,
            dvl_times,
            dvl_velocities,
            reference_tum,
            mode,
            dvl_frame_yaw_offset_rad,
        )
        dvl_step = float(np.linalg.norm(dvl_delta))
        dvl_cumulative += dvl_step if covered else 0.0
        reference_cumulative += reference_step
        if reference_step < min_reference_step_m:
            continue
        ratio = dvl_step / reference_step if covered and reference_step > 0.0 else math.nan
        cosine = direction_cosine(dvl_delta, reference_delta) if covered else math.nan
        h_error = heading_error_deg(dvl_delta, reference_delta) if covered else math.nan
        score = abs(dvl_step - reference_step) if covered else math.inf
        if covered and math.isfinite(cosine):
            score += reference_step * max(0.0, 1.0 - cosine)
        steps.append(DvlPriorStep(
            start_stamp_s=start_s,
            end_stamp_s=end_s,
            offset_s=end_s - t0,
            dt_s=end_s - start_s,
            dvl_step_m=dvl_step,
            reference_step_m=reference_step,
            length_ratio=ratio,
            direction_cosine=cosine,
            heading_error_deg=h_error,
            dvl_cumulative_m=dvl_cumulative,
            reference_cumulative_m=reference_cumulative,
            dvl_samples=dvl_samples,
            covered=covered,
            score=score,
        ))
    return steps


def matched_reference_positions(reference_path: Path, visual_path: Path):
    compare = load_compare_module()
    reference = compare.load_tum(reference_path)
    visual = compare.load_tum(visual_path)
    ref_at_visual = compare.interpolate_positions(reference, visual[:, 0])
    valid = ~np.isnan(ref_at_visual).any(axis=1)
    if np.count_nonzero(valid) < 2:
        raise ValueError("need at least two visual timestamps overlapping reference")
    return visual[valid, 0], ref_at_visual[valid], reference


def finite_values(steps: list[DvlPriorStep], attr: str, covered_only: bool = True) -> list[float]:
    values = []
    for step in steps:
        if covered_only and not step.covered:
            continue
        value = float(getattr(step, attr))
        if math.isfinite(value):
            values.append(value)
    return values


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if q <= 0.0:
        return ordered[0]
    if q >= 1.0:
        return ordered[-1]
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    alpha = pos - lo
    return ordered[lo] * (1.0 - alpha) + ordered[hi] * alpha


def stats(values: list[float]) -> dict:
    if not values:
        return {
            "count": 0,
            "min": math.nan,
            "median": math.nan,
            "mean": math.nan,
            "p95": math.nan,
            "max": math.nan,
            "std": math.nan,
        }
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "min": float(arr.min()),
        "median": percentile(values, 0.5),
        "mean": float(arr.mean()),
        "p95": percentile(values, 0.95),
        "max": float(arr.max()),
        "std": float(arr.std()),
    }


def format_float(value: float, precision: int = 6) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}g}"


def format_fixed(value: float, precision: int = 1) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{precision}f}"


def format_stats(label: str, summary: dict) -> str:
    return (
        f"| {label} | {summary['count']} | {format_float(float(summary['min']))} | "
        f"{format_float(float(summary['median']))} | {format_float(float(summary['mean']))} | "
        f"{format_float(float(summary['p95']))} | {format_float(float(summary['max']))} | "
        f"{format_float(float(summary['std']))} |"
    )


def worst_steps(steps: list[DvlPriorStep], top_k: int) -> list[DvlPriorStep]:
    return sorted(steps, key=lambda step: step.score, reverse=True)[:max(0, top_k)]


def write_csv(path: Path, steps: list[DvlPriorStep]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "start_stamp_s",
        "end_stamp_s",
        "offset_s",
        "dt_s",
        "dvl_step_m",
        "reference_step_m",
        "length_ratio",
        "direction_cosine",
        "heading_error_deg",
        "dvl_cumulative_m",
        "reference_cumulative_m",
        "dvl_samples",
        "covered",
        "score",
    ]
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for step in steps:
            writer.writerow({field: getattr(step, field) for field in fieldnames})


def format_markdown(args, steps: list[DvlPriorStep], dvl_count: int) -> str:
    covered = [step for step in steps if step.covered]
    coverage = len(covered) / len(steps) if steps else math.nan
    dvl_total = covered[-1].dvl_cumulative_m if covered else math.nan
    ref_total = steps[-1].reference_cumulative_m if steps else math.nan
    total_ratio = dvl_total / ref_total if math.isfinite(ref_total) and ref_total > 0.0 else math.nan
    ratio_stats = stats(finite_values(steps, "length_ratio"))
    cosine_stats = stats(finite_values(steps, "direction_cosine"))
    heading_stats = stats([abs(v) for v in finite_values(steps, "heading_error_deg")])
    lines = [
        "# Tank DVL Motion Prior Analysis",
        "",
        f"- Bag: `{args.bag}`",
        f"- DVL topic: `{args.dvl_topic}`",
        f"- Reference: `{args.reference}`",
        f"- Visual timestamps: `{args.visual}`",
        f"- Mode: `{args.mode}`",
        f"- DVL frame yaw offset: {format_float(float(args.dvl_frame_yaw_offset_deg), 4)} deg",
        f"- DVL samples: {dvl_count}",
        f"- Steps: {len(steps)}",
        f"- Covered steps: {len(covered)} ({format_fixed(100.0 * coverage, 1)}%)",
        f"- DVL cumulative distance: {format_float(dvl_total)} m",
        f"- Reference cumulative distance: {format_float(ref_total)} m",
        f"- DVL/reference cumulative ratio: {format_float(total_ratio)}",
        f"- CSV: `{args.csv}`",
        "",
        "## Summary",
        "",
        "| Metric | Count | Min | Median | Mean | P95 | Max | Std |",
        "|--------|------:|----:|-------:|-----:|----:|----:|----:|",
        format_stats("DVL/reference length ratio", ratio_stats),
        format_stats("direction cosine", cosine_stats),
        format_stats("absolute heading error deg", heading_stats),
        "",
        "## Worst Steps",
        "",
        "| Rank | Offset s | dt s | DVL m | Ref m | Ratio | Direction cosine | Heading error deg | Covered | Samples | Score |",
        "|-----:|---------:|-----:|------:|------:|------:|-----------------:|------------------:|:-------:|--------:|------:|",
    ]
    for rank, step in enumerate(worst_steps(steps, args.top_k), start=1):
        lines.append(
            f"| {rank} | {step.offset_s:.3f} | {step.dt_s:.3f} | "
            f"{step.dvl_step_m:.5f} | {step.reference_step_m:.5f} | "
            f"{format_float(step.length_ratio)} | {format_float(step.direction_cosine)} | "
            f"{format_float(step.heading_error_deg)} | {step.covered} | {step.dvl_samples} | "
            f"{format_float(step.score)} |"
        )
    lines.extend(["", "## Interpretation", ""])
    if math.isfinite(total_ratio):
        if total_ratio < 0.8 or total_ratio > 1.2:
            lines.append("- DVL cumulative magnitude is far from reference; scale/bias calibration is required before using it as a magnitude prior.")
        else:
            lines.append("- DVL cumulative magnitude is close enough to investigate as a visual step magnitude prior.")
    median_cosine = float(cosine_stats["median"])
    if math.isfinite(median_cosine) and median_cosine > 0.7:
        lines.append("- DVL step direction is broadly aligned in this mode; replacing GT yaw with IMU yaw is a sensible next check.")
    elif math.isfinite(median_cosine):
        lines.append("- DVL direction is weak in this mode; yaw/frame conventions or sensor quality need more work before fusion.")
    lines.append("")
    return "\n".join(lines)


def run_analysis(args) -> tuple[str, list[DvlPriorStep]]:
    visual_times, reference_xyz, reference_tum = matched_reference_positions(
        args.reference, args.visual
    )
    dvl_records = read_dvl_records(args.bag, args.dvl_topic)
    steps = build_dvl_prior_steps(
        visual_times,
        reference_xyz,
        dvl_records,
        reference_tum,
        args.mode,
        args.min_reference_step_m,
        math.radians(args.dvl_frame_yaw_offset_deg),
    )
    write_csv(args.csv, steps)
    text = format_markdown(args, steps, len(dvl_records))
    return text, steps


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Analyze Tank DVL velocity as a visual motion prior."
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--visual", required=True, type=Path, help="Visual TUM used only for timestamps.")
    parser.add_argument("--dvl-topic", default=DEFAULT_DVL_TOPIC)
    parser.add_argument("--mode", choices=["body_raw", "gt_yaw"], default="gt_yaw")
    parser.add_argument(
        "--dvl-frame-yaw-offset-deg",
        type=float,
        default=0.0,
        help="Yaw rotation from the DVL horizontal velocity axes into the body frame.",
    )
    parser.add_argument("--min-reference-step-m", type=float, default=0.0)
    parser.add_argument("--top-k", type=int, default=12)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--csv", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.min_reference_step_m < 0.0:
        raise ValueError("--min-reference-step-m must be non-negative")
    if args.top_k < 0:
        raise ValueError("--top-k must be non-negative")
    text, _ = run_analysis(args)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
