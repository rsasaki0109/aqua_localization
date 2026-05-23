"""Pure Tank DVL motion-prior math.

This module intentionally has no ROS imports. Keep bag decoding, CLI parsing,
and Markdown rendering outside this file so the core motion-prior behavior stays
cheap to test.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class DvlRecord:
    stamp_s: float
    velocity_mps: np.ndarray


@dataclass(frozen=True)
class ImuYawRecord:
    stamp_s: float
    yaw_rad: float


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


@dataclass(frozen=True)
class DvlPriorDelta:
    start_stamp_s: float
    end_stamp_s: float
    delta_xyz: np.ndarray
    dvl_samples: int
    covered: bool


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_from_quaternion_rows(traj: np.ndarray) -> np.ndarray:
    return np.unwrap(np.asarray([
        quaternion_to_yaw(float(row[4]), float(row[5]), float(row[6]), float(row[7]))
        for row in traj
    ], dtype=np.float64))


def interpolate_series(times: np.ndarray, values: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    out = np.full((query_times.shape[0], values.shape[1]), np.nan, dtype=np.float64)
    in_range = (query_times >= times[0]) & (query_times <= times[-1])
    if not np.any(in_range):
        return out
    for axis in range(values.shape[1]):
        out[in_range, axis] = np.interp(query_times[in_range], times, values[:, axis])
    return out


def interpolate_yaw_series(times: np.ndarray, yaw_rad: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    out = np.full(query_times.shape[0], np.nan, dtype=np.float64)
    in_range = (query_times >= times[0]) & (query_times <= times[-1])
    if np.any(in_range):
        out[in_range] = np.interp(query_times[in_range], times, np.unwrap(yaw_rad))
    return out


def interpolate_reference_yaw(reference_tum: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    return interpolate_yaw_series(reference_tum[:, 0], yaw_from_quaternion_rows(reference_tum), query_times)


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
    yaw_times: np.ndarray | None = None,
    yaw_rad: np.ndarray | None = None,
    imu_yaw_offset_rad: float = 0.0,
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
        yaw = interpolate_reference_yaw(reference_tum, sample_times)
        if np.isnan(yaw).any():
            return np.zeros(3, dtype=np.float64), False, int(inside.shape[0])
        velocities = rotate_body_velocity_by_yaw(velocities, yaw + dvl_frame_yaw_offset_rad)
    elif mode == "imu_yaw":
        if yaw_times is None or yaw_rad is None:
            raise ValueError("imu_yaw mode requires IMU yaw records")
        yaw = interpolate_yaw_series(yaw_times, yaw_rad, sample_times)
        if np.isnan(yaw).any():
            return np.zeros(3, dtype=np.float64), False, int(inside.shape[0])
        velocities = rotate_body_velocity_by_yaw(
            velocities,
            yaw + imu_yaw_offset_rad + dvl_frame_yaw_offset_rad,
        )
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
    imu_yaw_records: list[ImuYawRecord] | None = None,
    imu_yaw_offset_rad: float = 0.0,
) -> list[DvlPriorStep]:
    if min_reference_step_m < 0.0:
        raise ValueError("min_reference_step_m must be non-negative")
    dvl_times = np.asarray([record.stamp_s for record in dvl_records], dtype=np.float64)
    dvl_velocities = np.asarray([record.velocity_mps for record in dvl_records], dtype=np.float64)
    yaw_times = None
    yaws = None
    if imu_yaw_records is not None:
        yaw_times = np.asarray([record.stamp_s for record in imu_yaw_records], dtype=np.float64)
        yaws = np.asarray([record.yaw_rad for record in imu_yaw_records], dtype=np.float64)
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
            yaw_times,
            yaws,
            imu_yaw_offset_rad,
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


def build_dvl_prior_deltas(
    visual_times: np.ndarray,
    dvl_records: list[DvlRecord],
    reference_tum: np.ndarray,
    mode: str,
    dvl_frame_yaw_offset_rad: float = 0.0,
    imu_yaw_records: list[ImuYawRecord] | None = None,
    imu_yaw_offset_rad: float = 0.0,
    prior_scale: float = 1.0,
) -> list[DvlPriorDelta]:
    if prior_scale <= 0.0:
        raise ValueError("prior_scale must be positive")
    dvl_times = np.asarray([record.stamp_s for record in dvl_records], dtype=np.float64)
    dvl_velocities = np.asarray([record.velocity_mps for record in dvl_records], dtype=np.float64)
    yaw_times = None
    yaws = None
    if imu_yaw_records is not None:
        yaw_times = np.asarray([record.stamp_s for record in imu_yaw_records], dtype=np.float64)
        yaws = np.asarray([record.yaw_rad for record in imu_yaw_records], dtype=np.float64)
    deltas = []
    for index in range(1, visual_times.shape[0]):
        start_s = float(visual_times[index - 1])
        end_s = float(visual_times[index])
        delta, covered, dvl_samples = integrate_dvl_step(
            start_s,
            end_s,
            dvl_times,
            dvl_velocities,
            reference_tum,
            mode,
            dvl_frame_yaw_offset_rad,
            yaw_times,
            yaws,
            imu_yaw_offset_rad,
        )
        deltas.append(DvlPriorDelta(
            start_stamp_s=start_s,
            end_stamp_s=end_s,
            delta_xyz=prior_scale * delta if covered else np.zeros(3, dtype=np.float64),
            dvl_samples=dvl_samples,
            covered=covered,
        ))
    return deltas


def positions_from_deltas(
    times: np.ndarray,
    start_xyz: np.ndarray,
    deltas: list[DvlPriorDelta],
) -> np.ndarray:
    if len(deltas) != max(0, times.shape[0] - 1):
        raise ValueError("deltas length must equal len(times) - 1")
    xyz = np.zeros((times.shape[0], 3), dtype=np.float64)
    xyz[0] = start_xyz
    for index, delta in enumerate(deltas, start=1):
        xyz[index] = xyz[index - 1] + delta.delta_xyz
    return xyz


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


def worst_steps(steps: list[DvlPriorStep], top_k: int) -> list[DvlPriorStep]:
    return sorted(steps, key=lambda step: step.score, reverse=True)[:max(0, top_k)]
