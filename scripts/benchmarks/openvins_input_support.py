"""Versioned native-IMU support policy shared by both VIO comparison arms.

This is a new contract, not a reconstruction of the missing legacy converters.
It returns an explicit audit/mask; it never deletes or modifies input samples.
"""
from dataclasses import asdict, dataclass
from bisect import bisect_left
from typing import Iterable

POLICY_VERSION = "native-imu-support-v2"
CAMERA_OFFSET_NS = -40380655


def nanoseconds(value: int) -> int:
    if type(value) is not int:
        raise ValueError("timestamps and offsets must be integer nanoseconds")
    return value


@dataclass(frozen=True)
class Observation:
    sensor: str
    native_ordinal: int
    native_stamp_ns: int
    source_message_sha256: str = ""
    bag_stamp_ns: int | None = None


def audit_support(imu_stamps: Iterable[int], observations: Iterable[Observation],
                  camera_offset_ns: int = CAMERA_OFFSET_NS) -> dict:
    offset = nanoseconds(camera_offset_ns)
    imu = [nanoseconds(t) for t in imu_stamps]
    if len(imu) < 2 or any(a >= b for a, b in zip(imu, imu[1:])):
        raise ValueError("native IMU needs at least two strictly increasing timestamps")
    bounds = (imu[0], imu[-1])
    records, counts, eligible, previous = [], {}, {}, {}
    for observation in observations:
        sensor = observation.sensor
        if sensor not in {"camera", "pressure"}:
            raise ValueError("only camera and pressure observations are supported")
        ordinal = observation.native_ordinal
        if type(ordinal) is not int or ordinal != counts.get(sensor, 0):
            raise ValueError("retain consecutive native ordinals starting at zero")
        stamp = nanoseconds(observation.native_stamp_ns)
        if sensor in previous and stamp <= previous[sensor]:
            raise ValueError(f"{sensor} timestamps must be strictly increasing")
        previous[sensor] = stamp
        corrected = stamp + (offset if sensor == "camera" else 0)
        reason = ("before_native_imu_support" if corrected < bounds[0] else
                  "after_native_imu_support" if corrected > bounds[1] else None)
        record = asdict(observation)
        bracket = None
        if reason is None:
            upper = bisect_left(imu, corrected)
            bracket = [upper, upper] if imu[upper] == corrected else [upper - 1, upper]
        record.update(corrected_stamp_ns=corrected, included=reason is None,
                      exclusion_reason=reason,
                      native_imu_bracket_ordinals=bracket,
                      native_imu_bracket_stamps_ns=([imu[i] for i in bracket] if bracket else None),
                      distance_outside_support_ns=max(bounds[0] - corrected,
                                                      corrected - bounds[1], 0))
        records.append(record)
        counts[sensor] = counts.get(sensor, 0) + 1
        eligible[sensor] = eligible.get(sensor, 0) + int(reason is None)
    return {
        "policy_version": POLICY_VERSION,
        "camera_offset_ns": offset,
        "native_imu_count": len(imu),
        "native_imu_support_ns": list(bounds),
        "support_interval": "closed",
        "native_observation_counts": counts,
        "eligible_observation_counts": eligible,
        "excluded_observation_counts": {s: n - eligible[s] for s, n in counts.items()},
        "camera_mask_shared_by_both_arms": True,
        "input_retention": "native bag and every observation retained; this is an audit, not a stream",
        "records": records,
    }
