#!/usr/bin/env python3
"""Export MBES loop-closure status messages from a rosbag2 bag.

The CSV output is intended for threshold tuning. It preserves every
`aqua_msgs/LoopClosureStatus` sample and prints a compact markdown summary with
accepted/rejected/no-candidate counts, rejection reasons, and registration
fitness/correction/descriptor quantiles.

Example:

  ros2 run aqua_localization export_mbes_loop_status.py \\
    --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \\
    --out /tmp/mbes_loop_status.csv \\
    --summary-out /tmp/mbes_loop_status.md \\
    --descriptor-sweep-out /tmp/mbes_loop_descriptor_sweep.md \\
    --consistency-sweep-out /tmp/mbes_loop_consistency_sweep.md \\
    --consistency-rejection-audit-out /tmp/mbes_loop_consistency_rejections.md \\
    --batch-consistency-out /tmp/mbes_loop_batch_consistency.md \\
    --batch-consistency-selected-csv-out /tmp/mbes_loop_batch_selected_loops.csv
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter
from dataclasses import dataclass, replace
import math
from pathlib import Path
import sys
from typing import Iterable


NO_CANDIDATE_ID = 2**32 - 1

CSV_FIELDS = [
    "timestamp",
    "current_keyframe_timestamp",
    "candidate_keyframe_timestamp",
    "frame_id",
    "current_id",
    "candidate_id",
    "accepted",
    "converged",
    "fitness_score",
    "correction_translation_m",
    "correction_rotation_rad",
    "correction_pose_valid",
    "correction_x_m",
    "correction_y_m",
    "correction_z_m",
    "correction_qx",
    "correction_qy",
    "correction_qz",
    "correction_qw",
    "consistency_support_count",
    "consistency_required_support_count",
    "consistency_nearest_translation_delta_m",
    "consistency_nearest_rotation_delta_rad",
    "descriptor_centroid_distance_m",
    "descriptor_extent_ratio",
    "descriptor_point_count_ratio",
    "status",
]


@dataclass(frozen=True)
class LoopStatusSample:
    timestamp: float
    frame_id: str
    current_id: int
    candidate_id: int
    accepted: bool
    converged: bool
    fitness_score: float
    correction_translation_m: float
    correction_rotation_rad: float
    descriptor_centroid_distance_m: float
    descriptor_extent_ratio: float
    descriptor_point_count_ratio: float
    status: str
    correction_pose_valid: bool = False
    correction_x_m: float = math.nan
    correction_y_m: float = math.nan
    correction_z_m: float = math.nan
    correction_qx: float = math.nan
    correction_qy: float = math.nan
    correction_qz: float = math.nan
    correction_qw: float = math.nan
    consistency_support_count: int = 0
    consistency_required_support_count: int = 0
    consistency_nearest_translation_delta_m: float = math.nan
    consistency_nearest_rotation_delta_rad: float = math.nan
    current_keyframe_timestamp: float = math.nan
    candidate_keyframe_timestamp: float = math.nan


@dataclass(frozen=True)
class TopicValueSample:
    timestamp: float
    value: float


@dataclass(frozen=True)
class OptimizationDiagnostics:
    count_topic: str
    chi2_topic: str
    count_samples: list[TopicValueSample]
    chi2_samples: list[TopicValueSample]

    @property
    def latest_count(self) -> int | None:
        if not self.count_samples:
            return None
        return int(round(self.count_samples[-1].value))

    @property
    def latest_chi2(self) -> float:
        if not self.chi2_samples:
            return math.nan
        return float(self.chi2_samples[-1].value)


@dataclass(frozen=True)
class ConsistencyDeltaSample:
    anchor_current_id: int
    current_id: int
    translation_delta_m: float
    rotation_delta_rad: float
    uses_pose: bool


@dataclass(frozen=True)
class BatchConsistencyResult:
    candidates: list[LoopStatusSample]
    selected_indices: list[int]
    adjacency: list[int]
    translation_threshold_m: float
    rotation_threshold_rad: float
    translation_threshold_source: str
    rotation_threshold_source: str
    pair_count: int
    edge_count: int
    pose_pair_count: int
    magnitude_pair_count: int
    algorithm: str


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def optional_float(msg, attr: str) -> float:
    return float(getattr(msg, attr, math.nan))


def optional_bool(msg, attr: str) -> bool:
    return bool(getattr(msg, attr, False))


def optional_int(msg, attr: str) -> int:
    return int(getattr(msg, attr, 0))


def optional_pose_float(msg, attr: str, component: str) -> float:
    pose = getattr(msg, attr, None)
    if pose is None:
        return math.nan
    target = pose
    for name in component.split("."):
        target = getattr(target, name, None)
        if target is None:
            return math.nan
    return float(target)


def sample_from_msg(msg, fallback_time: float) -> LoopStatusSample:
    timestamp = stamp_to_seconds(msg.header.stamp)
    if timestamp <= 0.0:
        timestamp = fallback_time
    return LoopStatusSample(
        timestamp=timestamp,
        frame_id=str(msg.header.frame_id),
        current_id=int(msg.current_id),
        candidate_id=int(msg.candidate_id),
        accepted=bool(msg.accepted),
        converged=bool(msg.converged),
        fitness_score=float(msg.fitness_score),
        correction_translation_m=float(msg.correction_translation_m),
        correction_rotation_rad=float(msg.correction_rotation_rad),
        descriptor_centroid_distance_m=optional_float(
            msg, "descriptor_centroid_distance_m"
        ),
        descriptor_extent_ratio=optional_float(msg, "descriptor_extent_ratio"),
        descriptor_point_count_ratio=optional_float(
            msg, "descriptor_point_count_ratio"
        ),
        status=str(msg.status),
        correction_pose_valid=optional_bool(msg, "correction_pose_valid"),
        correction_x_m=optional_pose_float(msg, "correction_pose", "position.x"),
        correction_y_m=optional_pose_float(msg, "correction_pose", "position.y"),
        correction_z_m=optional_pose_float(msg, "correction_pose", "position.z"),
        correction_qx=optional_pose_float(msg, "correction_pose", "orientation.x"),
        correction_qy=optional_pose_float(msg, "correction_pose", "orientation.y"),
        correction_qz=optional_pose_float(msg, "correction_pose", "orientation.z"),
        correction_qw=optional_pose_float(msg, "correction_pose", "orientation.w"),
        consistency_support_count=optional_int(msg, "consistency_support_count"),
        consistency_required_support_count=optional_int(
            msg, "consistency_required_support_count"
        ),
        consistency_nearest_translation_delta_m=optional_float(
            msg, "consistency_nearest_translation_delta_m"
        ),
        consistency_nearest_rotation_delta_rad=optional_float(
            msg, "consistency_nearest_rotation_delta_rad"
        ),
        current_keyframe_timestamp=timestamp,
    )


def finite_values(samples: Iterable[LoopStatusSample], attr: str) -> list[float]:
    values = []
    for sample in samples:
        value = float(getattr(sample, attr))
        if math.isfinite(value):
            values.append(value)
    return values


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    if q <= 0.0:
        return min(values)
    if q >= 1.0:
        return max(values)
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    alpha = pos - lo
    return ordered[lo] * (1.0 - alpha) + ordered[hi] * alpha


def stats(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {
            "count": 0,
            "min": math.nan,
            "median": math.nan,
            "p95": math.nan,
            "max": math.nan,
        }
    return {
        "count": len(values),
        "min": min(values),
        "median": percentile(values, 0.5),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def unique_finite(values: Iterable[float]) -> list[float]:
    unique = sorted({float(value) for value in values if math.isfinite(float(value))})
    return unique


def candidate_thresholds(values: list[float], quantiles: list[float]) -> list[float]:
    if not values:
        return []
    return unique_finite(percentile(values, q) for q in quantiles)


def is_no_candidate(sample: LoopStatusSample) -> bool:
    status = sample.status.lower()
    return "no candidate" in status or (
        sample.candidate_id == NO_CANDIDATE_ID and not status
    )


def summarize(samples: list[LoopStatusSample]) -> dict:
    accepted = [sample for sample in samples if sample.accepted]
    no_candidate = [sample for sample in samples if is_no_candidate(sample)]
    rejected = [
        sample for sample in samples
        if not sample.accepted and not is_no_candidate(sample)
    ]
    return {
        "total": len(samples),
        "accepted": len(accepted),
        "rejected": len(rejected),
        "no_candidate": len(no_candidate),
        "converged": sum(1 for sample in samples if sample.converged),
        "status_counts": Counter(sample.status for sample in samples),
        "rejection_counts": Counter(sample.status for sample in rejected),
        "fitness": stats(finite_values(samples, "fitness_score")),
        "accepted_fitness": stats(finite_values(accepted, "fitness_score")),
        "correction_translation_m": stats(
            finite_values(samples, "correction_translation_m")
        ),
        "correction_rotation_rad": stats(
            finite_values(samples, "correction_rotation_rad")
        ),
        "descriptor_centroid_distance_m": stats(
            finite_values(samples, "descriptor_centroid_distance_m")
        ),
        "descriptor_extent_ratio": stats(
            finite_values(samples, "descriptor_extent_ratio")
        ),
        "descriptor_point_count_ratio": stats(
            finite_values(samples, "descriptor_point_count_ratio")
        ),
        "consistency_nearest_translation_delta_m": stats(
            finite_values(samples, "consistency_nearest_translation_delta_m")
        ),
        "consistency_nearest_rotation_delta_rad": stats(
            finite_values(samples, "consistency_nearest_rotation_delta_rad")
        ),
    }


def descriptor_sweep_rows(samples: list[LoopStatusSample]) -> list[dict[str, float | int]]:
    descriptor_samples = [
        sample for sample in samples
        if all(math.isfinite(value) for value in (
            sample.descriptor_centroid_distance_m,
            sample.descriptor_extent_ratio,
            sample.descriptor_point_count_ratio,
        ))
    ]
    if not descriptor_samples:
        return []

    centroid_thresholds = candidate_thresholds(
        [sample.descriptor_centroid_distance_m for sample in descriptor_samples],
        [0.5, 0.75, 0.9, 0.95],
    )
    extent_thresholds = candidate_thresholds(
        [sample.descriptor_extent_ratio for sample in descriptor_samples],
        [0.75, 0.9, 0.95, 1.0],
    )
    point_ratio_thresholds = candidate_thresholds(
        [sample.descriptor_point_count_ratio for sample in descriptor_samples],
        [0.0, 0.05, 0.1, 0.25],
    )

    rows = []
    for centroid_threshold in centroid_thresholds:
        for extent_threshold in extent_thresholds:
            for point_ratio_threshold in point_ratio_thresholds:
                pass_count = sum(
                    1 for sample in descriptor_samples
                    if sample.descriptor_centroid_distance_m <= centroid_threshold and
                    sample.descriptor_extent_ratio <= extent_threshold and
                    sample.descriptor_point_count_ratio >= point_ratio_threshold
                )
                rows.append({
                    "centroid_threshold": centroid_threshold,
                    "extent_threshold": extent_threshold,
                    "point_ratio_threshold": point_ratio_threshold,
                    "pass_count": pass_count,
                    "total_count": len(descriptor_samples),
                })
    rows.sort(
        key=lambda row: (
            int(row["pass_count"]),
            float(row["centroid_threshold"]),
            float(row["extent_threshold"]),
            -float(row["point_ratio_threshold"]),
        ),
        reverse=True,
    )
    return rows


def has_finite_correction_magnitudes(sample: LoopStatusSample) -> bool:
    return (
        math.isfinite(sample.correction_translation_m) and
        math.isfinite(sample.correction_rotation_rad)
    )


def accepted_correction_samples(samples: list[LoopStatusSample]) -> list[LoopStatusSample]:
    return [
        sample for sample in samples
        if sample.accepted and has_finite_correction_magnitudes(sample)
    ]


def has_finite_correction_pose(sample: LoopStatusSample) -> bool:
    if not sample.correction_pose_valid:
        return False
    values = [
        sample.correction_x_m,
        sample.correction_y_m,
        sample.correction_z_m,
        sample.correction_qx,
        sample.correction_qy,
        sample.correction_qz,
        sample.correction_qw,
    ]
    if not all(math.isfinite(value) for value in values):
        return False
    q_norm = math.sqrt(
        sample.correction_qx * sample.correction_qx +
        sample.correction_qy * sample.correction_qy +
        sample.correction_qz * sample.correction_qz +
        sample.correction_qw * sample.correction_qw
    )
    return q_norm > 1.0e-12


def correction_pose_delta(
    anchor: LoopStatusSample,
    sample: LoopStatusSample,
) -> tuple[float, float]:
    dx = sample.correction_x_m - anchor.correction_x_m
    dy = sample.correction_y_m - anchor.correction_y_m
    dz = sample.correction_z_m - anchor.correction_z_m
    translation_delta_m = math.sqrt(dx * dx + dy * dy + dz * dz)

    dot = (
        anchor.correction_qx * sample.correction_qx +
        anchor.correction_qy * sample.correction_qy +
        anchor.correction_qz * sample.correction_qz +
        anchor.correction_qw * sample.correction_qw
    )
    anchor_norm = math.sqrt(
        anchor.correction_qx * anchor.correction_qx +
        anchor.correction_qy * anchor.correction_qy +
        anchor.correction_qz * anchor.correction_qz +
        anchor.correction_qw * anchor.correction_qw
    )
    sample_norm = math.sqrt(
        sample.correction_qx * sample.correction_qx +
        sample.correction_qy * sample.correction_qy +
        sample.correction_qz * sample.correction_qz +
        sample.correction_qw * sample.correction_qw
    )
    normalized_dot = abs(dot / (anchor_norm * sample_norm))
    normalized_dot = max(-1.0, min(1.0, normalized_dot))
    rotation_delta_rad = 2.0 * math.acos(normalized_dot)
    return translation_delta_m, rotation_delta_rad


def correction_delta_between(
    anchor: LoopStatusSample,
    sample: LoopStatusSample,
) -> ConsistencyDeltaSample:
    uses_pose = (
        has_finite_correction_pose(anchor) and
        has_finite_correction_pose(sample)
    )
    if uses_pose:
        translation_delta_m, rotation_delta_rad = correction_pose_delta(
            anchor, sample
        )
    else:
        translation_delta_m = abs(
            sample.correction_translation_m -
            anchor.correction_translation_m
        )
        rotation_delta_rad = abs(
            sample.correction_rotation_rad -
            anchor.correction_rotation_rad
        )
    return ConsistencyDeltaSample(
        anchor_current_id=anchor.current_id,
        current_id=sample.current_id,
        translation_delta_m=translation_delta_m,
        rotation_delta_rad=rotation_delta_rad,
        uses_pose=uses_pose,
    )


def consistency_delta_samples(
    samples: list[LoopStatusSample],
) -> list[ConsistencyDeltaSample]:
    accepted = accepted_correction_samples(samples)
    deltas: list[ConsistencyDeltaSample] = []
    for index, sample in enumerate(accepted):
        for anchor in accepted[:index]:
            deltas.append(correction_delta_between(anchor, sample))
    return deltas


def consistency_delta_summary(samples: list[LoopStatusSample]) -> dict:
    accepted = accepted_correction_samples(samples)
    deltas = consistency_delta_samples(samples)
    return {
        "accepted_count": len(accepted),
        "pair_count": len(deltas),
        "pose_pair_count": sum(1 for delta in deltas if delta.uses_pose),
        "magnitude_pair_count": sum(1 for delta in deltas if not delta.uses_pose),
        "translation_delta_m": stats([
            delta.translation_delta_m for delta in deltas
        ]),
        "rotation_delta_rad": stats([
            delta.rotation_delta_rad for delta in deltas
        ]),
    }


def consistency_supported_count(
    accepted: list[LoopStatusSample],
    translation_threshold: float,
    rotation_threshold: float,
    min_support_count: int = 1,
) -> int:
    if not accepted:
        return 0
    configured_support = max(1, int(min_support_count))
    retained = [accepted[0]]
    for sample in accepted[1:]:
        required_support = min(configured_support, len(retained))
        support_count = 0
        for anchor in retained:
            delta = correction_delta_between(anchor, sample)
            if (
                delta.translation_delta_m <= translation_threshold and
                delta.rotation_delta_rad <= rotation_threshold
            ):
                support_count += 1
                if support_count >= required_support:
                    break
        if support_count >= required_support:
            retained.append(sample)
    return len(retained)


def consistency_sweep_rows(
    samples: list[LoopStatusSample],
    min_support_count: int = 1,
) -> list[dict[str, float | int]]:
    accepted = accepted_correction_samples(samples)
    if len(accepted) < 2:
        return []

    deltas = consistency_delta_samples(samples)
    translation_thresholds = candidate_thresholds(
        [delta.translation_delta_m for delta in deltas],
        [0.5, 0.75, 0.9, 0.95, 1.0],
    )
    rotation_thresholds = candidate_thresholds(
        [delta.rotation_delta_rad for delta in deltas],
        [0.5, 0.75, 0.9, 0.95, 1.0],
    )

    rows = []
    for translation_threshold in translation_thresholds:
        for rotation_threshold in rotation_thresholds:
            supported_count = consistency_supported_count(
                accepted,
                translation_threshold,
                rotation_threshold,
                min_support_count,
            )
            pair_support_count = sum(
                1 for delta in deltas
                if delta.translation_delta_m <= translation_threshold and
                delta.rotation_delta_rad <= rotation_threshold
            )
            rows.append({
                "translation_threshold_m": translation_threshold,
                "rotation_threshold_rad": rotation_threshold,
                "supported_count": supported_count,
                "total_count": len(accepted),
                "min_support_count": max(1, int(min_support_count)),
                "pair_support_count": pair_support_count,
                "pair_total_count": len(deltas),
            })
    rows.sort(
        key=lambda row: (
            -int(row["supported_count"]),
            float(row["translation_threshold_m"]),
            float(row["rotation_threshold_rad"]),
        )
    )
    return rows


def is_consistency_rejection(sample: LoopStatusSample) -> bool:
    return (
        not sample.accepted and
        "loop consistency rejected" in sample.status.lower()
    )


def support_deficit(sample: LoopStatusSample) -> int:
    return max(
        0,
        int(sample.consistency_required_support_count) -
        int(sample.consistency_support_count),
    )


def finite_sort_value(value: float) -> float:
    return value if math.isfinite(value) else -math.inf


def consistency_rejection_samples(
    samples: list[LoopStatusSample],
    limit: int = 50,
) -> list[LoopStatusSample]:
    rejections = [sample for sample in samples if is_consistency_rejection(sample)]
    rejections.sort(
        key=lambda sample: (
            -support_deficit(sample),
            -finite_sort_value(sample.consistency_nearest_translation_delta_m),
            -finite_sort_value(sample.consistency_nearest_rotation_delta_rad),
            sample.timestamp,
        )
    )
    if limit <= 0:
        return []
    return rejections[:limit]


def batch_consistency_candidates(
    samples: list[LoopStatusSample],
) -> list[LoopStatusSample]:
    return [
        sample for sample in samples
        if (
            has_finite_correction_magnitudes(sample) and
            not is_no_candidate(sample) and
            (sample.accepted or is_consistency_rejection(sample))
        )
    ]


def explicit_batch_threshold(value: float) -> float | None:
    if math.isfinite(value) and value >= 0.0:
        return value
    return None


def resolved_batch_threshold(
    explicit_value: float,
    values: list[float],
    auto_quantile: float,
) -> tuple[float, str]:
    configured = explicit_batch_threshold(explicit_value)
    if configured is not None:
        return configured, "explicit"
    if not values:
        return math.nan, "unavailable"
    quantile = min(1.0, max(0.0, float(auto_quantile)))
    return percentile(values, quantile), f"auto p{quantile * 100.0:.0f}"


def batch_clique_score(mask: int, candidates: list[LoopStatusSample]) -> tuple:
    indices = mask_to_indices(mask)
    accepted_count = sum(1 for index in indices if candidates[index].accepted)
    fitness_sum = sum(
        candidates[index].fitness_score
        if math.isfinite(candidates[index].fitness_score) else 1.0e9
        for index in indices
    )
    correction_sum = sum(
        candidates[index].correction_translation_m
        if math.isfinite(candidates[index].correction_translation_m) else 1.0e9
        for index in indices
    )
    if indices:
        first_current_id = min(candidates[index].current_id for index in indices)
    else:
        first_current_id = 0
    return (
        len(indices),
        accepted_count,
        -fitness_sum,
        -correction_sum,
        -first_current_id,
    )


def mask_to_indices(mask: int) -> list[int]:
    indices = []
    while mask:
        bit = mask & -mask
        indices.append(bit.bit_length() - 1)
        mask &= ~bit
    return indices


def maximum_clique_exact(
    adjacency: list[int],
    candidates: list[LoopStatusSample],
) -> int:
    if not adjacency:
        return 0

    best_mask = 0
    best_score = batch_clique_score(best_mask, candidates)

    def consider(mask: int) -> None:
        nonlocal best_mask, best_score
        score = batch_clique_score(mask, candidates)
        if score > best_score:
            best_mask = mask
            best_score = score

    def expand(clique: int, possible: int, excluded: int) -> None:
        nonlocal best_score
        if not possible and not excluded:
            consider(clique)
            return
        if clique.bit_count() + possible.bit_count() < best_score[0]:
            return

        pivot_pool = possible | excluded
        if pivot_pool:
            pivot = max(
                mask_to_indices(pivot_pool),
                key=lambda index: (adjacency[index] & possible).bit_count(),
            )
            branch = possible & ~adjacency[pivot]
        else:
            branch = possible

        while branch:
            bit = branch & -branch
            index = bit.bit_length() - 1
            expand(clique | bit, possible & adjacency[index], excluded & adjacency[index])
            possible &= ~bit
            excluded |= bit
            branch &= ~bit
            if clique.bit_count() + possible.bit_count() < best_score[0]:
                break

    expand(0, (1 << len(adjacency)) - 1, 0)
    return best_mask


def maximum_clique_greedy(
    adjacency: list[int],
    candidates: list[LoopStatusSample],
) -> int:
    if not adjacency:
        return 0

    degrees = [mask.bit_count() for mask in adjacency]
    seed_order = sorted(
        range(len(adjacency)),
        key=lambda index: (
            -degrees[index],
            candidates[index].fitness_score
            if math.isfinite(candidates[index].fitness_score) else math.inf,
            candidates[index].current_id,
        ),
    )
    best_mask = 0
    best_score = batch_clique_score(best_mask, candidates)
    for seed in seed_order:
        clique = 1 << seed
        possible = adjacency[seed]
        while possible:
            index = max(
                mask_to_indices(possible),
                key=lambda item: (
                    degrees[item],
                    -(
                        candidates[item].fitness_score
                        if math.isfinite(candidates[item].fitness_score)
                        else math.inf
                    ),
                    -candidates[item].current_id,
                ),
            )
            clique |= 1 << index
            possible &= adjacency[index]
        score = batch_clique_score(clique, candidates)
        if score > best_score:
            best_mask = clique
            best_score = score
    return best_mask


def batch_consistency_selection(
    samples: list[LoopStatusSample],
    translation_threshold_m: float = math.nan,
    rotation_threshold_rad: float = math.nan,
    auto_quantile: float = 0.95,
    exact_limit: int = 64,
) -> BatchConsistencyResult:
    candidates = batch_consistency_candidates(samples)
    deltas = [
        correction_delta_between(anchor, sample)
        for index, sample in enumerate(candidates)
        for anchor in candidates[:index]
    ]
    resolved_translation, translation_source = resolved_batch_threshold(
        translation_threshold_m,
        [delta.translation_delta_m for delta in deltas],
        auto_quantile,
    )
    resolved_rotation, rotation_source = resolved_batch_threshold(
        rotation_threshold_rad,
        [delta.rotation_delta_rad for delta in deltas],
        auto_quantile,
    )

    adjacency = [0 for _ in candidates]
    edge_count = 0
    if (
        len(candidates) > 1 and
        math.isfinite(resolved_translation) and
        math.isfinite(resolved_rotation)
    ):
        for index, sample in enumerate(candidates):
            for anchor_index, anchor in enumerate(candidates[:index]):
                delta = correction_delta_between(anchor, sample)
                if (
                    delta.translation_delta_m <= resolved_translation and
                    delta.rotation_delta_rad <= resolved_rotation
                ):
                    adjacency[index] |= 1 << anchor_index
                    adjacency[anchor_index] |= 1 << index
                    edge_count += 1

    if not candidates:
        selected_mask = 0
        algorithm = "none"
    elif len(candidates) == 1:
        selected_mask = 1
        algorithm = "single-candidate"
    elif len(candidates) <= max(1, int(exact_limit)):
        selected_mask = maximum_clique_exact(adjacency, candidates)
        algorithm = "exact maximum clique"
    else:
        selected_mask = maximum_clique_greedy(adjacency, candidates)
        algorithm = f"greedy clique, n>{max(1, int(exact_limit))}"

    return BatchConsistencyResult(
        candidates=candidates,
        selected_indices=mask_to_indices(selected_mask),
        adjacency=adjacency,
        translation_threshold_m=resolved_translation,
        rotation_threshold_rad=resolved_rotation,
        translation_threshold_source=translation_source,
        rotation_threshold_source=rotation_source,
        pair_count=len(deltas),
        edge_count=edge_count,
        pose_pair_count=sum(1 for delta in deltas if delta.uses_pose),
        magnitude_pair_count=sum(1 for delta in deltas if not delta.uses_pose),
        algorithm=algorithm,
    )


def selected_index_set(result: BatchConsistencyResult) -> set[int]:
    return set(result.selected_indices)


def format_float(value: float) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.6g}"


def current_keyframe_timestamp(sample: LoopStatusSample) -> float:
    if math.isfinite(sample.current_keyframe_timestamp):
        return sample.current_keyframe_timestamp
    return sample.timestamp


def format_stats(label: str, summary: dict[str, float | int]) -> str:
    return (
        f"| {label} | {summary['count']} | {format_float(float(summary['min']))} | "
        f"{format_float(float(summary['median']))} | {format_float(float(summary['p95']))} | "
        f"{format_float(float(summary['max']))} |"
    )


def format_descriptor_sweep_markdown(samples: list[LoopStatusSample], topic: str) -> str:
    rows = descriptor_sweep_rows(samples)
    lines = [
        "# MBES Loop Closure Descriptor Threshold Sweep",
        "",
        f"- Topic: `{topic}`",
        f"- Samples: {len(samples)}",
        "",
    ]
    if not rows:
        lines.extend([
            "No finite descriptor metrics were available for threshold sweep.",
            "",
        ])
        return "\n".join(lines)

    total_count = int(rows[0]["total_count"])
    lines.extend([
        f"- Descriptor samples: {total_count}",
        "",
        "| Centroid <= m | Extent <= ratio | Point count >= ratio | Would pass | Pass % |",
        "|--------------:|----------------:|---------------------:|-----------:|-------:|",
    ])
    for row in rows:
        pass_count = int(row["pass_count"])
        pass_pct = 100.0 * pass_count / total_count if total_count else 0.0
        lines.append(
            f"| {format_float(float(row['centroid_threshold']))} | "
            f"{format_float(float(row['extent_threshold']))} | "
            f"{format_float(float(row['point_ratio_threshold']))} | "
            f"{pass_count} | {pass_pct:.1f}% |"
        )
    lines.append("")
    return "\n".join(lines)


def format_consistency_sweep_markdown(
    samples: list[LoopStatusSample],
    topic: str,
    min_support_count: int = 1,
) -> str:
    summary = consistency_delta_summary(samples)
    rows = consistency_sweep_rows(samples, min_support_count)
    accepted_count = int(summary["accepted_count"])
    configured_support = max(1, int(min_support_count))
    lines = [
        "# MBES Loop Closure Consistency Threshold Sweep",
        "",
        f"- Topic: `{topic}`",
        f"- Samples: {len(samples)}",
        f"- Accepted loops with finite corrections: {accepted_count}",
        f"- Min support count: {configured_support}",
        "",
        "This report uses recorded correction poses when available, then falls "
        "back to scalar correction magnitudes for older bags. It is a tuning aid for "
        "`loop.consistency.max_correction_translation_delta_m` and "
        "`loop.consistency.max_correction_rotation_delta_rad`. Confirm loop "
        "geometry before copying thresholds into runtime config.",
        "",
    ]
    if not rows:
        lines.extend([
            "At least two accepted loop corrections with finite translation and "
            "rotation magnitudes are required for a consistency sweep.",
            "",
        ])
        return "\n".join(lines)

    pair_count = int(summary["pair_count"])
    pose_pair_count = int(summary["pose_pair_count"])
    magnitude_pair_count = int(summary["magnitude_pair_count"])
    lines.extend([
        "Runtime note: the first accepted loop bootstraps the consistency guard, "
        "so use trusted accepted-loop replays before enabling tight thresholds.",
        "The simulated support requirement is clamped to the retained accepted-loop "
        "history size during bootstrap, matching the runtime guard.",
        f"Delta source: {pose_pair_count} pose-aware pairs, "
        f"{magnitude_pair_count} scalar-magnitude fallback pairs.",
        "",
        "## Pairwise Accepted Correction Deltas",
        "",
        "| Metric | Count | Min | Median | P95 | Max |",
        "|--------|------:|----:|-------:|----:|----:|",
        format_stats("translation_delta_m", summary["translation_delta_m"]),
        format_stats("rotation_delta_rad", summary["rotation_delta_rad"]),
        "",
        "## Threshold Candidates",
        "",
        "| Translation delta <= m | Rotation delta <= rad | Would keep accepted | Keep % | Supported pairs | Pair % |",
        "|-----------------------:|----------------------:|--------------------:|-------:|----------------:|-------:|",
    ])
    for row in rows:
        supported_count = int(row["supported_count"])
        pair_support_count = int(row["pair_support_count"])
        keep_pct = 100.0 * supported_count / accepted_count if accepted_count else 0.0
        pair_pct = 100.0 * pair_support_count / pair_count if pair_count else 0.0
        lines.append(
            f"| {format_float(float(row['translation_threshold_m']))} | "
            f"{format_float(float(row['rotation_threshold_rad']))} | "
            f"{supported_count}/{accepted_count} | {keep_pct:.1f}% | "
            f"{pair_support_count}/{pair_count} | {pair_pct:.1f}% |"
        )
    lines.append("")
    return "\n".join(lines)


def format_consistency_rejection_audit_markdown(
    samples: list[LoopStatusSample],
    topic: str,
    limit: int = 50,
) -> str:
    rejections = [sample for sample in samples if is_consistency_rejection(sample)]
    diagnostic_count = sum(
        1 for sample in rejections
        if sample.consistency_required_support_count > 0 or
        math.isfinite(sample.consistency_nearest_translation_delta_m) or
        math.isfinite(sample.consistency_nearest_rotation_delta_rad)
    )
    deficits = Counter(support_deficit(sample) for sample in rejections)
    rows = consistency_rejection_samples(samples, limit)
    lines = [
        "# MBES Loop Closure Consistency Rejection Audit",
        "",
        f"- Topic: `{topic}`",
        f"- Samples: {len(samples)}",
        f"- Consistency rejections: {len(rejections)}",
        f"- Rejections with runtime diagnostics: {diagnostic_count}",
        f"- Rows shown: {len(rows)}",
        "",
    ]
    if not rejections:
        lines.extend([
            "No `loop consistency rejected` samples were found in this replay.",
            "",
        ])
        return "\n".join(lines)

    lines.extend([
        "Rows are sorted by largest support deficit, then by nearest correction "
        "delta. Use this report after enabling positive `loop.consistency.*` "
        "thresholds to distinguish a too-strict support count from a genuinely "
        "far correction outlier.",
        "",
        "## Support Deficits",
        "",
    ])
    for deficit, count in sorted(deficits.items()):
        lines.append(f"- required - support = {deficit}: {count}")
    lines.extend([
        "",
        "## Rejected Candidates",
        "",
        "| Time s | Current | Candidate | Support | Required | Deficit | "
        "Nearest trans m | Nearest rot rad | Correction trans m | "
        "Correction rot rad | Fitness |",
        "|-------:|--------:|----------:|--------:|---------:|--------:|"
        "----------------:|----------------:|-------------------:|"
        "-------------------:|--------:|",
    ])
    if not rows:
        lines.extend([
            "| n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |",
            "",
        ])
        return "\n".join(lines)

    for sample in rows:
        lines.append(
            f"| {format_float(sample.timestamp)} | "
            f"{sample.current_id} | {sample.candidate_id} | "
            f"{sample.consistency_support_count} | "
            f"{sample.consistency_required_support_count} | "
            f"{support_deficit(sample)} | "
            f"{format_float(sample.consistency_nearest_translation_delta_m)} | "
            f"{format_float(sample.consistency_nearest_rotation_delta_rad)} | "
            f"{format_float(sample.correction_translation_m)} | "
            f"{format_float(sample.correction_rotation_rad)} | "
            f"{format_float(sample.fitness_score)} |"
        )
    lines.append("")
    return "\n".join(lines)


def format_batch_candidate_row(
    rank: int,
    sample: LoopStatusSample,
    degree: int,
) -> str:
    return (
        f"| {rank} | {format_float(sample.timestamp)} | "
        f"{sample.current_id} | {sample.candidate_id} | "
        f"{int(sample.accepted)} | {degree} | "
        f"{format_float(sample.fitness_score)} | "
        f"{format_float(sample.correction_translation_m)} | "
        f"{format_float(sample.correction_rotation_rad)} | {sample.status} |"
    )


def format_batch_consistency_markdown(
    samples: list[LoopStatusSample],
    topic: str,
    translation_threshold_m: float = math.nan,
    rotation_threshold_rad: float = math.nan,
    auto_quantile: float = 0.95,
    exact_limit: int = 64,
    limit: int = 100,
) -> str:
    result = batch_consistency_selection(
        samples,
        translation_threshold_m,
        rotation_threshold_rad,
        auto_quantile,
        exact_limit,
    )
    selected = selected_index_set(result)
    rejected_indices = [
        index for index in range(len(result.candidates))
        if index not in selected
    ]
    rows_limit = max(0, int(limit))
    candidate_count = len(result.candidates)
    selected_count = len(result.selected_indices)
    rejected_count = len(rejected_indices)
    edge_pct = 100.0 * result.edge_count / result.pair_count if result.pair_count else 0.0
    lines = [
        "# MBES Loop Closure Batch Consistency Selection",
        "",
        f"- Topic: `{topic}`",
        f"- Samples: {len(samples)}",
        f"- Candidate loop corrections: {candidate_count}",
        f"- Selected consistent set: {selected_count}/{candidate_count}",
        f"- Batch-rejected candidates: {rejected_count}",
        f"- Translation threshold m: {format_float(result.translation_threshold_m)} "
        f"({result.translation_threshold_source})",
        f"- Rotation threshold rad: {format_float(result.rotation_threshold_rad)} "
        f"({result.rotation_threshold_source})",
        f"- Algorithm: {result.algorithm}",
        f"- Pairwise consistent edges: {result.edge_count}/{result.pair_count} "
        f"({edge_pct:.1f}%)",
        f"- Delta source: {result.pose_pair_count} pose-aware pairs, "
        f"{result.magnitude_pair_count} scalar-magnitude fallback pairs",
        "",
        "This PCM-like report builds a pairwise consistency graph from accepted "
        "and runtime-consistency-rejected loop corrections, then selects one "
        "internally consistent clique. It is an offline selection artifact for "
        "audit and replay planning; it is not a trajectory-accuracy claim by "
        "itself.",
        "",
    ]
    if not result.candidates:
        lines.extend([
            "No accepted or `loop consistency rejected` finite loop corrections "
            "were available for batch consistency selection.",
            "",
        ])
        return "\n".join(lines)

    if result.pair_count == 0:
        lines.extend([
            "At least two candidate loop corrections are required to build a "
            "pairwise consistency graph.",
            "",
        ])

    sorted_selected = sorted(
        result.selected_indices,
        key=lambda index: (
            -result.adjacency[index].bit_count(),
            result.candidates[index].timestamp,
            result.candidates[index].current_id,
        ),
    )
    sorted_rejected = sorted(
        rejected_indices,
        key=lambda index: (
            result.adjacency[index].bit_count(),
            result.candidates[index].timestamp,
            result.candidates[index].current_id,
        ),
    )
    lines.extend([
        "## Selected Loop IDs",
        "",
        "| Rank | Time s | Current | Candidate | Accepted | Degree | Fitness | Correction trans m | Correction rot rad | Status |",
        "|-----:|-------:|--------:|----------:|---------:|-------:|--------:|-------------------:|-------------------:|--------|",
    ])
    for rank, index in enumerate(sorted_selected[:rows_limit], start=1):
        lines.append(
            format_batch_candidate_row(
                rank,
                result.candidates[index],
                result.adjacency[index].bit_count(),
            )
        )
    if rows_limit < len(sorted_selected):
        lines.append(
            f"| ... | ... | ... | ... | ... | ... | ... | ... | ... | "
            f"{len(sorted_selected) - rows_limit} more selected rows omitted |"
        )
    lines.append("")

    lines.extend([
        "## Batch-Rejected Candidates",
        "",
        "| Rank | Time s | Current | Candidate | Accepted | Degree | Fitness | Correction trans m | Correction rot rad | Status |",
        "|-----:|-------:|--------:|----------:|---------:|-------:|--------:|-------------------:|-------------------:|--------|",
    ])
    if not sorted_rejected:
        lines.append("| n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a | none |")
    for rank, index in enumerate(sorted_rejected[:rows_limit], start=1):
        lines.append(
            format_batch_candidate_row(
                rank,
                result.candidates[index],
                result.adjacency[index].bit_count(),
            )
        )
    if rows_limit < len(sorted_rejected):
        lines.append(
            f"| ... | ... | ... | ... | ... | ... | ... | ... | ... | "
            f"{len(sorted_rejected) - rows_limit} more rejected rows omitted |"
        )
    lines.append("")
    return "\n".join(lines)


def format_optimization_diagnostics_markdown(
    optimization: OptimizationDiagnostics | None,
) -> list[str]:
    if optimization is None:
        return []
    latest_count = (
        "n/a" if optimization.latest_count is None else str(optimization.latest_count)
    )
    return [
        "## Pose Graph Optimization",
        "",
        f"- Count topic: `{optimization.count_topic}`",
        f"- Count samples: {len(optimization.count_samples)}",
        f"- Latest optimize count: {latest_count}",
        f"- Chi2 topic: `{optimization.chi2_topic}`",
        f"- Chi2 samples: {len(optimization.chi2_samples)}",
        f"- Latest active chi2: {format_float(optimization.latest_chi2)}",
        "",
    ]


def format_summary_markdown(
    summary: dict,
    topic: str,
    optimization: OptimizationDiagnostics | None = None,
) -> str:
    lines = [
        "# MBES Loop Closure Status Summary",
        "",
        f"- Topic: `{topic}`",
        f"- Samples: {summary['total']}",
        f"- Accepted: {summary['accepted']}",
        f"- Rejected: {summary['rejected']}",
        f"- No candidate: {summary['no_candidate']}",
        f"- Converged registrations: {summary['converged']}",
        "",
    ]
    lines.extend(format_optimization_diagnostics_markdown(optimization))
    lines.extend([
        "## Numeric Distributions",
        "",
        "| Metric | Count | Min | Median | P95 | Max |",
        "|--------|------:|----:|-------:|----:|----:|",
        format_stats("fitness_score", summary["fitness"]),
        format_stats("accepted fitness_score", summary["accepted_fitness"]),
        format_stats("correction_translation_m", summary["correction_translation_m"]),
        format_stats("correction_rotation_rad", summary["correction_rotation_rad"]),
        format_stats(
            "descriptor_centroid_distance_m",
            summary["descriptor_centroid_distance_m"],
        ),
        format_stats("descriptor_extent_ratio", summary["descriptor_extent_ratio"]),
        format_stats(
            "descriptor_point_count_ratio",
            summary["descriptor_point_count_ratio"],
        ),
        format_stats(
            "consistency_nearest_translation_delta_m",
            summary["consistency_nearest_translation_delta_m"],
        ),
        format_stats(
            "consistency_nearest_rotation_delta_rad",
            summary["consistency_nearest_rotation_delta_rad"],
        ),
        "",
        "## Rejection Reasons",
        "",
    ])
    if summary["rejection_counts"]:
        for reason, count in summary["rejection_counts"].most_common():
            lines.append(f"- {reason}: {count}")
    else:
        lines.append("- none")
    lines.extend(["", "## All Status Values", ""])
    if summary["status_counts"]:
        for status, count in summary["status_counts"].most_common():
            lines.append(f"- {status}: {count}")
    else:
        lines.append("- none")
    lines.append("")
    return "\n".join(lines)


def write_csv(path: Path, samples: list[LoopStatusSample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for sample in samples:
            writer.writerow({
                "timestamp": f"{sample.timestamp:.9f}",
                "current_keyframe_timestamp": (
                    f"{current_keyframe_timestamp(sample):.9f}"
                ),
                "candidate_keyframe_timestamp": f"{sample.candidate_keyframe_timestamp:.9f}",
                "frame_id": sample.frame_id,
                "current_id": sample.current_id,
                "candidate_id": sample.candidate_id,
                "accepted": int(sample.accepted),
                "converged": int(sample.converged),
                "fitness_score": f"{sample.fitness_score:.9f}",
                "correction_translation_m": f"{sample.correction_translation_m:.9f}",
                "correction_rotation_rad": f"{sample.correction_rotation_rad:.9f}",
                "correction_pose_valid": int(sample.correction_pose_valid),
                "correction_x_m": f"{sample.correction_x_m:.9f}",
                "correction_y_m": f"{sample.correction_y_m:.9f}",
                "correction_z_m": f"{sample.correction_z_m:.9f}",
                "correction_qx": f"{sample.correction_qx:.9f}",
                "correction_qy": f"{sample.correction_qy:.9f}",
                "correction_qz": f"{sample.correction_qz:.9f}",
                "correction_qw": f"{sample.correction_qw:.9f}",
                "consistency_support_count": sample.consistency_support_count,
                "consistency_required_support_count": (
                    sample.consistency_required_support_count
                ),
                "consistency_nearest_translation_delta_m": (
                    f"{sample.consistency_nearest_translation_delta_m:.9f}"
                ),
                "consistency_nearest_rotation_delta_rad": (
                    f"{sample.consistency_nearest_rotation_delta_rad:.9f}"
                ),
                "descriptor_centroid_distance_m": (
                    f"{sample.descriptor_centroid_distance_m:.9f}"
                ),
                "descriptor_extent_ratio": f"{sample.descriptor_extent_ratio:.9f}",
                "descriptor_point_count_ratio": (
                    f"{sample.descriptor_point_count_ratio:.9f}"
                ),
                "status": sample.status,
            })


def write_batch_consistency_selected_csv(
    path: Path,
    samples: list[LoopStatusSample],
    translation_threshold_m: float = math.nan,
    rotation_threshold_rad: float = math.nan,
    auto_quantile: float = 0.95,
    exact_limit: int = 64,
) -> None:
    result = batch_consistency_selection(
        samples,
        translation_threshold_m,
        rotation_threshold_rad,
        auto_quantile,
        exact_limit,
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "timestamp",
        "current_keyframe_timestamp",
        "candidate_keyframe_timestamp",
        "current_id",
        "candidate_id",
        "accepted",
        "degree",
        "fitness_score",
        "correction_translation_m",
        "correction_rotation_rad",
        "status",
    ]
    sorted_indices = sorted(
        result.selected_indices,
        key=lambda index: (
            result.candidates[index].timestamp,
            result.candidates[index].current_id,
            result.candidates[index].candidate_id,
        ),
    )
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=fields)
        writer.writeheader()
        for index in sorted_indices:
            sample = result.candidates[index]
            writer.writerow({
                "timestamp": f"{sample.timestamp:.9f}",
                "current_keyframe_timestamp": (
                    f"{current_keyframe_timestamp(sample):.9f}"
                ),
                "candidate_keyframe_timestamp": (
                    f"{sample.candidate_keyframe_timestamp:.9f}"
                ),
                "current_id": sample.current_id,
                "candidate_id": sample.candidate_id,
                "accepted": int(sample.accepted),
                "degree": result.adjacency[index].bit_count(),
                "fitness_score": f"{sample.fitness_score:.9f}",
                "correction_translation_m": f"{sample.correction_translation_m:.9f}",
                "correction_rotation_rad": f"{sample.correction_rotation_rad:.9f}",
                "status": sample.status,
            })


def default_ros2_typestore():
    try:
        from rosbags.typesys import Stores, get_typestore
    except ImportError as e:
        raise RuntimeError(
            "missing dependency: install rosbags type stores to read rosbag2 files"
        ) from e

    for store_name in ("ROS2_HUMBLE", "ROS2_JAZZY", "ROS2_IRON"):
        store = getattr(Stores, store_name, None)
        if store is not None:
            return get_typestore(store)
    raise RuntimeError("rosbags does not provide a supported ROS 2 typestore")


def open_reader_with_typestore_fallback(any_reader, bag_dir: Path):
    reader = any_reader([bag_dir])
    try:
        reader.open()
        return reader
    except Exception as e:
        if "no type definitions" not in str(e).lower():
            raise

    reader = any_reader([bag_dir], default_typestore=default_ros2_typestore())
    reader.open()
    return reader


def deserialize_status_message(reader, raw, msgtype: str):
    try:
        return reader.deserialize(raw, msgtype)
    except Exception:
        if msgtype != "aqua_msgs/msg/LoopClosureStatus":
            raise

    try:
        from aqua_msgs.msg import LoopClosureStatus
        from rclpy.serialization import deserialize_message
    except ImportError as e:
        raise RuntimeError(
            "cannot deserialize aqua_msgs/msg/LoopClosureStatus; source the ROS 2 "
            "workspace or record bags with embedded type definitions"
        ) from e
    return deserialize_message(raw, LoopClosureStatus)


def deserialize_keyframe_message(reader, raw, msgtype: str):
    try:
        return reader.deserialize(raw, msgtype)
    except Exception:
        if msgtype != "aqua_msgs/msg/PoseGraphKeyframe":
            raise

    try:
        from aqua_msgs.msg import PoseGraphKeyframe
        from rclpy.serialization import deserialize_message
    except ImportError as e:
        raise RuntimeError(
            "cannot deserialize aqua_msgs/msg/PoseGraphKeyframe; source the ROS 2 "
            "workspace or record bags with embedded type definitions"
        ) from e
    return deserialize_message(raw, PoseGraphKeyframe)


def deserialize_scalar_message(reader, raw, msgtype: str):
    try:
        return reader.deserialize(raw, msgtype)
    except Exception:
        if msgtype == "std_msgs/msg/UInt32":
            from rclpy.serialization import deserialize_message
            from std_msgs.msg import UInt32
            return deserialize_message(raw, UInt32)
        if msgtype == "std_msgs/msg/Float64":
            from rclpy.serialization import deserialize_message
            from std_msgs.msg import Float64
            return deserialize_message(raw, Float64)
        raise


def read_bag_samples(bag: Path, topic: str) -> list[LoopStatusSample]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        raise RuntimeError("missing dependency: install rosbags to read rosbag2 files") from e

    bag_dir = bag if bag.is_dir() else bag.parent
    if not bag_dir.is_dir():
        raise RuntimeError(f"not a rosbag2 directory: {bag_dir}")

    samples: list[LoopStatusSample] = []
    reader = open_reader_with_typestore_fallback(AnyReader, bag_dir)
    try:
        wanted = [connection for connection in reader.connections if connection.topic == topic]
        if not wanted:
            available = ", ".join(sorted({connection.topic for connection in reader.connections}))
            raise RuntimeError(f"topic {topic!r} not found. Available topics: {available}")
        for connection, t_ns, raw in reader.messages(connections=wanted):
            try:
                msg = deserialize_status_message(reader, raw, connection.msgtype)
            except RuntimeError:
                raise
            except Exception:
                continue
            samples.append(sample_from_msg(msg, t_ns * 1.0e-9))
    finally:
        reader.close()
    return samples


def read_bag_keyframe_stamps(bag: Path, topic: str) -> dict[int, float]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        raise RuntimeError("missing dependency: install rosbags to read rosbag2 files") from e

    bag_dir = bag if bag.is_dir() else bag.parent
    if not bag_dir.is_dir():
        raise RuntimeError(f"not a rosbag2 directory: {bag_dir}")

    stamps: dict[int, float] = {}
    reader = open_reader_with_typestore_fallback(AnyReader, bag_dir)
    try:
        wanted = [connection for connection in reader.connections if connection.topic == topic]
        if not wanted:
            return stamps
        for connection, t_ns, raw in reader.messages(connections=wanted):
            try:
                msg = deserialize_keyframe_message(reader, raw, connection.msgtype)
            except Exception:
                continue
            stamp = stamp_to_seconds(msg.header.stamp)
            if stamp <= 0.0:
                stamp = t_ns * 1.0e-9
            stamps[int(msg.id)] = stamp
    finally:
        reader.close()
    return stamps


def enrich_samples_with_keyframe_stamps(
    samples: list[LoopStatusSample],
    keyframe_stamps: dict[int, float],
) -> list[LoopStatusSample]:
    if not keyframe_stamps:
        return samples
    enriched = []
    for sample in samples:
        current_stamp = keyframe_stamps.get(sample.current_id, sample.timestamp)
        candidate_stamp = (
            math.nan
            if sample.candidate_id == NO_CANDIDATE_ID
            else keyframe_stamps.get(sample.candidate_id, math.nan)
        )
        enriched.append(
            replace(
                sample,
                current_keyframe_timestamp=current_stamp,
                candidate_keyframe_timestamp=candidate_stamp,
            )
        )
    return enriched


def read_bag_scalar_samples(bag: Path, topic: str) -> list[TopicValueSample]:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as e:
        raise RuntimeError("missing dependency: install rosbags to read rosbag2 files") from e

    bag_dir = bag if bag.is_dir() else bag.parent
    if not bag_dir.is_dir():
        raise RuntimeError(f"not a rosbag2 directory: {bag_dir}")

    samples: list[TopicValueSample] = []
    reader = open_reader_with_typestore_fallback(AnyReader, bag_dir)
    try:
        wanted = [connection for connection in reader.connections if connection.topic == topic]
        if not wanted:
            return samples
        for connection, t_ns, raw in reader.messages(connections=wanted):
            try:
                msg = deserialize_scalar_message(reader, raw, connection.msgtype)
            except Exception:
                continue
            value = getattr(msg, "data", math.nan)
            try:
                value = float(value)
            except (TypeError, ValueError):
                continue
            samples.append(TopicValueSample(t_ns * 1.0e-9, value))
    finally:
        reader.close()
    return samples


def read_optimization_diagnostics(
    bag: Path,
    count_topic: str,
    chi2_topic: str,
) -> OptimizationDiagnostics:
    return OptimizationDiagnostics(
        count_topic=count_topic,
        chi2_topic=chi2_topic,
        count_samples=read_bag_scalar_samples(bag, count_topic),
        chi2_samples=read_bag_scalar_samples(bag, chi2_topic),
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path,
                        help="rosbag2 directory or .mcap file")
    parser.add_argument("--out", required=True, type=Path,
                        help="Output CSV path")
    parser.add_argument("--topic", default="/mbes_loop_closure/status",
                        help="LoopClosureStatus topic")
    parser.add_argument("--keyframe-topic", default="/aqua_pose_graph/keyframe",
                        help="Optional PoseGraphKeyframe topic used to add stable "
                        "candidate/current keyframe timestamps to CSV outputs")
    parser.add_argument("--optimization-count-topic",
                        default="/aqua_pose_graph/optimization_count",
                        help="Optional std_msgs/UInt32 pose-graph optimize-count topic")
    parser.add_argument("--optimization-chi2-topic",
                        default="/aqua_pose_graph/optimization_chi2",
                        help="Optional std_msgs/Float64 pose-graph active-chi2 topic")
    parser.add_argument("--summary-out", type=Path,
                        help="Optional markdown summary output path")
    parser.add_argument("--descriptor-sweep-out", type=Path,
                        help="Optional descriptor threshold sweep markdown output path")
    parser.add_argument("--consistency-sweep-out", type=Path,
                        help="Optional consistency threshold sweep markdown output path")
    parser.add_argument("--consistency-rejection-audit-out", type=Path,
                        help="Optional runtime consistency rejection audit markdown output path")
    parser.add_argument("--consistency-rejection-audit-limit", type=int, default=50,
                        help="Maximum rows to include in the consistency rejection audit")
    parser.add_argument("--consistency-min-support-count", type=int, default=1,
                        help="Support count to simulate in the consistency sweep")
    parser.add_argument("--batch-consistency-out", type=Path,
                        help="Optional batch pairwise-consistency selection markdown output path")
    parser.add_argument("--batch-consistency-selected-csv-out", type=Path,
                        help="Optional selected loop-id CSV for loop.selection.allowlist_csv replay")
    parser.add_argument("--batch-consistency-translation-threshold-m",
                        type=float, default=math.nan,
                        help="Pairwise translation threshold for batch consistency; "
                        "omit to use the auto quantile")
    parser.add_argument("--batch-consistency-rotation-threshold-rad",
                        type=float, default=math.nan,
                        help="Pairwise rotation threshold for batch consistency; "
                        "omit to use the auto quantile")
    parser.add_argument("--batch-consistency-auto-quantile",
                        type=float, default=0.95,
                        help="Delta quantile used when a batch consistency threshold is omitted")
    parser.add_argument("--batch-consistency-exact-limit", type=int, default=64,
                        help="Use exact maximum-clique search up to this candidate count")
    parser.add_argument("--batch-consistency-limit", type=int, default=100,
                        help="Maximum selected/rejected rows to include in the batch consistency report")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        samples = read_bag_samples(args.bag, args.topic)
        samples = enrich_samples_with_keyframe_stamps(
            samples,
            read_bag_keyframe_stamps(args.bag, args.keyframe_topic),
        )
        optimization = read_optimization_diagnostics(
            args.bag,
            args.optimization_count_topic,
            args.optimization_chi2_topic,
        )
    except RuntimeError as e:
        sys.stderr.write(f"{e}\n")
        return 1

    write_csv(args.out, samples)
    summary_text = format_summary_markdown(
        summarize(samples),
        args.topic,
        optimization,
    )
    if args.summary_out:
        args.summary_out.parent.mkdir(parents=True, exist_ok=True)
        args.summary_out.write_text(summary_text, encoding="utf-8")
    if args.descriptor_sweep_out:
        args.descriptor_sweep_out.parent.mkdir(parents=True, exist_ok=True)
        args.descriptor_sweep_out.write_text(
            format_descriptor_sweep_markdown(samples, args.topic),
            encoding="utf-8",
        )
    if args.consistency_sweep_out:
        args.consistency_sweep_out.parent.mkdir(parents=True, exist_ok=True)
        args.consistency_sweep_out.write_text(
            format_consistency_sweep_markdown(
                samples,
                args.topic,
                args.consistency_min_support_count,
            ),
            encoding="utf-8",
        )
    if args.consistency_rejection_audit_out:
        args.consistency_rejection_audit_out.parent.mkdir(parents=True, exist_ok=True)
        args.consistency_rejection_audit_out.write_text(
            format_consistency_rejection_audit_markdown(
                samples,
                args.topic,
                args.consistency_rejection_audit_limit,
            ),
            encoding="utf-8",
        )
    if args.batch_consistency_out:
        args.batch_consistency_out.parent.mkdir(parents=True, exist_ok=True)
        args.batch_consistency_out.write_text(
            format_batch_consistency_markdown(
                samples,
                args.topic,
                args.batch_consistency_translation_threshold_m,
                args.batch_consistency_rotation_threshold_rad,
                args.batch_consistency_auto_quantile,
                args.batch_consistency_exact_limit,
                args.batch_consistency_limit,
            ),
            encoding="utf-8",
        )
    if args.batch_consistency_selected_csv_out:
        write_batch_consistency_selected_csv(
            args.batch_consistency_selected_csv_out,
            samples,
            args.batch_consistency_translation_threshold_m,
            args.batch_consistency_rotation_threshold_rad,
            args.batch_consistency_auto_quantile,
            args.batch_consistency_exact_limit,
        )
    print(summary_text)
    print(f"wrote {len(samples)} samples to {args.out}", file=sys.stderr)
    if args.summary_out:
        print(f"wrote summary to {args.summary_out}", file=sys.stderr)
    if args.descriptor_sweep_out:
        print(f"wrote descriptor sweep to {args.descriptor_sweep_out}", file=sys.stderr)
    if args.consistency_sweep_out:
        print(
            f"wrote consistency sweep to {args.consistency_sweep_out}",
            file=sys.stderr,
        )
    if args.consistency_rejection_audit_out:
        print(
            f"wrote consistency rejection audit to "
            f"{args.consistency_rejection_audit_out}",
            file=sys.stderr,
        )
    if args.batch_consistency_out:
        print(
            f"wrote batch consistency selection to {args.batch_consistency_out}",
            file=sys.stderr,
        )
    if args.batch_consistency_selected_csv_out:
        print(
            f"wrote batch selected loop CSV to "
            f"{args.batch_consistency_selected_csv_out}",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
