#!/usr/bin/env python3
"""Evaluate descriptor retrieval recall for MBES selected-loop replay candidates."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys


NO_CANDIDATE_ID = 2**32 - 1


@dataclass(frozen=True)
class LoopSignature:
    timestamp_s: float
    current_keyframe_timestamp_s: float | None
    candidate_keyframe_timestamp_s: float | None
    current_id: int
    candidate_id: int
    descriptor_centroid_distance_m: float | None
    descriptor_extent_ratio: float | None
    descriptor_point_count_ratio: float | None

    @property
    def pair(self) -> tuple[int, int]:
        return (self.candidate_id, self.current_id)

    @property
    def query_time_s(self) -> float:
        return (
            self.current_keyframe_timestamp_s
            if self.current_keyframe_timestamp_s is not None
            else self.timestamp_s
        )


@dataclass(frozen=True)
class StatusCandidate:
    timestamp_s: float
    current_keyframe_timestamp_s: float | None
    candidate_keyframe_timestamp_s: float | None
    current_id: int
    candidate_id: int
    accepted: bool
    status: str
    fitness_score: str
    correction_translation_m: str
    correction_rotation_rad: str
    descriptor_centroid_distance_m: float | None
    descriptor_extent_ratio: float | None
    descriptor_point_count_ratio: float | None

    @property
    def pair(self) -> tuple[int, int]:
        return (self.candidate_id, self.current_id)

    @property
    def has_candidate(self) -> bool:
        return self.candidate_id != NO_CANDIDATE_ID

    @property
    def candidate_time_s(self) -> float | None:
        return self.candidate_keyframe_timestamp_s

    @property
    def query_time_s(self) -> float:
        return (
            self.current_keyframe_timestamp_s
            if self.current_keyframe_timestamp_s is not None
            else self.timestamp_s
        )


@dataclass(frozen=True)
class RankedCandidate:
    row: StatusCandidate
    score: float
    current_dt_s: float
    candidate_dt_s: float | None
    centroid_delta_m: float
    extent_delta: float
    point_ratio_delta: float
    exact_match: bool
    endpoint_match: bool


@dataclass(frozen=True)
class RetrievalResult:
    signature: LoopSignature
    candidates: list[RankedCandidate]

    @property
    def exact_rank(self) -> int | None:
        for index, candidate in enumerate(self.candidates, start=1):
            if candidate.exact_match:
                return index
        return None

    @property
    def endpoint_rank(self) -> int | None:
        for index, candidate in enumerate(self.candidates, start=1):
            if candidate.endpoint_match:
                return index
        return None


def truthy(value: str | None) -> bool:
    return value in {"1", "true", "True", "TRUE", "yes", "Yes"}


def optional_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def optional_int(value: str | None) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def read_selected(path: Path) -> list[LoopSignature]:
    signatures: list[LoopSignature] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"timestamp", "current_id", "candidate_id"} - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            timestamp_s = optional_float(row.get("timestamp"))
            current_id = optional_int(row.get("current_id"))
            candidate_id = optional_int(row.get("candidate_id"))
            if timestamp_s is None or current_id is None or candidate_id is None:
                raise ValueError(f"{path}:{line_number}: invalid selected loop row")
            current_time = optional_float(row.get("current_keyframe_timestamp"))
            signatures.append(
                LoopSignature(
                    timestamp_s=timestamp_s,
                    current_keyframe_timestamp_s=(
                        current_time if current_time is not None else timestamp_s
                    ),
                    candidate_keyframe_timestamp_s=optional_float(
                        row.get("candidate_keyframe_timestamp")
                    ),
                    current_id=current_id,
                    candidate_id=candidate_id,
                    descriptor_centroid_distance_m=optional_float(
                        row.get("descriptor_centroid_distance_m")
                    ),
                    descriptor_extent_ratio=optional_float(
                        row.get("descriptor_extent_ratio")
                    ),
                    descriptor_point_count_ratio=optional_float(
                        row.get("descriptor_point_count_ratio")
                    ),
                )
            )
    return signatures


def read_status(path: Path) -> list[StatusCandidate]:
    rows: list[StatusCandidate] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"timestamp", "current_id", "candidate_id", "accepted", "status"} - set(
            reader.fieldnames
        )
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            timestamp_s = optional_float(row.get("timestamp"))
            current_id = optional_int(row.get("current_id"))
            candidate_id = optional_int(row.get("candidate_id"))
            if timestamp_s is None or current_id is None or candidate_id is None:
                raise ValueError(f"{path}:{line_number}: invalid status row")
            status = row.get("status", "")
            rows.append(
                StatusCandidate(
                    timestamp_s=timestamp_s,
                    current_keyframe_timestamp_s=optional_float(
                        row.get("current_keyframe_timestamp")
                    ),
                    candidate_keyframe_timestamp_s=optional_float(
                        row.get("candidate_keyframe_timestamp")
                    ),
                    current_id=current_id,
                    candidate_id=candidate_id,
                    accepted=truthy(row.get("accepted")) or status == "accepted",
                    status=status,
                    fitness_score=row.get("fitness_score", ""),
                    correction_translation_m=row.get("correction_translation_m", ""),
                    correction_rotation_rad=row.get("correction_rotation_rad", ""),
                    descriptor_centroid_distance_m=optional_float(
                        row.get("descriptor_centroid_distance_m")
                    ),
                    descriptor_extent_ratio=optional_float(
                        row.get("descriptor_extent_ratio")
                    ),
                    descriptor_point_count_ratio=optional_float(
                        row.get("descriptor_point_count_ratio")
                    ),
                )
            )
    return rows


def finite_descriptor(signature: LoopSignature | StatusCandidate) -> bool:
    return (
        signature.descriptor_centroid_distance_m is not None
        and signature.descriptor_extent_ratio is not None
        and signature.descriptor_point_count_ratio is not None
    )


def endpoint_matches(
    signature: LoopSignature,
    row: StatusCandidate,
    candidate_timestamp_window_s: float,
) -> bool:
    if signature.candidate_keyframe_timestamp_s is None or row.candidate_time_s is None:
        return False
    return (
        abs(row.candidate_time_s - signature.candidate_keyframe_timestamp_s)
        <= candidate_timestamp_window_s
    )


def fitness_sort_value(value: str) -> float:
    parsed = optional_float(value)
    return parsed if parsed is not None else math.inf


def rank_candidates(
    signature: LoopSignature,
    status_rows: list[StatusCandidate],
    current_timestamp_window_s: float,
    candidate_timestamp_window_s: float,
    centroid_scale_m: float,
    extent_scale: float,
    point_ratio_scale: float,
) -> list[RankedCandidate]:
    if current_timestamp_window_s < 0.0 or candidate_timestamp_window_s < 0.0:
        raise ValueError("timestamp windows must be non-negative")
    if min(centroid_scale_m, extent_scale, point_ratio_scale) <= 0.0:
        raise ValueError("descriptor scales must be positive")
    if not finite_descriptor(signature):
        return []

    ranked: list[RankedCandidate] = []
    for row in status_rows:
        if not row.has_candidate or not finite_descriptor(row):
            continue
        current_dt_s = row.query_time_s - signature.query_time_s
        if abs(current_dt_s) > current_timestamp_window_s:
            continue
        centroid_delta = abs(
            row.descriptor_centroid_distance_m
            - signature.descriptor_centroid_distance_m
        )
        extent_delta = abs(row.descriptor_extent_ratio - signature.descriptor_extent_ratio)
        point_ratio_delta = abs(
            row.descriptor_point_count_ratio
            - signature.descriptor_point_count_ratio
        )
        score = (
            centroid_delta / centroid_scale_m
            + extent_delta / extent_scale
            + point_ratio_delta / point_ratio_scale
        )
        candidate_dt_s = (
            None if row.candidate_time_s is None
            or signature.candidate_keyframe_timestamp_s is None
            else row.candidate_time_s - signature.candidate_keyframe_timestamp_s
        )
        ranked.append(
            RankedCandidate(
                row=row,
                score=score,
                current_dt_s=current_dt_s,
                candidate_dt_s=candidate_dt_s,
                centroid_delta_m=centroid_delta,
                extent_delta=extent_delta,
                point_ratio_delta=point_ratio_delta,
                exact_match=row.pair == signature.pair,
                endpoint_match=endpoint_matches(
                    signature, row, candidate_timestamp_window_s
                ),
            )
        )
    ranked.sort(
        key=lambda candidate: (
            candidate.score,
            abs(candidate.current_dt_s),
            fitness_sort_value(candidate.row.fitness_score),
            candidate.row.candidate_id,
            candidate.row.current_id,
        )
    )
    return ranked


def evaluate_retrieval(
    signatures: list[LoopSignature],
    status_rows: list[StatusCandidate],
    current_timestamp_window_s: float,
    candidate_timestamp_window_s: float,
    centroid_scale_m: float,
    extent_scale: float,
    point_ratio_scale: float,
) -> list[RetrievalResult]:
    return [
        RetrievalResult(
            signature=signature,
            candidates=rank_candidates(
                signature,
                status_rows,
                current_timestamp_window_s,
                candidate_timestamp_window_s,
                centroid_scale_m,
                extent_scale,
                point_ratio_scale,
            ),
        )
        for signature in signatures
    ]


def parse_top_k(text: str) -> list[int]:
    values: list[int] = []
    for part in text.split(","):
        stripped = part.strip()
        if not stripped:
            continue
        value = int(stripped)
        if value <= 0:
            raise ValueError("top-k values must be positive")
        values.append(value)
    return sorted(set(values))


def format_float(value: float | None, digits: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def rank_text(rank: int | None) -> str:
    return str(rank) if rank is not None else "miss"


def pair_text(candidate_id: int, current_id: int) -> str:
    return f"{candidate_id} -> {current_id}"


def recall_line(label: str, ranks: list[int | None], top_k: list[int]) -> list[str]:
    ranked_positives = sum(1 for rank in ranks if rank is not None)
    lines = [f"- {label} queries with ranked positive: {ranked_positives}/{len(ranks)}"]
    if not ranks:
        return lines
    for k in top_k:
        hits = sum(1 for rank in ranks if rank is not None and rank <= k)
        denominator = len(ranks)
        lines.append(f"- {label} recall@{k}: {hits}/{len(ranks)} ({hits / denominator:.1%})")
    return lines


def format_report(
    selected_path: Path,
    status_path: Path,
    results: list[RetrievalResult],
    top_k: list[int],
    current_timestamp_window_s: float,
    candidate_timestamp_window_s: float,
    centroid_scale_m: float,
    extent_scale: float,
    point_ratio_scale: float,
    max_rows: int,
    top_examples: int,
) -> str:
    if max_rows < 0 or top_examples < 0:
        raise ValueError("max rows and top examples must be non-negative")

    exact_ranks = [result.exact_rank for result in results]
    endpoint_ranks = [result.endpoint_rank for result in results]
    query_with_candidates = sum(1 for result in results if result.candidates)
    query_with_descriptors = sum(1 for result in results if finite_descriptor(result.signature))
    candidate_counts = [len(result.candidates) for result in results]

    lines = [
        "# MBES Descriptor Retrieval Evaluation",
        "",
        f"- Selected loop CSV: `{selected_path}`",
        f"- Replay status CSV: `{status_path}`",
        f"- Queries: {len(results)}",
        f"- Queries with descriptor signatures: {query_with_descriptors}",
        f"- Queries with ranked replay candidates: {query_with_candidates}",
        f"- Current timestamp window: {format_float(current_timestamp_window_s)} s",
        f"- Candidate endpoint timestamp window: {format_float(candidate_timestamp_window_s)} s",
        f"- Descriptor score scales: centroid={format_float(centroid_scale_m)} m, "
        f"extent={format_float(extent_scale)}, point_ratio={format_float(point_ratio_scale)}",
        f"- Candidate rows per query: min={min(candidate_counts, default=0)}, "
        f"median={format_float(sorted(candidate_counts)[len(candidate_counts) // 2] if candidate_counts else 0, 0)}, "
        f"max={max(candidate_counts, default=0)}",
        "",
        "## Recall",
        "",
    ]
    lines.extend(recall_line("Exact ID", exact_ranks, top_k))
    lines.extend(recall_line("Endpoint timestamp", endpoint_ranks, top_k))
    lines.extend([
        "",
        "Endpoint recall is the more useful drift-aware target when replay keyframe IDs "
        "change; exact-ID recall is retained as a strict determinism check.",
        "",
        "## Query Ranking",
        "",
        "| Rank | Source time s | Selected pair | Candidates | Exact rank | Endpoint rank | Top candidate | Score | dt current s | dt candidate s | Descriptor deltas | Status |",
        "|-----:|--------------:|---------------|-----------:|-----------:|--------------:|---------------|------:|-------------:|---------------:|-------------------|--------|",
    ])
    for rank, result in enumerate(results[:max_rows], start=1):
        top = result.candidates[0] if result.candidates else None
        top_deltas = (
            f"{format_float(top.centroid_delta_m)}/{format_float(top.extent_delta)}/"
            f"{format_float(top.point_ratio_delta)}"
            if top
            else "n/a"
        )
        lines.append(
            f"| {rank} | {format_float(result.signature.timestamp_s)} | "
            f"{pair_text(result.signature.candidate_id, result.signature.current_id)} | "
            f"{len(result.candidates)} | {rank_text(result.exact_rank)} | "
            f"{rank_text(result.endpoint_rank)} | "
            f"{pair_text(top.row.candidate_id, top.row.current_id) if top else 'n/a'} | "
            f"{format_float(top.score) if top else 'n/a'} | "
            f"{format_float(top.current_dt_s) if top else 'n/a'} | "
            f"{format_float(top.candidate_dt_s) if top else 'n/a'} | "
            f"{top_deltas} | "
            f"{top.row.status if top else 'n/a'} |"
        )
    if len(results) > max_rows:
        lines.extend(["", f"Only the first {max_rows} queries are shown."])

    lines.extend(["", "## Top Candidates", ""])
    for index, result in enumerate(results[:max_rows], start=1):
        lines.extend([
            f"### Query {index}: {pair_text(result.signature.candidate_id, result.signature.current_id)}",
            "",
            "| Rank | Candidate -> Current | Score | dt current s | dt candidate s | Descriptor deltas | Exact | Endpoint | Status |",
            "|-----:|----------------------|------:|-------------:|---------------:|-------------------|-------|----------|--------|",
        ])
        for rank, candidate in enumerate(result.candidates[:top_examples], start=1):
            lines.append(
                f"| {rank} | {pair_text(candidate.row.candidate_id, candidate.row.current_id)} | "
                f"{format_float(candidate.score)} | {format_float(candidate.current_dt_s)} | "
                f"{format_float(candidate.candidate_dt_s)} | "
                f"{format_float(candidate.centroid_delta_m)}/"
                f"{format_float(candidate.extent_delta)}/"
                f"{format_float(candidate.point_ratio_delta)} | "
                f"{int(candidate.exact_match)} | {int(candidate.endpoint_match)} | "
                f"{candidate.row.status} |"
            )
        if not result.candidates:
            lines.append("| 1 | n/a | n/a | n/a | n/a | n/a | 0 | 0 | no ranked candidates |")
        lines.append("")
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selected", required=True, type=Path,
                        help="Selected loop CSV with descriptor signature columns")
    parser.add_argument("--status", required=True, type=Path,
                        help="Replay mbes_beach_pond_loop_status.csv candidate database")
    parser.add_argument("--out", type=Path, help="Optional Markdown output path")
    parser.add_argument("--current-timestamp-window-s", type=float, default=5.0,
                        help="Current-endpoint window for candidate retrieval")
    parser.add_argument("--candidate-timestamp-window-s", type=float, default=1.0,
                        help="Candidate-endpoint window used as drift-aware positive label")
    parser.add_argument("--descriptor-centroid-scale-m", type=float, default=0.6,
                        help="Descriptor centroid delta normalization scale")
    parser.add_argument("--descriptor-extent-scale", type=float, default=0.15,
                        help="Descriptor extent-ratio delta normalization scale")
    parser.add_argument("--descriptor-point-ratio-scale", type=float, default=0.10,
                        help="Descriptor point-count-ratio delta normalization scale")
    parser.add_argument("--top-k", default="1,3,5,10",
                        help="Comma-separated recall cutoffs")
    parser.add_argument("--max-rows", type=int, default=50,
                        help="Maximum query rows to show")
    parser.add_argument("--top-examples", type=int, default=5,
                        help="Top ranked candidates to show per query")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        top_k = parse_top_k(args.top_k)
        signatures = read_selected(args.selected)
        status_rows = read_status(args.status)
        results = evaluate_retrieval(
            signatures,
            status_rows,
            args.current_timestamp_window_s,
            args.candidate_timestamp_window_s,
            args.descriptor_centroid_scale_m,
            args.descriptor_extent_scale,
            args.descriptor_point_ratio_scale,
        )
        report = format_report(
            args.selected,
            args.status,
            results,
            top_k,
            args.current_timestamp_window_s,
            args.candidate_timestamp_window_s,
            args.descriptor_centroid_scale_m,
            args.descriptor_extent_scale,
            args.descriptor_point_ratio_scale,
            args.max_rows,
            args.top_examples,
        )
    except (OSError, ValueError) as exc:
        print(f"failed to evaluate MBES descriptor retrieval: {exc}", file=sys.stderr)
        return 1

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
        print(f"wrote MBES descriptor retrieval evaluation to {args.out}")
    else:
        print(report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
