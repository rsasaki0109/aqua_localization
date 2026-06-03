#!/usr/bin/env python3
"""Audit whether a selected-loop replay accepted only allowlisted loop IDs."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys


@dataclass(frozen=True)
class LoopStatusRow:
    timestamp: str
    current_id: int
    candidate_id: int
    accepted: bool
    status: str
    fitness_score: str
    correction_translation_m: str
    correction_rotation_rad: str


@dataclass(frozen=True)
class LoopSignature:
    timestamp_s: float
    fitness_score: float | None
    correction_translation_m: float | None
    correction_rotation_rad: float | None


@dataclass(frozen=True)
class LoopAllowlist:
    pairs: set[tuple[int, int]]
    signatures: list[LoopSignature]


@dataclass(frozen=True)
class SignatureMatchOptions:
    timestamp_window_s: float = 0.0
    max_fitness_delta: float = 0.0
    max_translation_delta_m: float = 0.0
    max_rotation_delta_rad: float = 0.0


def truthy(value: str | None) -> bool:
    return value in {"1", "true", "True", "TRUE", "yes", "Yes"}


def parse_optional_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        parsed = float(value)
    except ValueError:
        return None
    return parsed if math.isfinite(parsed) else None


def read_allowlist(path: Path) -> LoopAllowlist:
    pairs: set[tuple[int, int]] = set()
    signatures: list[LoopSignature] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"current_id", "candidate_id"} - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                current_id = int(row["current_id"])
                candidate_id = int(row["candidate_id"])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_number}: non-numeric loop IDs") from exc
            pairs.add((candidate_id, current_id))
            timestamp_s = parse_optional_float(row.get("timestamp"))
            if timestamp_s is not None:
                signatures.append(
                    LoopSignature(
                        timestamp_s=timestamp_s,
                        fitness_score=parse_optional_float(row.get("fitness_score")),
                        correction_translation_m=parse_optional_float(
                            row.get("correction_translation_m")
                        ),
                        correction_rotation_rad=parse_optional_float(
                            row.get("correction_rotation_rad")
                        ),
                    )
                )
    return LoopAllowlist(pairs=pairs, signatures=signatures)


def read_status_rows(path: Path) -> list[LoopStatusRow]:
    rows: list[LoopStatusRow] = []
    with path.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        if reader.fieldnames is None:
            raise ValueError(f"{path}: missing CSV header")
        missing = {"current_id", "candidate_id", "accepted", "status"} - set(reader.fieldnames)
        if missing:
            raise ValueError(f"{path}: missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            try:
                rows.append(
                    LoopStatusRow(
                        timestamp=row.get("timestamp", ""),
                        current_id=int(row["current_id"]),
                        candidate_id=int(row["candidate_id"]),
                        accepted=truthy(row.get("accepted")) or row.get("status") == "accepted",
                        status=row.get("status", ""),
                        fitness_score=row.get("fitness_score", ""),
                        correction_translation_m=row.get("correction_translation_m", ""),
                        correction_rotation_rad=row.get("correction_rotation_rad", ""),
                    )
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path}:{line_number}: non-numeric loop IDs") from exc
    return rows


def accepted_rows(rows: list[LoopStatusRow]) -> list[LoopStatusRow]:
    return [row for row in rows if row.accepted]


def metric_matches(
    observed_text: str,
    expected: float | None,
    max_delta: float,
) -> bool:
    if max_delta <= 0.0:
        return True
    observed = parse_optional_float(observed_text)
    if observed is None or expected is None:
        return False
    return abs(observed - expected) <= max_delta


def signature_matches(
    row: LoopStatusRow,
    signature: LoopSignature,
    options: SignatureMatchOptions,
) -> bool:
    if options.timestamp_window_s <= 0.0:
        return False
    timestamp_s = parse_optional_float(row.timestamp)
    if timestamp_s is None:
        return False
    if abs(timestamp_s - signature.timestamp_s) > options.timestamp_window_s:
        return False
    return (
        metric_matches(row.fitness_score, signature.fitness_score, options.max_fitness_delta)
        and metric_matches(
            row.correction_translation_m,
            signature.correction_translation_m,
            options.max_translation_delta_m,
        )
        and metric_matches(
            row.correction_rotation_rad,
            signature.correction_rotation_rad,
            options.max_rotation_delta_rad,
        )
    )


def matches_allowlist(
    row: LoopStatusRow,
    allowlist: LoopAllowlist,
    options: SignatureMatchOptions,
) -> tuple[bool, str]:
    if (row.candidate_id, row.current_id) in allowlist.pairs:
        return True, "exact"
    if any(signature_matches(row, signature, options) for signature in allowlist.signatures):
        return True, "signature"
    return False, "outside"


def classify_accepted_rows(
    rows: list[LoopStatusRow],
    allowlist: LoopAllowlist,
    options: SignatureMatchOptions,
) -> dict[str, list[LoopStatusRow]]:
    classified = {"exact": [], "signature": [], "outside": []}
    for row in accepted_rows(rows):
        _, reason = matches_allowlist(row, allowlist, options)
        classified[reason].append(row)
    return classified


def off_allowlist_rows(
    rows: list[LoopStatusRow],
    allowlist: LoopAllowlist,
    options: SignatureMatchOptions | None = None,
) -> list[LoopStatusRow]:
    return classify_accepted_rows(
        rows,
        allowlist,
        options or SignatureMatchOptions(),
    )["outside"]


def format_report(
    allowlist_path: Path,
    status_path: Path,
    allowlist: LoopAllowlist,
    rows: list[LoopStatusRow],
    max_rows: int,
    signature_options: SignatureMatchOptions | None = None,
) -> str:
    options = signature_options or SignatureMatchOptions()
    accepted = accepted_rows(rows)
    classified = classify_accepted_rows(rows, allowlist, options)
    off_rows = classified["outside"]
    exact_accepted = len(classified["exact"])
    signature_accepted = len(classified["signature"])
    allowed_accepted = exact_accepted + signature_accepted
    lines = [
        "# MBES Selected-Loop Allowlist Replay Audit",
        "",
        f"- Allowlist CSV: `{allowlist_path}`",
        f"- Selected replay status CSV: `{status_path}`",
        f"- Allowlisted pairs: {len(allowlist.pairs)}",
        f"- Allowlist signatures: {len(allowlist.signatures)}",
        f"- Signature timestamp window: {options.timestamp_window_s:.3f} s",
        f"- Accepted replay loops: {len(accepted)}",
        f"- Accepted allowlisted loops: {allowed_accepted}",
        f"- Accepted by exact ID: {exact_accepted}",
        f"- Accepted by signature: {signature_accepted}",
        f"- Accepted outside allowlist: {len(off_rows)}",
        "",
    ]
    if not off_rows:
        match_text = (
            "matched the allowlist by exact ID or configured signature"
            if signature_accepted
            else "matched the allowlist exactly"
        )
        lines.extend([
            "## Result",
            "",
            f"PASS: every accepted selected-replay loop {match_text}.",
            "",
        ])
        return "\n".join(lines)

    lines.extend([
        "## Result",
        "",
        "FAIL: selected replay accepted loop pairs that were not present in the allowlist.",
        "",
        "## Off-Allowlist Accepted Loops",
        "",
        "| Rank | Time s | Candidate -> Current | Status | Fitness | Correction m | Rotation rad |",
        "|-----:|--------|----------------------|--------|--------:|-------------:|-------------:|",
    ])
    for rank, row in enumerate(off_rows[:max_rows], start=1):
        lines.append(
            f"| {rank} | {row.timestamp} | {row.candidate_id} -> {row.current_id} | "
            f"{row.status} | {row.fitness_score} | {row.correction_translation_m} | "
            f"{row.correction_rotation_rad} |"
        )
    remaining = len(off_rows) - max_rows
    if remaining > 0:
        lines.extend(["", f"Only the first {max_rows} rows are shown; {remaining} more omitted."])
    lines.append("")
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allowlist", required=True, type=Path,
                        help="CSV used for loop.selection.allowlist_csv")
    parser.add_argument("--status", required=True, type=Path,
                        help="Selected replay mbes_beach_pond_loop_status.csv")
    parser.add_argument("--out", type=Path, help="Optional Markdown output path")
    parser.add_argument("--max-rows", type=int, default=50,
                        help="Maximum off-allowlist rows to show")
    parser.add_argument("--strict", action="store_true",
                        help="Exit non-zero if any accepted loop is outside the allowlist")
    parser.add_argument("--signature-timestamp-window-s", type=float, default=0.0,
                        help="Allow accepted rows matching an allowlist timestamp within this window")
    parser.add_argument("--signature-max-fitness-delta", type=float, default=0.0,
                        help="Optional fitness-score tolerance for signature matching")
    parser.add_argument("--signature-max-translation-delta-m", type=float, default=0.0,
                        help="Optional correction-translation tolerance for signature matching")
    parser.add_argument("--signature-max-rotation-delta-rad", type=float, default=0.0,
                        help="Optional correction-rotation tolerance for signature matching")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    signature_options = SignatureMatchOptions(
        timestamp_window_s=args.signature_timestamp_window_s,
        max_fitness_delta=args.signature_max_fitness_delta,
        max_translation_delta_m=args.signature_max_translation_delta_m,
        max_rotation_delta_rad=args.signature_max_rotation_delta_rad,
    )
    try:
        allowlist = read_allowlist(args.allowlist)
        rows = read_status_rows(args.status)
        report = format_report(
            args.allowlist,
            args.status,
            allowlist,
            rows,
            args.max_rows,
            signature_options,
        )
        off_rows = off_allowlist_rows(rows, allowlist, signature_options)
    except (OSError, ValueError) as exc:
        print(f"failed to audit MBES loop allowlist replay: {exc}", file=sys.stderr)
        return 1

    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
        print(f"wrote MBES selected-loop allowlist audit to {args.out}")
    else:
        print(report)

    if args.strict and off_rows:
        print(
            f"selected replay accepted {len(off_rows)} loop(s) outside the allowlist",
            file=sys.stderr,
        )
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
