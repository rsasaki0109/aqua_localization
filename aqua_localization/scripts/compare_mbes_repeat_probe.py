#!/usr/bin/env python3
"""Compare two MBES repeat-probe artifact directories for determinism.

Each run directory is expected to contain (symlink or copy):

  input_odometry.tum    upstream /aqua_imu_loc/odometry export (TUM format)
  mbes_loop_status.csv  export_mbes_loop_status.py output (optional)
  estimator_status.csv  record_status.py / export_estimator_status.py output
                        (optional)

The report answers, per artifact:

- input odometry: SHA256 hash, sample count, duplicate-stamp count (a nonzero
  duplicate count means more than one publisher wrote into the recording —
  see the leaked-node pitfall in PLAN.md);
- windowed inter-run agreement: raw position difference over the common time
  window with NO alignment — two deterministic runs must agree pointwise, so
  any Umeyama alignment would hide exactly the divergence being measured;
- accepted loop pairs: endpoint keyframe stamp pairs and their overlap;
- estimator status tail: update_count and sonar_feedback_* counters from the
  final row.

Exit code is 0 unless --require-window-agreement is set and the windowed
mean diff exceeds --window-agreement-mean-max-m.

Example:

  python3 aqua_localization/scripts/compare_mbes_repeat_probe.py \\
    /tmp/aqua_mbes_repeat_det/weight_0_run1 \\
    /tmp/aqua_mbes_repeat_det/weight_0_run2 \\
    --out /tmp/aqua_mbes_repeat_det/mbes_repeat_probe_compare.md
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

INPUT_ODOMETRY_NAME = "input_odometry.tum"
LOOP_STATUS_NAME = "mbes_loop_status.csv"
ESTIMATOR_STATUS_NAME = "estimator_status.csv"

STATUS_TAIL_FIELDS = [
    "update_count",
    "sonar_feedback_received",
    "sonar_feedback_applied",
    "sonar_feedback_skipped_stale",
    "sonar_feedback_skipped_nonfinite",
    "sonar_feedback_pending",
]


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as fp:
        for chunk in iter(lambda: fp.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_tum(path: Path) -> np.ndarray:
    """Load a TUM file as an (N, 8) array sorted by timestamp."""
    rows = []
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                raise ValueError(f"{path}: expected 8 fields, got {len(parts)}: {line}")
            rows.append([float(p) for p in parts])
    if not rows:
        raise ValueError(f"{path}: empty trajectory")
    arr = np.asarray(rows, dtype=np.float64)
    order = np.argsort(arr[:, 0], kind="stable")
    return arr[order]


@dataclass
class TumSummary:
    sha256: str
    samples: int
    duplicate_stamps: int
    max_stamp_multiplicity: int
    first_stamp: float
    last_stamp: float


def summarize_tum(path: Path) -> TumSummary:
    arr = load_tum(path)
    stamps, counts = np.unique(arr[:, 0], return_counts=True)
    duplicates = int(np.sum(counts > 1))
    return TumSummary(
        sha256=sha256_of(path),
        samples=int(arr.shape[0]),
        duplicate_stamps=duplicates,
        max_stamp_multiplicity=int(counts.max()) if counts.size else 0,
        first_stamp=float(stamps[0]),
        last_stamp=float(stamps[-1]),
    )


@dataclass
class WindowAgreement:
    """Raw inter-run position difference on the common time window."""

    window_start: float
    window_end: float
    compared_samples: int
    mean_diff_m: float
    max_diff_m: float

    @property
    def window_seconds(self) -> float:
        return self.window_end - self.window_start


def windowed_agreement(a: np.ndarray, b: np.ndarray) -> WindowAgreement | None:
    """Pointwise position diff of run B interpolated at run A's stamps.

    No alignment is applied: deterministic repeats of the same replay share
    the same frame, so any residual is real divergence.
    """
    start = max(a[0, 0], b[0, 0])
    end = min(a[-1, 0], b[-1, 0])
    if end <= start:
        return None
    mask = (a[:, 0] >= start) & (a[:, 0] <= end)
    if not np.any(mask):
        return None
    query = a[mask]
    interp = np.empty((query.shape[0], 3), dtype=np.float64)
    for axis in range(3):
        interp[:, axis] = np.interp(query[:, 0], b[:, 0], b[:, 1 + axis])
    diff = np.linalg.norm(query[:, 1:4] - interp, axis=1)
    return WindowAgreement(
        window_start=float(start),
        window_end=float(end),
        compared_samples=int(diff.shape[0]),
        mean_diff_m=float(np.mean(diff)),
        max_diff_m=float(np.max(diff)),
    )


def accepted_loop_pairs(path: Path) -> set[tuple[str, str]]:
    """Endpoint keyframe stamp pairs of accepted loops from a status CSV."""
    pairs: set[tuple[str, str]] = set()
    with path.open("r", encoding="utf-8", newline="") as fp:
        for row in csv.DictReader(fp):
            accepted = str(row.get("accepted", "")).strip().lower()
            if accepted not in {"1", "true"}:
                continue
            pairs.add((
                str(row.get("current_keyframe_timestamp", "")).strip(),
                str(row.get("candidate_keyframe_timestamp", "")).strip(),
            ))
    return pairs


def estimator_status_tail(path: Path) -> dict[str, str]:
    """Final-row counters from an estimator status CSV."""
    last: dict[str, str] | None = None
    with path.open("r", encoding="utf-8", newline="") as fp:
        for row in csv.DictReader(fp):
            last = row
    if last is None:
        return {}
    return {key: last.get(key, "") for key in STATUS_TAIL_FIELDS}


@dataclass
class RunArtifacts:
    directory: Path
    tum: TumSummary | None = None
    trajectory: np.ndarray | None = None
    loop_pairs: set[tuple[str, str]] | None = None
    status_tail: dict[str, str] | None = None
    missing: list[str] = field(default_factory=list)


def collect_run(directory: Path) -> RunArtifacts:
    run = RunArtifacts(directory=directory)
    tum_path = directory / INPUT_ODOMETRY_NAME
    if tum_path.exists():
        run.trajectory = load_tum(tum_path)
        run.tum = summarize_tum(tum_path)
    else:
        run.missing.append(INPUT_ODOMETRY_NAME)
    loop_path = directory / LOOP_STATUS_NAME
    if loop_path.exists():
        run.loop_pairs = accepted_loop_pairs(loop_path)
    else:
        run.missing.append(LOOP_STATUS_NAME)
    status_path = directory / ESTIMATOR_STATUS_NAME
    if status_path.exists():
        run.status_tail = estimator_status_tail(status_path)
    else:
        run.missing.append(ESTIMATOR_STATUS_NAME)
    return run


def match_marker(matches: bool) -> str:
    return "yes" if matches else "no"


def render_report(
    run1: RunArtifacts,
    run2: RunArtifacts,
    agreement: WindowAgreement | None,
    agreement_mean_max_m: float,
) -> str:
    lines: list[str] = []
    lines.append("# MBES repeat probe comparison")
    lines.append("")
    lines.append(f"- run1: `{run1.directory}`")
    lines.append(f"- run2: `{run2.directory}`")
    for run, label in ((run1, "run1"), (run2, "run2")):
        for name in run.missing:
            lines.append(f"- WARNING: {label} is missing `{name}`")
    lines.append("")

    lines.append("## Input odometry")
    lines.append("")
    lines.append("| Metric | run1 | run2 | Match? |")
    lines.append("|--------|------|------|--------|")
    if run1.tum and run2.tum:
        t1, t2 = run1.tum, run2.tum
        lines.append(
            f"| `{INPUT_ODOMETRY_NAME}` SHA256 | `{t1.sha256[:16]}…` | `{t2.sha256[:16]}…` "
            f"| {match_marker(t1.sha256 == t2.sha256)} |")
        lines.append(
            f"| Samples | {t1.samples} | {t2.samples} "
            f"| {match_marker(t1.samples == t2.samples)} |")
        lines.append(
            f"| Duplicate stamps | {t1.duplicate_stamps} (max x{t1.max_stamp_multiplicity}) "
            f"| {t2.duplicate_stamps} (max x{t2.max_stamp_multiplicity}) "
            f"| {match_marker((t1.duplicate_stamps, t2.duplicate_stamps) == (0, 0))} |")
        lines.append(
            f"| First stamp | {t1.first_stamp:.9f} | {t2.first_stamp:.9f} "
            f"| {match_marker(t1.first_stamp == t2.first_stamp)} |")
        if t1.duplicate_stamps or t2.duplicate_stamps:
            lines.append("")
            lines.append(
                "Duplicate stamps mean multiple publishers wrote into the same "
                "recording — check for leaked nodes from interrupted runs "
                "(`pgrep -af imu_loc_node`) before trusting this probe.")
    else:
        lines.append("| (input odometry missing in at least one run) | - | - | - |")
    lines.append("")

    lines.append("## Windowed inter-run agreement")
    lines.append("")
    lines.append(
        "Raw common-window position difference, no alignment. Deterministic "
        "repeats must agree pointwise.")
    lines.append("")
    if agreement is not None:
        gate = agreement.mean_diff_m <= agreement_mean_max_m
        lines.append("| Metric | Value |")
        lines.append("|--------|-------|")
        lines.append(
            f"| Common window | {agreement.window_seconds:.1f} s "
            f"({agreement.compared_samples} samples) |")
        lines.append(f"| Mean position diff | {agreement.mean_diff_m:.4f} m |")
        lines.append(f"| Max position diff | {agreement.max_diff_m:.4f} m |")
        lines.append(
            f"| Gate (mean <= {agreement_mean_max_m:g} m) | "
            f"{'PASS' if gate else 'FAIL'} |")
    else:
        lines.append("No overlapping time window (or input odometry missing).")
    lines.append("")

    lines.append("## Accepted loop pairs")
    lines.append("")
    if run1.loop_pairs is not None and run2.loop_pairs is not None:
        p1, p2 = run1.loop_pairs, run2.loop_pairs
        common = p1 & p2
        lines.append("| Metric | run1 | run2 | Match? |")
        lines.append("|--------|------|------|--------|")
        lines.append(
            f"| Accepted loop pairs | {len(p1)} | {len(p2)} "
            f"| {match_marker(p1 == p2)} |")
        lines.append(f"| Endpoint-pair overlap | {len(common)} | {len(common)} | - |")
        for label, only in (("run1", p1 - p2), ("run2", p2 - p1)):
            for current, candidate in sorted(only):
                lines.append(f"| only in {label} | `{current}` | `{candidate}` | no |")
    else:
        lines.append(f"`{LOOP_STATUS_NAME}` missing in at least one run.")
    lines.append("")

    lines.append("## Estimator status tail")
    lines.append("")
    if run1.status_tail is not None and run2.status_tail is not None:
        lines.append("| Counter | run1 | run2 | Match? |")
        lines.append("|---------|------|------|--------|")
        for key in STATUS_TAIL_FIELDS:
            v1 = run1.status_tail.get(key, "")
            v2 = run2.status_tail.get(key, "")
            lines.append(f"| `{key}` | {v1} | {v2} | {match_marker(v1 == v2)} |")
    else:
        lines.append(f"`{ESTIMATOR_STATUS_NAME}` missing in at least one run.")
    lines.append("")
    return "\n".join(lines)


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run1", type=Path, help="first probe artifact directory")
    parser.add_argument("run2", type=Path, help="second probe artifact directory")
    parser.add_argument("--out", type=Path, help="write the markdown report here")
    parser.add_argument(
        "--window-agreement-mean-max-m", type=float, default=1.0,
        help="windowed mean position diff gate in meters (default: 1.0)")
    parser.add_argument(
        "--require-window-agreement", action="store_true",
        help="exit nonzero when the windowed mean diff exceeds the gate "
             "(or cannot be computed)")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    run1 = collect_run(args.run1)
    run2 = collect_run(args.run2)

    agreement = None
    if run1.trajectory is not None and run2.trajectory is not None:
        agreement = windowed_agreement(run1.trajectory, run2.trajectory)

    report = render_report(run1, run2, agreement, args.window_agreement_mean_max_m)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(report, encoding="utf-8")
        print(f"wrote {args.out}")
    else:
        print(report)

    if args.require_window_agreement:
        if agreement is None:
            print("window agreement unavailable; failing under --require-window-agreement",
                  file=sys.stderr)
            return 2
        if agreement.mean_diff_m > args.window_agreement_mean_max_m:
            print(
                f"window agreement FAIL: mean {agreement.mean_diff_m:.4f} m > "
                f"{args.window_agreement_mean_max_m:g} m", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
