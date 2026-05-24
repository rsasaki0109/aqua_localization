#!/usr/bin/env python3
"""Locate local Tank held-out validation inputs."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import shlex
import sys

import benchmark_gap_report
import check_tank_aqua_slam_baseline_ready as readiness


OFFICIAL_DOWNLOAD_URL = "https://senseroboticslab.github.io/underwater-tank-dataset/download/"
DEFAULT_PROFILE = readiness.DEFAULT_PROFILE
SKIP_DIR_NAMES = {
    ".cache",
    ".git",
    "__pycache__",
    "build",
    "install",
    "log",
    "node_modules",
    "target",
}


@dataclass(frozen=True)
class Candidate:
    role: str
    path: Path
    detail: str


@dataclass(frozen=True)
class LocateReport:
    sequence: str
    roots: tuple[Path, ...]
    candidates: tuple[Candidate, ...]

    def by_role(self, role: str) -> tuple[Candidate, ...]:
        return tuple(candidate for candidate in self.candidates if candidate.role == role)


def sanitize_sequence(value: str) -> str:
    return readiness.sequence_slug(value)


def sequence_aliases(sequence: str) -> tuple[str, ...]:
    slug = sanitize_sequence(sequence)
    aliases = {slug, sequence.lower()}
    if slug == "medium":
        aliases.update({
            "medium",
            "halftankmedium",
            "half_tank_medium",
            "whole_tank_medium",
            "wholetankmedium",
            "structuremedium",
            "structure_medium",
        })
    return tuple(sorted(alias for alias in aliases if alias))


def path_matches_sequence(path: Path, aliases: tuple[str, ...]) -> bool:
    text = str(path).lower().replace("-", "_")
    compact = text.replace("_", "")
    return any(alias in text or alias.replace("_", "") in compact for alias in aliases)


def depth_from(root: Path, path: Path) -> int:
    try:
        rel = path.relative_to(root)
    except ValueError:
        return 0
    return len(rel.parts)


def should_skip_dir(name: str) -> bool:
    return name in SKIP_DIR_NAMES or name.startswith("pytest-") or name.startswith("pytest-of-")


def iter_paths(roots: tuple[Path, ...], max_depth: int):
    seen = set()
    for root in roots:
        if not root.exists():
            continue
        root = root.resolve()
        for dirpath, dirnames, filenames in os.walk(root, topdown=True):
            current = Path(dirpath)
            dirnames[:] = [
                name for name in dirnames
                if not should_skip_dir(name) and depth_from(root, current / name) <= max_depth
            ]
            for name in filenames:
                path = current / name
                key = str(path)
                if key in seen:
                    continue
                seen.add(key)
                yield path


def is_ros2_bag_metadata(path: Path) -> bool:
    if path.name != "metadata.yaml":
        return False
    bag_dir = path.parent
    return any(child.suffix in {".db3", ".mcap"} for child in bag_dir.iterdir())


def benchmark_row_detail(path: Path) -> str:
    try:
        rows = benchmark_gap_report.parse_markdown_benchmark_rows(path.read_text(encoding="utf-8"))
    except Exception:
        return "AQUA-SLAM benchmark row"
    if not rows:
        return "AQUA-SLAM benchmark row; no parseable rows"
    row = rows[0]
    detail = (
        f"AQUA-SLAM benchmark row; samples={row.samples}, "
        f"matched_s={row.matched_seconds}, rmse={row.rmse_m:.4f} m"
    )
    if row.samples is None:
        detail += "; not baseline-ready: missing sample count"
    elif row.samples < readiness.DEFAULT_MIN_BASELINE_SAMPLES:
        detail += "; smoke-sized candidate"
    elif row.matched_seconds is None:
        detail += "; not baseline-ready: missing matched duration"
    elif row.matched_seconds < readiness.DEFAULT_MIN_BASELINE_MATCHED_S:
        detail += "; not baseline-ready: short matched duration"
    return detail


def classify_path(path: Path, aliases: tuple[str, ...]) -> list[Candidate]:
    if not path_matches_sequence(path, aliases):
        return []
    lower = str(path).lower()
    candidates = []
    if path.suffix == ".bag":
        candidates.append(Candidate("ros1_bag", path, "ROS 1 bag"))
    if is_ros2_bag_metadata(path):
        candidates.append(Candidate("ros2_bag", path.parent, "rosbag2 directory"))
    if path.suffix == ".tum":
        role = "reference_tum"
        detail = "TUM trajectory"
        if "visual" in lower:
            role = "visual_tum"
            detail = "visual frontend TUM"
        elif "aqua_slam" in lower or "orb_odom" in lower:
            role = "aqua_slam_tum"
            detail = "AQUA-SLAM TUM"
        elif "gt" in lower or "ref" in lower or "reference" in lower:
            role = "reference_tum"
            detail = "reference TUM"
        candidates.append(Candidate(role, path, detail))
    if path.suffix == ".csv" and ("aqua_slam" in lower or "orb_odom" in lower):
        candidates.append(Candidate("aqua_slam_csv", path, "AQUA-SLAM CSV"))
    if path.name.endswith("_aqua_slam_benchmark_row.md"):
        candidates.append(Candidate("baseline_row", path, benchmark_row_detail(path)))
    return candidates


def locate_inputs(sequence: str, roots: tuple[Path, ...], max_depth: int) -> LocateReport:
    aliases = sequence_aliases(sequence)
    candidates = []
    for path in iter_paths(roots, max_depth):
        candidates.extend(classify_path(path, aliases))
    candidates.sort(key=lambda item: (item.role, str(item.path)))
    return LocateReport(sequence=sequence, roots=roots, candidates=tuple(candidates))


def first_path(report: LocateReport, role: str) -> Path | None:
    candidates = report.by_role(role)
    return candidates[0].path if candidates else None


def shell_join(command: tuple[str, ...]) -> str:
    return shlex.join([str(part) for part in command])


def command_block(command: tuple[str, ...]) -> list[str]:
    return ["```bash", shell_join(command), "```"]


def default_roots(repo_root: Path) -> tuple[Path, ...]:
    return (
        Path("/tmp"),
        repo_root / "datasets",
        repo_root.parent,
        Path("/media/autoware/aa"),
        Path.home() / "Downloads",
    )


def format_role_section(report: LocateReport, role: str, label: str) -> list[str]:
    lines = [f"### {label}", ""]
    candidates = report.by_role(role)
    if not candidates:
        lines.append("- none")
    else:
        for candidate in candidates:
            lines.append(f"- `{candidate.path}` ({candidate.detail})")
    lines.append("")
    return lines


def format_default_link_commands(report: LocateReport) -> list[str]:
    sequence_slug = sanitize_sequence(report.sequence)
    reference = first_path(report, "reference_tum")
    ros2_bag = first_path(report, "ros2_bag")
    visual = first_path(report, "visual_tum")
    lines = ["## Default Path Commands", ""]
    commands = []
    if reference is not None:
        commands.append(("ln", "-sfn", str(reference), f"/tmp/tank_{sequence_slug}_gt.tum"))
    if ros2_bag is not None:
        commands.append(("ln", "-sfn", str(ros2_bag), f"/tmp/tank_{sequence_slug}_ros2_visual"))
    if visual is not None:
        commands.append(("ln", "-sfn", str(visual), f"/tmp/tank_{sequence_slug}_visual_frontend.tum"))
    if not commands:
        lines.append("- no local candidates found for default links")
    else:
        lines.append("Run these only after checking the candidate paths:")
        lines.append("")
        lines.append("```bash")
        lines.extend(shell_join(command) for command in commands)
        lines.append("```")
    lines.append("")
    return lines


def format_next_commands(args, report: LocateReport) -> list[str]:
    sequence_slug = sanitize_sequence(report.sequence)
    lines = ["## Next Commands", ""]
    if first_path(report, "reference_tum") is None or first_path(report, "ros2_bag") is None:
        lines.extend([
            "Held-out validation is blocked until the Medium reference and ROS 2 bag are available.",
            "",
            f"Official download page: {OFFICIAL_DOWNLOAD_URL}",
            "",
        ])
    lines.extend(command_block((
        "ros2",
        "run",
        "aqua_localization",
        "prepare_tank_dvl_heldout_inputs.py",
        "--sequence",
        report.sequence,
        "--profile",
        str(args.profile),
        "--ros2-bag",
        f"/tmp/tank_{sequence_slug}_ros2_visual",
        "--reference",
        f"/tmp/tank_{sequence_slug}_gt.tum",
        "--benchmark-markdown",
        str(args.benchmark_markdown),
        "--out-dir",
        f"/tmp/aqua_tank_dvl_{sequence_slug}_prepare",
        "--dry-run",
    )))
    lines.append("")
    lines.extend(command_block((
        "ros2",
        "run",
        "aqua_localization",
        "locate_tank_heldout_inputs.py",
        "--sequence",
        report.sequence,
        "--profile",
        str(args.profile),
        "--out",
        str(args.out or Path(f"/tmp/aqua_tank_{sequence_slug}_heldout_locator.md")),
    )))
    lines.append("")
    return lines


def format_report(args, report: LocateReport) -> str:
    lines = [
        "# Tank Held-Out Input Locator",
        "",
        f"- Sequence: `{report.sequence}`",
        f"- Candidates found: `{len(report.candidates)}`",
        f"- Official download page: {OFFICIAL_DOWNLOAD_URL}",
        "",
        "## Search Roots",
        "",
    ]
    for root in report.roots:
        lines.append(f"- `{root}`")
    lines.extend(["", "## Candidates", ""])
    lines.extend(format_role_section(report, "reference_tum", "Reference TUM"))
    lines.extend(format_role_section(report, "ros2_bag", "ROS 2 Bag"))
    lines.extend(format_role_section(report, "ros1_bag", "ROS 1 Bag"))
    lines.extend(format_role_section(report, "visual_tum", "Visual TUM"))
    lines.extend(format_role_section(report, "aqua_slam_csv", "AQUA-SLAM CSV"))
    lines.extend(format_role_section(report, "aqua_slam_tum", "AQUA-SLAM TUM"))
    lines.extend(format_role_section(report, "baseline_row", "AQUA-SLAM Baseline Row"))
    lines.extend(format_default_link_commands(report))
    lines.extend(format_next_commands(args, report))
    return "\n".join(lines)


def parse_args(argv):
    repo_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Find local Tank held-out bag/reference/benchmark inputs."
    )
    parser.add_argument("--sequence", default="Medium")
    parser.add_argument("--root", action="append", type=Path, default=[])
    parser.add_argument("--max-depth", type=int, default=7)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument(
        "--benchmark-markdown",
        type=Path,
        default=repo_root / "docs" / "benchmarks" / "tank_aqua_slam.md",
    )
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    args.roots = tuple(args.root) if args.root else default_roots(repo_root)
    return args


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.max_depth < 0:
        print("error: --max-depth must be non-negative", file=sys.stderr)
        return 2
    report = locate_inputs(args.sequence, args.roots, args.max_depth)
    text = format_report(args, report)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
