#!/usr/bin/env python3
"""Fail-fast verifier for the Tank Medium AQUA-SLAM held-out comparison."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import shlex
import sys

import check_tank_aqua_slam_baseline_ready as readiness
import locate_tank_heldout_inputs as locator
import run_tank_dvl_validation_bundle as validation_bundle


DEFAULT_OUT_DIR = Path("/tmp/aqua_slam_medium_heldout_verify")


@dataclass(frozen=True)
class NextAction:
    title: str
    detail: str
    command: tuple[str, ...]


@dataclass(frozen=True)
class VerifyReport:
    readiness_report: readiness.ReadinessReport
    locate_report: locator.LocateReport
    next_action: NextAction
    applied_links: tuple[tuple[Path, Path], ...] = ()

    @property
    def ready(self) -> bool:
        return self.readiness_report.validation_ready


def shell_join(command: tuple[str, ...]) -> str:
    return shlex.join([str(part) for part in command])


def command_block(command: tuple[str, ...]) -> list[str]:
    if not command:
        return []
    return ["```bash", shell_join(command), "```"]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def locator_roots(args) -> tuple[Path, ...]:
    return tuple(args.locator_root) if args.locator_root else locator.default_roots(repo_root())


def build_readiness_args(args) -> argparse.Namespace:
    return argparse.Namespace(
        sequence=args.sequence,
        dataset=args.dataset,
        alignment=args.alignment,
        baseline_system=args.baseline_system,
        target_system=args.target_system,
        reference=args.reference,
        csv=args.csv,
        tum=args.tum,
        baseline_dir=args.baseline_dir,
        baseline_row=args.baseline_row,
        benchmark_markdown=list(args.benchmark_markdown),
        profile=args.profile,
        bag=args.bag,
        visual=args.visual,
        time_unit=args.time_unit,
        source_topic=args.source_topic,
        config=args.config,
        min_baseline_samples=args.min_baseline_samples,
        min_baseline_matched_s=args.min_baseline_matched_s,
    )


def locate_candidates(args) -> locator.LocateReport:
    return locator.locate_inputs(args.sequence, locator_roots(args), args.locator_max_depth)


def locator_command(args) -> tuple[str, ...]:
    return (
        "ros2",
        "run",
        "aqua_localization",
        "locate_tank_heldout_inputs.py",
        "--sequence",
        args.sequence,
        "--profile",
        str(args.profile),
        "--out",
        str(args.out_dir / "heldout_locator.md"),
    )


def link_command(candidate: Path, target: Path) -> tuple[str, ...]:
    return ("ln", "-sfn", str(candidate), str(target))


def extract_archive_command(archive: Path, out_dir: Path) -> tuple[str, ...]:
    suffixes = [suffix.lower() for suffix in archive.suffixes]
    if suffixes and suffixes[-1] == ".zip":
        return (
            "bash",
            "-lc",
            f"mkdir -p {shlex.quote(str(out_dir))} && "
            f"python3 -m zipfile -e {shlex.quote(str(archive))} {shlex.quote(str(out_dir))}",
        )
    return (
        "bash",
        "-lc",
        f"mkdir -p {shlex.quote(str(out_dir))} && "
        f"tar -xf {shlex.quote(str(archive))} -C {shlex.quote(str(out_dir))}",
    )


def safe_symlink(candidate: Path, target: Path) -> bool:
    candidate = candidate.resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_symlink():
        try:
            if target.resolve() == candidate:
                return False
        except FileNotFoundError:
            pass
        target.unlink()
    elif target.exists():
        raise ValueError(f"refusing to replace existing non-symlink path: {target}")
    target.symlink_to(candidate, target_is_directory=candidate.is_dir())
    return True


def ingest_command(report: readiness.ReadinessReport) -> tuple[str, ...]:
    args = report.args
    source_flag = "--tum" if report.tum.valid and not report.csv.valid else "--csv"
    source_path = args.tum if source_flag == "--tum" else args.csv
    return (
        "ros2",
        "run",
        "aqua_localization",
        "ingest_tank_aqua_slam_baseline.py",
        source_flag,
        str(source_path),
        "--reference",
        str(args.reference),
        "--sequence",
        args.sequence,
        "--config",
        args.config,
        "--out-dir",
        str(readiness.baseline_paths(args).estimate_tum.parent),
    )


def record_aqua_slam_command(report: readiness.ReadinessReport) -> tuple[str, ...]:
    args = report.args
    return (
        "bash",
        "-lc",
        f"rostopic echo -p {shlex.quote(args.source_topic)} > {shlex.quote(str(args.csv))}",
    )


def convert_ros1_bag_command(report: readiness.ReadinessReport, ros1_bag: Path) -> tuple[str, ...]:
    args = report.args
    return (
        "ros2",
        "run",
        "aqua_localization",
        "convert_tank_dataset_bag.py",
        "--src",
        str(ros1_bag),
        "--dst",
        str(args.bag),
        "--include-cameras",
    )


def export_reference_command(report: readiness.ReadinessReport, args) -> tuple[str, ...]:
    ready_args = report.args
    return (
        "ros2",
        "run",
        "aqua_localization",
        "export_rosbag_odometry_tum.py",
        "--bag",
        str(ready_args.bag),
        "--topic",
        args.reference_topic,
        "--out",
        str(ready_args.reference),
    )


def promote_profile_command(args) -> tuple[str, ...]:
    return (
        "ros2",
        "run",
        "aqua_localization",
        "promote_tank_dvl_sweep_profile.py",
        "--base-profile",
        "/tmp/aqua_tank_dvl_prior_profile_short_to_medium.yaml",
        "--sweep-csv",
        "/tmp/aqua_tank_dvl_prior_confidence_sweep_short_diag/tank_dvl_prior_gate_sweep.csv",
        "--rank",
        "1",
        "--out",
        str(args.profile),
        "--name",
        "tank_short_to_medium_confidence_sweep_rank1",
        "--note",
        "same-sequence confidence gate sweep rank 1; validate on held-out Medium before benchmark use",
    )


def prepare_visual_command(report: readiness.ReadinessReport, args) -> tuple[str, ...]:
    ready_args = report.args
    return (
        "ros2",
        "run",
        "aqua_localization",
        "prepare_tank_dvl_heldout_inputs.py",
        "--sequence",
        ready_args.sequence,
        "--profile",
        str(ready_args.profile),
        "--ros2-bag",
        str(ready_args.bag),
        "--reference",
        str(ready_args.reference),
        "--benchmark-markdown",
        str(ready_args.baseline_row),
        "--out-dir",
        str(args.out_dir / "prepare"),
        "--dry-run",
    )


def validation_command(report: readiness.ReadinessReport, args) -> tuple[str, ...]:
    ready_args = report.args
    command = [
        "ros2",
        "run",
        "aqua_localization",
        "run_tank_dvl_validation_bundle.py",
        "--profile",
        str(ready_args.profile),
        "--sequence",
        ready_args.sequence,
        "--bag",
        str(ready_args.bag),
        "--reference",
        str(ready_args.reference),
        "--visual",
        str(ready_args.visual),
        "--benchmark-markdown",
        str(readiness.repo_benchmark_markdown()),
        "--benchmark-markdown",
        str(ready_args.baseline_row),
        "--max-gap-x",
        str(args.max_gap_x),
        "--min-target-samples",
        str(args.min_target_samples),
        "--min-target-matched-s",
        str(args.min_target_matched_s),
        "--fail-on-gate-failure",
        "--out-dir",
        str(args.validation_out_dir),
    ]
    return tuple(command)


def first_candidate_path(locate_report: locator.LocateReport, role: str) -> Path | None:
    path = locator.first_path(locate_report, role)
    return path.resolve() if path is not None else None


def first_usable_baseline_candidate(
    locate_report: locator.LocateReport,
    ready_args: argparse.Namespace,
) -> Path | None:
    for candidate in locate_report.by_role("baseline_row"):
        summary = readiness.summarize_markdown_source(candidate.path, ready_args)
        if summary.matching_rows:
            return candidate.path.resolve()
    return None


def next_action(report: readiness.ReadinessReport, locate_report: locator.LocateReport, args) -> NextAction:
    ready_args = report.args
    if not report.reference.valid:
        candidate = first_candidate_path(locate_report, "reference_tum")
        if candidate is not None and candidate != ready_args.reference.resolve():
            return NextAction(
                "Link Medium reference TUM",
                f"Use the located reference candidate for `{ready_args.reference}`.",
                link_command(candidate, ready_args.reference),
            )
        if report.bag_exists:
            return NextAction(
                "Export Medium reference TUM",
                f"Extract `{args.reference_topic}` from `{ready_args.bag}` into `{ready_args.reference}`.",
                export_reference_command(report, args),
            )
        ros2_candidate = first_candidate_path(locate_report, "ros2_bag")
        if ros2_candidate is not None and ros2_candidate != ready_args.bag.resolve():
            return NextAction(
                "Link Medium ROS 2 bag",
                f"Use the located rosbag2 directory for `{ready_args.bag}`.",
                link_command(ros2_candidate, ready_args.bag),
            )
        ros1_candidate = first_candidate_path(locate_report, "ros1_bag")
        if ros1_candidate is not None:
            return NextAction(
                "Convert Medium ROS 1 bag to ROS 2",
                f"Convert the located ROS 1 bag into `{ready_args.bag}` with camera topics kept.",
                convert_ros1_bag_command(report, ros1_candidate),
            )
        archive_candidate = first_candidate_path(locate_report, "archive")
        if archive_candidate is not None:
            return NextAction(
                "Extract Medium download archive",
                f"Extract the located archive into `{args.archive_out_dir}`, then rescan.",
                extract_archive_command(archive_candidate, args.archive_out_dir),
            )
        return NextAction(
            "Find Medium reference TUM",
            f"Copy/download the Medium reference TUM, then rescan. {locator.OFFICIAL_DOWNLOAD_URL}",
            locator_command(args),
        )

    if not report.bag_exists:
        candidate = first_candidate_path(locate_report, "ros2_bag")
        if candidate is not None and candidate != ready_args.bag.resolve():
            return NextAction(
                "Link Medium ROS 2 bag",
                f"Use the located rosbag2 directory for `{ready_args.bag}`.",
                link_command(candidate, ready_args.bag),
            )
        ros1_candidate = first_candidate_path(locate_report, "ros1_bag")
        if ros1_candidate is not None:
            return NextAction(
                "Convert Medium ROS 1 bag to ROS 2",
                f"Convert the located ROS 1 bag into `{ready_args.bag}` with camera topics kept.",
                convert_ros1_bag_command(report, ros1_candidate),
            )
        archive_candidate = first_candidate_path(locate_report, "archive")
        if archive_candidate is not None:
            return NextAction(
                "Extract Medium download archive",
                f"Extract the located archive into `{args.archive_out_dir}`, then rescan.",
                extract_archive_command(archive_candidate, args.archive_out_dir),
            )
        return NextAction(
            "Find Medium ROS 2 bag",
            f"Copy/download or convert the Medium bag, then rescan. {locator.OFFICIAL_DOWNLOAD_URL}",
            locator_command(args),
        )

    if not report.profile_exists:
        return NextAction(
            "Promote rank-1 DVL profile",
            f"Create the rank-1 profile expected at `{ready_args.profile}`.",
            promote_profile_command(args),
        )

    if not report.source_ready:
        csv_candidate = first_candidate_path(locate_report, "aqua_slam_csv")
        if csv_candidate is not None and csv_candidate != ready_args.csv.resolve():
            return NextAction(
                "Link Medium AQUA-SLAM CSV",
                f"Use the located AQUA-SLAM CSV for `{ready_args.csv}`.",
                link_command(csv_candidate, ready_args.csv),
            )
        tum_candidate = first_candidate_path(locate_report, "aqua_slam_tum")
        if tum_candidate is not None and tum_candidate != ready_args.tum.resolve():
            return NextAction(
                "Link Medium AQUA-SLAM TUM",
                f"Use the located AQUA-SLAM TUM for `{ready_args.tum}`.",
                link_command(tum_candidate, ready_args.tum),
            )
        return NextAction(
            "Record AQUA-SLAM odometry CSV",
            "Run this inside the AQUA-SLAM ROS 1 environment while the Medium bag is playing.",
            record_aqua_slam_command(report),
        )

    if not report.baseline_row_ready:
        candidate = first_usable_baseline_candidate(locate_report, ready_args)
        if candidate is not None and candidate != ready_args.baseline_row.resolve():
            return NextAction(
                "Link Medium AQUA-SLAM baseline row",
                f"Use the located usable benchmark row for `{ready_args.baseline_row}`.",
                link_command(candidate, ready_args.baseline_row),
            )
        return NextAction(
            "Ingest AQUA-SLAM baseline row",
            "Generate a usable Medium AQUA-SLAM row that passes sample and matched-duration gates.",
            ingest_command(report),
        )

    if not report.visual.valid:
        candidate = first_candidate_path(locate_report, "visual_tum")
        if candidate is not None and candidate != ready_args.visual.resolve():
            return NextAction(
                "Link Medium visual TUM",
                f"Use the located visual frontend trajectory for `{ready_args.visual}`.",
                link_command(candidate, ready_args.visual),
            )
        return NextAction(
            "Generate Medium visual TUM",
            "Prepare or regenerate the visual frontend trajectory before running the held-out bundle.",
            prepare_visual_command(report, args),
        )

    return NextAction(
        "Run Medium held-out validation bundle",
        "All required inputs are ready.",
        validation_command(report, args),
    )


def build_verify_report(args) -> VerifyReport:
    readiness_args = build_readiness_args(args)
    readiness_report = readiness.build_report(readiness_args)
    locate_report = locate_candidates(args)
    return VerifyReport(
        readiness_report=readiness_report,
        locate_report=locate_report,
        next_action=next_action(readiness_report, locate_report, args),
    )


def apply_located_links(args) -> VerifyReport:
    applied: list[tuple[Path, Path]] = []
    for _ in range(args.max_link_passes):
        verify = build_verify_report(args)
        command = verify.next_action.command
        if len(command) != 4 or command[:2] != ("ln", "-sfn"):
            return VerifyReport(
                verify.readiness_report,
                verify.locate_report,
                verify.next_action,
                tuple(applied),
            )
        candidate = Path(command[2])
        target = Path(command[3])
        if safe_symlink(candidate, target):
            applied.append((candidate.resolve(), target))
            continue
        return VerifyReport(
            verify.readiness_report,
            verify.locate_report,
            verify.next_action,
            tuple(applied),
        )
    verify = build_verify_report(args)
    return VerifyReport(
        verify.readiness_report,
        verify.locate_report,
        verify.next_action,
        tuple(applied),
    )


def candidate_count(locate_report: locator.LocateReport, role: str) -> int:
    return len(locate_report.by_role(role))


def rejected_baseline_count(report: readiness.ReadinessReport) -> int:
    return sum(len(source.rejected_rows) for source in report.benchmark_sources)


def format_report(verify: VerifyReport, args) -> str:
    report = verify.readiness_report
    ready_args = report.args
    locate_report = verify.locate_report
    status = "PASS" if verify.ready else "BLOCKED"
    usable_rows = sum(len(source.matching_rows) for source in report.benchmark_sources)
    lines = [
        "# Tank Medium Held-Out Verifier",
        "",
        f"- Status: `{status}`",
        f"- Sequence: `{report.sequence}`",
        f"- Validation command ready: `{readiness.pass_fail(verify.ready)}`",
        f"- Official download page: {locator.OFFICIAL_DOWNLOAD_URL}",
        "",
        "## Required Inputs",
        "",
        "| Input | Path | Status | Detail |",
        "|-------|------|--------|--------|",
        f"| Reference TUM | `{ready_args.reference}` | {readiness.pass_fail(report.reference.valid)} | {report.reference.detail} |",
        f"| ROS 2 bag | `{ready_args.bag}` | {readiness.pass_fail(report.bag_exists)} | {'exists' if report.bag_exists else 'missing'} |",
        f"| Rank-1 profile | `{ready_args.profile}` | {readiness.pass_fail(report.profile_exists)} | {'exists' if report.profile_exists else 'missing'} |",
        f"| AQUA-SLAM source | `{ready_args.csv}` / `{ready_args.tum}` | {readiness.pass_fail(report.source_ready)} | {readiness.source_label(report)} |",
        (
            f"| AQUA-SLAM baseline row | `{ready_args.baseline_row}` | "
            f"{readiness.pass_fail(report.baseline_row_ready)} | usable={usable_rows}, "
            f"rejected={rejected_baseline_count(report)}, min samples={readiness.min_baseline_samples(ready_args)}, "
            f"min matched s={readiness.min_baseline_matched_s(ready_args):.2f} |"
        ),
        f"| Visual TUM | `{ready_args.visual}` | {readiness.pass_fail(report.visual.valid)} | {report.visual.detail} |",
        "",
        "## Located Candidates",
        "",
        "| Role | Count | First candidate |",
        "|------|------:|-----------------|",
    ]
    for role, label in (
        ("reference_tum", "Reference TUM"),
        ("ros2_bag", "ROS 2 bag"),
        ("ros1_bag", "ROS 1 bag"),
        ("visual_tum", "Visual TUM"),
        ("aqua_slam_csv", "AQUA-SLAM CSV"),
        ("aqua_slam_tum", "AQUA-SLAM TUM"),
        ("baseline_row", "AQUA-SLAM baseline row"),
        ("archive", "Download archive"),
    ):
        first = locator.first_path(locate_report, role)
        lines.append(
            f"| {label} | {candidate_count(locate_report, role)} | "
            f"{'`' + str(first) + '`' if first is not None else 'none'} |"
        )

    lines.extend([
        "",
        "## Next Action",
        "",
        f"{verify.next_action.title}: {verify.next_action.detail}",
        "",
        *command_block(verify.next_action.command),
        "",
    ])
    if verify.applied_links:
        lines.extend(["## Applied Links", "", "| Source | Target |", "|--------|--------|"])
        for source, target in verify.applied_links:
            lines.append(f"| `{source}` | `{target}` |")
        lines.append("")
    if verify.ready:
        lines.extend([
            "## Validation Command",
            "",
            *command_block(validation_command(report, args)),
            "",
        ])
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Verify the Tank Medium held-out comparison inputs and print the next command."
    )
    parser.add_argument("--sequence", default="Medium")
    parser.add_argument("--dataset", default=readiness.DEFAULT_DATASET)
    parser.add_argument("--alignment", default=readiness.DEFAULT_ALIGNMENT)
    parser.add_argument("--baseline-system", default=readiness.DEFAULT_BASELINE_SYSTEM)
    parser.add_argument("--target-system", default=readiness.DEFAULT_TARGET_SYSTEM)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--tum", type=Path)
    parser.add_argument("--baseline-dir", type=Path)
    parser.add_argument("--baseline-row", type=Path)
    parser.add_argument("--benchmark-markdown", action="append", type=Path, default=[])
    parser.add_argument("--profile", type=Path, default=readiness.DEFAULT_PROFILE)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--visual", type=Path)
    parser.add_argument("--time-unit", choices=("auto", "seconds", "nanoseconds"), default="auto")
    parser.add_argument("--source-topic", default="/AQUA_SLAM/orb_odom")
    parser.add_argument("--reference-topic", default="/apriltag_slam/GT")
    parser.add_argument("--config", default="underwater_orbslam3_blue_gx5_medium.yaml")
    parser.add_argument("--min-baseline-samples", type=int, default=readiness.DEFAULT_MIN_BASELINE_SAMPLES)
    parser.add_argument("--min-baseline-matched-s", type=float, default=readiness.DEFAULT_MIN_BASELINE_MATCHED_S)
    parser.add_argument("--min-target-samples", type=int, default=validation_bundle.DEFAULT_MIN_TARGET_SAMPLES)
    parser.add_argument("--min-target-matched-s", type=float, default=validation_bundle.DEFAULT_MIN_TARGET_MATCHED_S)
    parser.add_argument("--max-gap-x", type=float, default=1.0)
    parser.add_argument("--validation-out-dir", type=Path)
    parser.add_argument("--locator-root", action="append", type=Path, default=[])
    parser.add_argument("--locator-max-depth", type=int, default=7)
    parser.add_argument("--archive-out-dir", type=Path, default=Path("/tmp/tank_medium_download"))
    parser.add_argument(
        "--apply-located-links",
        action="store_true",
        help="Safely symlink located candidates into the verifier's default paths.",
    )
    parser.add_argument("--max-link-passes", type=int, default=8)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    if args.validation_out_dir is None:
        args.validation_out_dir = Path(
            f"/tmp/aqua_tank_dvl_prior_{readiness.sequence_slug(args.sequence)}_validation_bundle"
        )
    return args


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        verify = (
            apply_located_links(args)
            if args.apply_located_links
            else build_verify_report(args)
        )
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    text = format_report(verify, args)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0 if verify.ready else 1


if __name__ == "__main__":
    sys.exit(main())
