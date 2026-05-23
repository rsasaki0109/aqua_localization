#!/usr/bin/env python3
"""Summarize the next TODOs for Tank AQUA-SLAM held-out comparison."""

from __future__ import annotations

from dataclasses import dataclass
import sys

import check_tank_aqua_slam_baseline_ready as readiness


@dataclass(frozen=True)
class TodoItem:
    title: str
    done: bool
    detail: str
    command: tuple[str, ...] = ()
    blocked: bool = False


def fence_command(lines: tuple[str, ...]) -> list[str]:
    if not lines:
        return []
    return ["", "```bash", *lines, "```"]


def ingest_command(report: readiness.ReadinessReport) -> tuple[str, ...]:
    args = report.args
    source_arg = readiness.ingest_source_command_arg(report).rstrip()
    return (
        "ros2 run aqua_localization ingest_tank_aqua_slam_baseline.py \\",
        source_arg,
        f"  --reference {args.reference} \\",
        f"  --sequence {args.sequence} \\",
        f"  --config {args.config} \\",
        f"  --out-dir {readiness.baseline_paths(args).estimate_tum.parent}",
    )


def prepare_command(report: readiness.ReadinessReport) -> tuple[str, ...]:
    args = report.args
    return (
        "ros2 run aqua_localization prepare_tank_dvl_heldout_inputs.py \\",
        f"  --sequence {args.sequence} \\",
        f"  --profile {args.profile} \\",
        f"  --ros1-bag /path/to/{readiness.sequence_stem(args.sequence)}.bag \\",
        f"  --reference {args.reference} \\",
        f"  --benchmark-markdown {readiness.repo_benchmark_markdown()} \\",
        f"  --out-dir /tmp/aqua_tank_dvl_{readiness.sequence_slug(args.sequence)}_prepare \\",
        "  --dry-run",
    )


def validation_command(report: readiness.ReadinessReport) -> tuple[str, ...]:
    args = report.args
    return (
        "ros2 run aqua_localization run_tank_dvl_validation_bundle.py \\",
        f"  --profile {args.profile} \\",
        f"  --sequence {args.sequence} \\",
        f"  --bag {args.bag} \\",
        f"  --reference {args.reference} \\",
        f"  --visual {args.visual} \\",
        f"  --benchmark-markdown {readiness.repo_benchmark_markdown()} \\",
        f"  --benchmark-markdown {args.baseline_row} \\",
        "  --max-gap-x 1.0 \\",
        "  --fail-on-gate-failure \\",
        f"  --out-dir /tmp/aqua_tank_dvl_prior_{readiness.sequence_slug(args.sequence)}_validation_bundle",
    )


def build_todos(report: readiness.ReadinessReport) -> list[TodoItem]:
    args = report.args
    todos: list[TodoItem] = [
        TodoItem(
            title="Reference TUM ready",
            done=report.reference.valid,
            detail=(
                f"{report.reference.path} "
                f"({readiness.format_count(report.reference.count)} samples, "
                f"{readiness.format_duration(report.reference.duration_s)} s)"
                if report.reference.valid
                else f"{report.reference.path}: {report.reference.detail}"
            ),
        )
    ]

    if report.source_ready:
        todos.append(
            TodoItem(
                title="AQUA-SLAM trajectory source ready",
                done=True,
                detail=readiness.source_label(report),
            )
        )
    else:
        todos.append(
            TodoItem(
                title="Record AQUA-SLAM odometry CSV",
                done=False,
                detail=f"Expected CSV: {args.csv}",
                command=(f"rostopic echo -p {args.source_topic} > {args.csv}",),
            )
        )

    if report.baseline_row_ready:
        rows = sum(len(source.matching_rows) for source in report.benchmark_sources)
        todos.append(
            TodoItem(
                title="AQUA-SLAM baseline row ready",
                done=True,
                detail=(
                    f"{rows} usable {args.baseline_system} row(s) for {args.sequence} {args.alignment} "
                    f"with >= {readiness.min_baseline_samples(args)} samples and "
                    f">= {readiness.min_baseline_matched_s(args):.2f} matched s"
                ),
            )
        )
    elif any(source.rejected_rows for source in report.benchmark_sources):
        rejected = sum(len(source.rejected_rows) for source in report.benchmark_sources)
        todos.append(
            TodoItem(
                title="AQUA-SLAM baseline row has enough samples",
                done=False,
                detail=(
                    f"{rejected} matching row(s) rejected; require >= "
                    f"{readiness.min_baseline_samples(args)} samples and >= "
                    f"{readiness.min_baseline_matched_s(args):.2f} matched s for gap checks."
                ),
                command=ingest_command(report) if report.ingest_ready else (),
                blocked=not report.ingest_ready,
            )
        )
    elif report.ingest_ready:
        todos.append(
            TodoItem(
                title="Ingest AQUA-SLAM baseline row",
                done=False,
                detail=f"Write {args.baseline_row}",
                command=ingest_command(report),
            )
        )
    else:
        todos.append(
            TodoItem(
                title="Ingest AQUA-SLAM baseline row",
                done=False,
                detail="Blocked until reference TUM and AQUA-SLAM trajectory source are ready.",
                blocked=True,
            )
        )

    todos.extend(
        [
            TodoItem(
                title="DVL prior profile ready",
                done=report.profile_exists,
                detail=f"{args.profile}: {'exists' if report.profile_exists else 'missing'}",
            ),
            TodoItem(
                title="ROS 2 validation bag ready",
                done=report.bag_exists,
                detail=f"{args.bag}: {'exists' if report.bag_exists else 'missing'}",
                command=() if report.bag_exists else prepare_command(report),
            ),
            TodoItem(
                title="Visual frontend TUM ready",
                done=report.visual.valid,
                detail=(
                    f"{report.visual.path} "
                    f"({readiness.format_count(report.visual.count)} samples, "
                    f"{readiness.format_duration(report.visual.duration_s)} s)"
                    if report.visual.valid
                    else f"{report.visual.path}: {report.visual.detail}"
                ),
                command=() if report.visual.valid or not report.bag_exists else prepare_command(report),
            ),
        ]
    )

    todos.append(
        TodoItem(
            title="Run held-out DVL prior validation bundle",
            done=False,
            detail=(
                "All validation inputs are ready."
                if report.validation_ready
                else "Blocked until the missing inputs above are ready."
            ),
            command=validation_command(report) if report.validation_ready else (),
            blocked=not report.validation_ready,
        )
    )
    return todos


def next_action(todos: list[TodoItem]) -> TodoItem | None:
    for item in todos:
        if not item.done and not item.blocked:
            return item
    for item in todos:
        if not item.done:
            return item
    return None


def format_todos(report: readiness.ReadinessReport, todos: list[TodoItem]) -> str:
    args = report.args
    lines = [
        "# Tank AQUA-SLAM Baseline TODOs",
        "",
        f"- Dataset: `{report.dataset}`",
        f"- Sequence: `{report.sequence}`",
        f"- Ready to run validation bundle: **{readiness.pass_fail(report.validation_ready)}**",
        "",
        "## Checklist",
        "",
    ]
    for item in todos:
        checkbox = "x" if item.done else " "
        blocked = " blocked" if item.blocked else ""
        lines.append(f"- [{checkbox}] {item.title}{blocked}: {item.detail}")
        lines.extend(fence_command(item.command))

    action = next_action(todos)
    lines.extend(["", "## Next Action", ""])
    if action is None:
        lines.append("No remaining TODOs.")
    else:
        lines.append(f"{action.title}: {action.detail}")
        lines.extend(fence_command(action.command))

    lines.extend(
        [
            "",
            "## Inputs",
            "",
            f"- Reference: `{args.reference}`",
            f"- AQUA-SLAM CSV: `{args.csv}`",
            f"- AQUA-SLAM TUM: `{args.tum}`",
            f"- Baseline row: `{args.baseline_row}`",
            f"- Profile: `{args.profile}`",
            f"- ROS 2 bag: `{args.bag}`",
            f"- Visual TUM: `{args.visual}`",
        ]
    )
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    args = readiness.parse_args(argv if argv is not None else sys.argv[1:])
    try:
        report = readiness.build_report(args)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    text = format_todos(report, build_todos(report))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0 if report.validation_ready else 1


if __name__ == "__main__":
    sys.exit(main())
