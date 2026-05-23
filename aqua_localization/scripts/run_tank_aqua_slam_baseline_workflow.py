#!/usr/bin/env python3
"""Run the Tank AQUA-SLAM baseline ingest and held-out validation workflow."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import check_tank_aqua_slam_baseline_ready as readiness
import ingest_tank_aqua_slam_baseline as baseline_ingest
import locate_tank_heldout_inputs as input_locator
import run_tank_dvl_validation_bundle as validation_bundle
import summarize_tank_aqua_slam_baseline_todos as todo_summary


STATUS_PASS = "PASS"
STATUS_BLOCKED = "BLOCKED"
STATUS_FAIL = "FAIL"
STATUS_SKIPPED = "SKIPPED"


@dataclass(frozen=True)
class WorkflowPaths:
    out_dir: Path
    readiness: Path
    todos: Path
    summary: Path
    locator: Path
    validation_out_dir: Path


@dataclass(frozen=True)
class StepResult:
    name: str
    status: str
    detail: str


@dataclass(frozen=True)
class WorkflowResult:
    status: str
    steps: tuple[StepResult, ...]
    paths: WorkflowPaths
    readiness_report: readiness.ReadinessReport
    todos: tuple[todo_summary.TodoItem, ...]
    validation_summary: str = ""

    @property
    def ok(self) -> bool:
        return self.status == STATUS_PASS


def default_out_dir(args) -> Path:
    return readiness.baseline_paths(args).estimate_tum.parent


def workflow_paths(args) -> WorkflowPaths:
    out_dir = args.out_dir or default_out_dir(args)
    return WorkflowPaths(
        out_dir=out_dir,
        readiness=args.readiness_out or out_dir / "readiness.md",
        todos=args.todos_out or out_dir / "todos.md",
        summary=args.summary_out or out_dir / "workflow_summary.md",
        locator=args.locator_out or out_dir / "heldout_locator.md",
        validation_out_dir=args.validation_out_dir
        or Path(f"/tmp/aqua_tank_dvl_prior_{readiness.sequence_slug(args.sequence)}_validation_bundle"),
    )


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def source_argv(report: readiness.ReadinessReport) -> list[str]:
    if report.tum.valid and not report.csv.valid:
        return ["--tum", str(report.args.tum)]
    return ["--csv", str(report.args.csv)]


def make_ingest_argv(args, report: readiness.ReadinessReport) -> list[str]:
    paths = readiness.baseline_paths(args)
    argv = [
        *source_argv(report),
        "--reference",
        str(args.reference),
        "--sequence",
        args.sequence,
        "--dataset",
        args.dataset,
        "--system",
        args.baseline_system,
        "--out-dir",
        str(paths.estimate_tum.parent),
        "--tum-out",
        str(args.tum),
        "--row-out",
        str(args.baseline_row),
        "--manifest-out",
        str(paths.manifest),
        "--runtime",
        args.baseline_runtime,
        "--source",
        args.source_topic,
        "--config",
        args.config,
    ]
    if args.baseline_commit:
        argv.extend(["--commit", args.baseline_commit])
    if args.baseline_note:
        argv.extend(["--note", args.baseline_note])
    if args.scale:
        argv.append("--scale")
    if args.no_align:
        argv.append("--no-align")
    return argv


def run_ingest_if_ready(args, report: readiness.ReadinessReport) -> StepResult:
    if report.baseline_row_ready:
        return StepResult("Baseline ingest", STATUS_SKIPPED, "matching baseline row already exists")
    if not report.ingest_ready:
        return StepResult("Baseline ingest", STATUS_BLOCKED, "reference and AQUA-SLAM trajectory source are not ready")
    if args.skip_ingest:
        return StepResult("Baseline ingest", STATUS_SKIPPED, "--skip-ingest was requested")
    if args.dry_run:
        return StepResult("Baseline ingest", STATUS_SKIPPED, "--dry-run: baseline row was not generated")

    try:
        ingest_args = baseline_ingest.parse_args(make_ingest_argv(args, report))
        ingest_paths = baseline_ingest.output_paths(ingest_args)
        baseline_ingest.validate_args(ingest_args)
        ingest_args.out_dir.mkdir(parents=True, exist_ok=True)
        estimate, used_csv = baseline_ingest.prepare_estimate(ingest_args, ingest_paths)
        baseline_ingest.write_row_and_manifest(ingest_args, estimate, ingest_paths, used_csv)
    except ValueError as exc:
        return StepResult("Baseline ingest", STATUS_FAIL, str(exc))
    return StepResult("Baseline ingest", STATUS_PASS, f"wrote {ingest_paths.benchmark_row}")


def existing_validation_markdown(args) -> list[Path]:
    paths = []
    for path in [*args.benchmark_markdown, args.baseline_row]:
        if path.exists() and path not in paths:
            paths.append(path)
    return paths


def make_bundle_argv(args, paths: WorkflowPaths) -> list[str]:
    argv = [
        "--profile",
        str(args.profile),
        "--sequence",
        args.sequence,
        "--bag",
        str(args.bag),
        "--reference",
        str(args.reference),
        "--visual",
        str(args.visual),
        "--out-dir",
        str(paths.validation_out_dir),
        "--dataset",
        args.dataset,
        "--target-system",
        args.target_system,
        "--baseline-system",
        args.baseline_system,
    ]
    for path in existing_validation_markdown(args):
        argv.extend(["--benchmark-markdown", str(path)])
    if args.skip_residual_analysis:
        argv.append("--skip-residual-analysis")
    if args.allow_same_sequence:
        argv.append("--allow-same-sequence")
    if args.allow_profile_sequence_mismatch:
        argv.append("--allow-profile-sequence-mismatch")
    if args.max_corrected_rmse_m is not None:
        argv.extend(["--max-corrected-rmse-m", str(args.max_corrected_rmse_m)])
    if args.min_improvement_percent is not None:
        argv.extend(["--min-improvement-percent", str(args.min_improvement_percent)])
    if args.fail_on_gate_failure:
        argv.append("--fail-on-gate-failure")
    if args.max_gap_x is not None:
        argv.extend(["--max-gap-x", str(args.max_gap_x)])
    if args.max_improvement_to_tie_percent is not None:
        argv.extend(["--max-improvement-to-tie-percent", str(args.max_improvement_to_tie_percent)])
    if args.residual_top_k is not None:
        argv.extend(["--residual-top-k", str(args.residual_top_k)])
    if args.validation_note:
        argv.extend(["--note", args.validation_note])
    return argv


def run_validation_if_ready(args, paths: WorkflowPaths, report: readiness.ReadinessReport) -> tuple[StepResult, str]:
    if not report.validation_ready:
        return StepResult("Validation bundle", STATUS_BLOCKED, "held-out validation inputs are not ready"), ""
    if args.skip_validation:
        return StepResult("Validation bundle", STATUS_SKIPPED, "--skip-validation was requested"), ""
    if args.dry_run:
        return StepResult("Validation bundle", STATUS_SKIPPED, "--dry-run: validation bundle was not executed"), ""

    try:
        bundle_args = validation_bundle.parse_args(make_bundle_argv(args, paths))
        validation_bundle.validate_args(bundle_args)
        _bundle_paths, bundle_result, summary = validation_bundle.run_bundle(bundle_args)
    except (FileNotFoundError, ValueError) as exc:
        return StepResult("Validation bundle", STATUS_FAIL, str(exc)), ""
    status = STATUS_PASS if bundle_result.ok else STATUS_FAIL
    detail = "validation and gap gates passed" if bundle_result.ok else "validation or gap gate failed"
    return StepResult("Validation bundle", status, detail), summary


def locator_roots(args) -> tuple[Path, ...]:
    if args.locator_root:
        return tuple(args.locator_root)
    repo_root = Path(__file__).resolve().parents[2]
    return input_locator.default_roots(repo_root)


def first_non_smoke_candidate(report: input_locator.LocateReport, role: str) -> input_locator.Candidate | None:
    for candidate in report.by_role(role):
        if "smoke-sized candidate" in candidate.detail:
            continue
        if "not baseline-ready" in candidate.detail:
            continue
        return candidate
    return None


def add_benchmark_source(args, path: Path) -> bool:
    if not args.benchmark_markdown:
        repo_markdown = readiness.repo_benchmark_markdown()
        if repo_markdown.exists():
            args.benchmark_markdown.append(repo_markdown)
    if path in args.benchmark_markdown:
        return False
    args.benchmark_markdown.append(path)
    return True


def apply_located_inputs(args, report: input_locator.LocateReport) -> tuple[str, ...]:
    filled = []
    path_roles = (
        ("reference", "reference_tum", "reference"),
        ("bag", "ros2_bag", "ROS 2 bag"),
        ("visual", "visual_tum", "visual TUM"),
        ("csv", "aqua_slam_csv", "AQUA-SLAM CSV"),
        ("tum", "aqua_slam_tum", "AQUA-SLAM TUM"),
    )
    for attr, role, label in path_roles:
        if getattr(args, attr) is not None:
            continue
        candidate_path = input_locator.first_path(report, role)
        if candidate_path is None:
            continue
        setattr(args, attr, candidate_path)
        filled.append(f"{label}={candidate_path}")

    baseline_candidate = first_non_smoke_candidate(report, "baseline_row")
    if baseline_candidate is not None and add_benchmark_source(args, baseline_candidate.path):
        filled.append(f"baseline row source={baseline_candidate.path}")
    return tuple(filled)


def run_input_locator(args, paths: WorkflowPaths) -> StepResult:
    roots = locator_roots(args)
    report = input_locator.locate_inputs(args.sequence, roots, args.locator_max_depth)
    locator_args = argparse.Namespace(
        sequence=args.sequence,
        profile=args.profile,
        benchmark_markdown=args.baseline_row
        or readiness.baseline_paths(args).benchmark_row,
        out=paths.locator,
    )
    write_text(paths.locator, input_locator.format_report(locator_args, report))
    filled = apply_located_inputs(args, report)
    if filled:
        return StepResult(
            "Input locator",
            STATUS_PASS,
            f"filled {len(filled)} input(s); report={paths.locator}",
        )
    if report.candidates:
        return StepResult(
            "Input locator",
            STATUS_BLOCKED,
            f"found {len(report.candidates)} candidate(s) but no workflow-ready inputs; report={paths.locator}",
        )
    return StepResult("Input locator", STATUS_BLOCKED, f"no candidates found; report={paths.locator}")


def workflow_status(steps: list[StepResult]) -> str:
    if any(step.status == STATUS_FAIL for step in steps):
        return STATUS_FAIL
    validation = next((step for step in steps if step.name == "Validation bundle"), None)
    if validation is not None and validation.status == STATUS_PASS:
        return STATUS_PASS
    return STATUS_BLOCKED


def format_step_table(steps: tuple[StepResult, ...]) -> list[str]:
    lines = [
        "| Step | Status | Detail |",
        "|------|--------|--------|",
    ]
    for step in steps:
        lines.append(f"| {step.name} | {step.status} | {step.detail} |")
    return lines


def format_workflow_summary(result: WorkflowResult) -> str:
    report = result.readiness_report
    args = report.args
    action = todo_summary.next_action(list(result.todos))
    lines = [
        "# Tank AQUA-SLAM Baseline Workflow",
        "",
        f"- Status: `{result.status}`",
        f"- Dataset: `{report.dataset}`",
        f"- Sequence: `{report.sequence}`",
        f"- Baseline system: `{args.baseline_system}`",
        f"- Target system: `{args.target_system}`",
        "",
        "## Steps",
        "",
        *format_step_table(result.steps),
        "",
        "## Outputs",
        "",
        f"- Readiness report: `{result.paths.readiness}`",
        f"- TODO summary: `{result.paths.todos}`",
        f"- Workflow summary: `{result.paths.summary}`",
        f"- Validation output dir: `{result.paths.validation_out_dir}`",
        "",
        "## Current Readiness",
        "",
        f"- Baseline ingest inputs: `{readiness.pass_fail(report.ingest_ready)}`",
        f"- Baseline row for gap checks: `{readiness.pass_fail(report.baseline_row_ready)}`",
        f"- Held-out validation bundle inputs: `{readiness.pass_fail(report.validation_ready)}`",
        "",
        "## Next Action",
        "",
    ]
    if any(step.name == "Input locator" for step in result.steps):
        insert_at = lines.index(f"- Validation output dir: `{result.paths.validation_out_dir}`")
        lines.insert(insert_at, f"- Input locator report: `{result.paths.locator}`")
    if result.status == STATUS_PASS:
        lines.append("Workflow completed and validation passed.")
    elif action is None:
        lines.append("No remaining TODO action was generated.")
    else:
        lines.append(f"{action.title}: {action.detail}")
        lines.extend(todo_summary.fence_command(action.command))
    if result.validation_summary:
        lines.extend(["", "## Validation Summary", "", result.validation_summary.strip()])
    return "\n".join(lines) + "\n"


def refresh_readiness_artifacts(args, paths: WorkflowPaths) -> tuple[readiness.ReadinessReport, tuple[todo_summary.TodoItem, ...]]:
    report = readiness.build_report(args)
    todos = tuple(todo_summary.build_todos(report))
    write_text(paths.readiness, readiness.format_report(report))
    write_text(paths.todos, todo_summary.format_todos(report, list(todos)))
    return report, todos


def run_workflow(args) -> WorkflowResult:
    # Fill readiness defaults before deriving workflow output paths.
    paths = workflow_paths(args)
    steps: list[StepResult] = []
    if args.auto_locate_inputs:
        steps.append(run_input_locator(args, paths))

    initial_report = readiness.build_report(args)
    steps.append(
        StepResult(
            "Initial readiness",
            STATUS_PASS if initial_report.ingest_ready or initial_report.baseline_row_ready else STATUS_BLOCKED,
            (
                "baseline ingest inputs or row are available"
                if initial_report.ingest_ready or initial_report.baseline_row_ready
                else "waiting for reference and AQUA-SLAM trajectory source"
            ),
        )
    )

    steps.append(run_ingest_if_ready(args, initial_report))
    final_report, todos = refresh_readiness_artifacts(args, paths)
    steps.append(StepResult("TODO summary", STATUS_PASS, f"wrote {paths.todos}"))
    validation_step, validation_summary = run_validation_if_ready(args, paths, final_report)
    steps.append(validation_step)

    status = workflow_status(steps)
    result = WorkflowResult(
        status=status,
        steps=tuple(steps),
        paths=paths,
        readiness_report=final_report,
        todos=todos,
        validation_summary=validation_summary,
    )
    write_text(paths.summary, format_workflow_summary(result))
    return result


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Run the Tank AQUA-SLAM baseline ingest and DVL prior validation workflow."
    )
    parser.add_argument("--sequence", default=readiness.DEFAULT_SEQUENCE)
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
    parser.add_argument(
        "--profile",
        type=Path,
        default=readiness.DEFAULT_PROFILE,
    )
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--visual", type=Path)
    parser.add_argument("--time-unit", choices=("auto", "seconds", "nanoseconds"), default="auto")
    parser.add_argument("--source-topic", default=baseline_ingest.DEFAULT_SOURCE)
    parser.add_argument("--config", default="underwater_orbslam3_blue_gx5_medium.yaml")
    parser.add_argument("--min-baseline-samples", type=int, default=readiness.DEFAULT_MIN_BASELINE_SAMPLES)
    parser.add_argument("--min-baseline-matched-s", type=float, default=readiness.DEFAULT_MIN_BASELINE_MATCHED_S)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--readiness-out", type=Path)
    parser.add_argument("--todos-out", type=Path)
    parser.add_argument("--summary-out", type=Path)
    parser.add_argument("--locator-out", type=Path)
    parser.add_argument("--validation-out-dir", type=Path)
    parser.add_argument("--auto-locate-inputs", action="store_true")
    parser.add_argument("--locator-root", action="append", type=Path, default=[])
    parser.add_argument("--locator-max-depth", type=int, default=7)
    parser.add_argument("--baseline-runtime", default=baseline_ingest.DEFAULT_RUNTIME)
    parser.add_argument("--baseline-commit", default="")
    parser.add_argument("--baseline-note", default="")
    parser.add_argument("--scale", action="store_true")
    parser.add_argument("--no-align", action="store_true")
    parser.add_argument("--skip-ingest", action="store_true")
    parser.add_argument("--skip-validation", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-residual-analysis", action="store_true")
    parser.add_argument("--allow-same-sequence", action="store_true")
    parser.add_argument("--allow-profile-sequence-mismatch", action="store_true")
    parser.add_argument("--max-corrected-rmse-m", type=float)
    parser.add_argument("--min-improvement-percent", type=float)
    parser.add_argument("--fail-on-gate-failure", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--max-gap-x", type=float, default=1.0)
    parser.add_argument("--max-improvement-to-tie-percent", type=float)
    parser.add_argument("--residual-top-k", type=int, default=10)
    parser.add_argument("--validation-note", default="")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        result = run_workflow(args)
    except (FileNotFoundError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    summary = format_workflow_summary(result)
    print(summary)
    if result.status == STATUS_PASS:
        return 0
    if result.status == STATUS_BLOCKED:
        return 1
    return 2


if __name__ == "__main__":
    sys.exit(main())
