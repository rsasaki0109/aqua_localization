"""Regression tests for compare_mbes_repeat_probe.py."""

import csv
import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "compare_mbes_repeat_probe.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("compare_mbes_repeat_probe", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, positions, *, duplicate_first=False):
    lines = []
    for index, position in enumerate(positions):
        stamp = float(index)
        line = (
            f"{stamp:.9f} {position:.9f} 0.000000000 0.000000000 "
            "0.000000000 0.000000000 0.000000000 1.000000000\n"
        )
        lines.append(line)
        if index == 0 and duplicate_first:
            lines.append(line)
    path.write_text("".join(lines), encoding="utf-8")


def write_loop_status(path: Path, pairs):
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(
            fp,
            fieldnames=[
                "accepted",
                "current_keyframe_timestamp",
                "candidate_keyframe_timestamp",
            ],
        )
        writer.writeheader()
        for current, candidate, accepted in pairs:
            writer.writerow(
                {
                    "accepted": int(accepted),
                    "current_keyframe_timestamp": current,
                    "candidate_keyframe_timestamp": candidate,
                }
            )


def write_estimator_status(path: Path, update_count: int, applied: int):
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(
            fp,
            fieldnames=[
                "update_count",
                "sonar_feedback_received",
                "sonar_feedback_applied",
                "sonar_feedback_skipped_stale",
                "sonar_feedback_skipped_nonfinite",
                "sonar_feedback_pending",
            ],
        )
        writer.writeheader()
        writer.writerow(
            {
                "update_count": update_count,
                "sonar_feedback_received": applied,
                "sonar_feedback_applied": applied,
                "sonar_feedback_skipped_stale": 0,
                "sonar_feedback_skipped_nonfinite": 0,
                "sonar_feedback_pending": 0,
            }
        )


def make_run(root: Path, name: str, positions, *, duplicate_first=False):
    run = root / name
    run.mkdir()
    write_tum(
        run / "input_odometry.tum",
        positions,
        duplicate_first=duplicate_first,
    )
    write_loop_status(run / "mbes_loop_status.csv", [("10.0", "2.0", True)])
    write_estimator_status(run / "estimator_status.csv", 20, 4)
    return run


def test_matching_runs_report_matching_hash_and_counters(tmp_path):
    module = load_module()
    run1 = make_run(tmp_path, "run1", [0.0, 1.0, 2.0])
    run2 = make_run(tmp_path, "run2", [0.0, 1.0, 2.0])

    first = module.collect_run(run1)
    second = module.collect_run(run2)
    agreement = module.windowed_agreement(first.trajectory, second.trajectory)
    report = module.render_report(first, second, agreement, 0.01)

    assert first.tum.sha256 == second.tum.sha256
    assert agreement.mean_diff_m == 0.0
    assert "| `update_count` | 20 | 20 | yes |" in report
    assert "| Gate (mean <= 0.01 m) | PASS |" in report


def test_hash_mismatch_and_window_difference_are_visible(tmp_path):
    module = load_module()
    run1 = make_run(tmp_path, "run1", [0.0, 1.0, 2.0])
    run2 = make_run(tmp_path, "run2", [0.0, 2.0, 4.0])

    first = module.collect_run(run1)
    second = module.collect_run(run2)
    agreement = module.windowed_agreement(first.trajectory, second.trajectory)
    report = module.render_report(first, second, agreement, 0.1)

    assert first.tum.sha256 != second.tum.sha256
    assert agreement.mean_diff_m == 1.0
    assert "| Gate (mean <= 0.1 m) | FAIL |" in report


def test_duplicate_stamp_warning_is_reported(tmp_path):
    module = load_module()
    run1 = make_run(tmp_path, "run1", [0.0, 1.0], duplicate_first=True)
    run2 = make_run(tmp_path, "run2", [0.0, 1.0])

    first = module.collect_run(run1)
    second = module.collect_run(run2)
    agreement = module.windowed_agreement(first.trajectory, second.trajectory)
    report = module.render_report(first, second, agreement, 1.0)

    assert first.tum.duplicate_stamps == 1
    assert "Duplicate stamps mean multiple publishers" in report


def test_loop_endpoint_overlap_lists_run_specific_pairs(tmp_path):
    module = load_module()
    run1 = make_run(tmp_path, "run1", [0.0, 1.0])
    run2 = make_run(tmp_path, "run2", [0.0, 1.0])
    write_loop_status(
        run1 / "mbes_loop_status.csv",
        [("10.0", "2.0", True), ("20.0", "5.0", True)],
    )
    write_loop_status(
        run2 / "mbes_loop_status.csv",
        [("10.0", "2.0", True), ("30.0", "7.0", True)],
    )

    first = module.collect_run(run1)
    second = module.collect_run(run2)
    report = module.render_report(
        first,
        second,
        module.windowed_agreement(first.trajectory, second.trajectory),
        1.0,
    )

    assert first.loop_pairs & second.loop_pairs == {("10.0", "2.0")}
    assert "| only in run1 | `20.0` | `5.0` | no |" in report
    assert "| only in run2 | `30.0` | `7.0` | no |" in report


def test_required_window_gate_returns_failure(tmp_path):
    module = load_module()
    run1 = make_run(tmp_path, "run1", [0.0, 1.0, 2.0])
    run2 = make_run(tmp_path, "run2", [0.0, 2.0, 4.0])
    out = tmp_path / "comparison.md"

    result = module.main(
        [
            str(run1),
            str(run2),
            "--out",
            str(out),
            "--window-agreement-mean-max-m",
            "0.1",
            "--require-window-agreement",
        ]
    )

    assert result == 1
    assert "FAIL" in out.read_text(encoding="utf-8")

