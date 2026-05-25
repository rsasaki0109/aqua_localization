"""Tests for AQUA-SLAM claim readiness reports."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "aqua_slam_claim_readiness_report.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("aqua_slam_claim_readiness_report", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def diagnostic_markdown() -> str:
    return """
# AQUA-SLAM Head-to-Head

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | baseline |
| Tank Dataset | short_test | aqua_dvl_prior_visual | SE(3) | 218 | 11.10 | 0.0141 | 0.0132 | 0.0154 | 0.0342 | diagnostic override |
| Tank Dataset | Medium | aqua_dvl_prior_visual | TBD | TBD | TBD | TBD | TBD | TBD | TBD | held-out validation after short_test calibration |
"""


def claimable_markdown() -> str:
    return """
# AQUA-SLAM Head-to-Head

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | Medium | AQUA-SLAM | SE(3) | 42 | 12.50 | 0.0200 | 0.0190 | 0.0210 | 0.0500 | baseline |
| Tank Dataset | Medium | aqua_dvl_prior_visual | SE(3) | 42 | 12.50 | 0.0160 | 0.0150 | 0.0180 | 0.0400 | held-out validation |
"""


def args_for_tmp(module, tmp_path, markdown):
    return module.parse_args([
        str(markdown),
        "--reference",
        str(tmp_path / "Medium_gt.tum"),
        "--bag",
        str(tmp_path / "Medium_ros2"),
        "--visual",
        str(tmp_path / "Medium_visual_frontend.tum"),
        "--csv",
        str(tmp_path / "aqua_slam_medium_orb_odom.csv"),
        "--tum",
        str(tmp_path / "Medium_aqua_slam.tum"),
        "--baseline-row",
        str(tmp_path / "Medium_aqua_slam_benchmark_row.md"),
        "--profile",
        str(tmp_path / "best_profile.yaml"),
        "--locator-root",
        str(tmp_path / "scan"),
    ])


def test_report_blocks_diagnostic_win_and_points_to_heldout_input(tmp_path):
    module = load_module()
    markdown = tmp_path / "bench.md"
    markdown.write_text(diagnostic_markdown(), encoding="utf-8")

    state = module.build_readiness(args_for_tmp(module, tmp_path, markdown))
    report = module.format_report(state)

    assert state.claimable is False
    assert state.best_numeric.target.system == "aqua_dvl_prior_visual"
    assert state.heldout.next_action.title == "Find Medium reference TUM"
    assert "Status: `BLOCKED`" in report
    assert "0.79x" in report
    assert "current row is diagnostic; held-out validation not established" in report
    assert "--apply-located-links" in report
    assert "Find Medium reference TUM" in report


def test_report_marks_existing_heldout_win_claimable(tmp_path):
    module = load_module()
    markdown = tmp_path / "bench.md"
    markdown.write_text(claimable_markdown(), encoding="utf-8")

    state = module.build_readiness(args_for_tmp(module, tmp_path, markdown))
    report = module.format_report(state)

    assert state.claimable is True
    assert state.best_claimable_win.target.system == "aqua_dvl_prior_visual"
    assert "Status: `CLAIMABLE_WIN`" in report
    assert "Best claimable win: `aqua_dvl_prior_visual` on `Medium` at 0.86x." in report


def test_cli_writes_blocked_report_and_returns_nonzero(tmp_path):
    markdown = tmp_path / "bench.md"
    out = tmp_path / "claim.md"
    markdown.write_text(diagnostic_markdown(), encoding="utf-8")

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(markdown),
            "--reference",
            str(tmp_path / "Medium_gt.tum"),
            "--baseline-row",
            str(tmp_path / "Medium_aqua_slam_benchmark_row.md"),
            "--locator-root",
            str(tmp_path / "scan"),
            "--out",
            str(out),
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 1
    assert "wrote AQUA-SLAM claim readiness report" in proc.stdout
    assert "Status: `BLOCKED`" in out.read_text(encoding="utf-8")


def test_cli_returns_zero_for_claimable_report(tmp_path):
    markdown = tmp_path / "bench.md"
    markdown.write_text(claimable_markdown(), encoding="utf-8")

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(markdown),
            "--reference",
            str(tmp_path / "Medium_gt.tum"),
            "--baseline-row",
            str(tmp_path / "Medium_aqua_slam_benchmark_row.md"),
            "--locator-root",
            str(tmp_path / "scan"),
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 0
    assert "Status: `CLAIMABLE_WIN`" in proc.stdout
