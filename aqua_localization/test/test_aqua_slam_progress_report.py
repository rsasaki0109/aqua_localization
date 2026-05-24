"""Tests for AQUA-SLAM progress report generation."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "aqua_slam_progress_report.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("aqua_slam_progress_report", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sample_markdown() -> str:
    return """
# Tank Dataset AQUA-SLAM Head-to-Head

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | baseline |
| Tank Dataset | short_test | aqua_localization | SE(3) | 5399 | 14.94 | 0.3796 | 0.4014 | 0.4291 | 0.7652 | DVL anchor |
| Tank Dataset | short_test | aqua_visual_frontend | SE(3) | 200 | 11.25 | 0.0815 | 0.0792 | 0.0947 | 0.2416 | visual |
| Tank Dataset | short_test | aqua_localization+visual | SE(3) | 5400 | 14.95 | 0.1793 | 0.1394 | 0.2175 | 0.8564 | fused |
| Tank Dataset | short_test | aqua_dvl_prior_visual | SE(3) | 218 | 11.10 | 0.0141 | 0.0132 | 0.0154 | 0.0342 | diagnostic override |
| Tank Dataset | Medium | aqua_visual_frontend | TBD | TBD | TBD | TBD | TBD | TBD | TBD | pending |
"""


def test_collect_progress_rows_reports_all_aqua_targets():
    module = load_module()
    rows = module.gap_report.parse_markdown_benchmark_rows(sample_markdown())

    progress = module.collect_progress_rows(
        rows,
        baseline_system="AQUA-SLAM",
        anchor_system="aqua_localization",
        target_prefixes=["aqua_"],
    )

    assert [row.target.system for row in progress] == [
        "aqua_dvl_prior_visual",
        "aqua_visual_frontend",
        "aqua_localization+visual",
        "aqua_localization",
    ]
    assert progress[0].gap_x == 0.0154 / 0.0194
    assert progress[0].diagnostic is True
    assert module.best_claimable_progress(progress).target.system == "aqua_visual_frontend"


def test_format_report_includes_best_readout_and_anchor_improvement(tmp_path):
    module = load_module()
    rows = module.gap_report.parse_markdown_benchmark_rows(sample_markdown())
    progress = module.collect_progress_rows(
        rows,
        baseline_system="AQUA-SLAM",
        anchor_system="aqua_localization",
        target_prefixes=["aqua_"],
    )

    report = module.format_report(
        progress,
        source_paths=[tmp_path / "tank.md"],
        baseline_system="AQUA-SLAM",
        anchor_system="aqua_localization",
        target_prefixes=["aqua_"],
    )

    assert "# AQUA-SLAM Progress Report" in report
    assert "| Tank Dataset | short_test | SE(3) | aqua_dvl_prior_visual | 0.0154 | 0.79x" in report
    assert "| diagnostic | diagnostic override |" in report
    assert "Best current row: `aqua_dvl_prior_visual`" in report
    assert "Best current row is diagnostic" in report
    assert "Best non-diagnostic row: `aqua_visual_frontend`" in report
    assert "Improvement versus `aqua_localization` anchor" in report


def test_cli_writes_progress_report(tmp_path):
    src = tmp_path / "tank.md"
    out = tmp_path / "progress.md"
    src.write_text(sample_markdown(), encoding="utf-8")

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(src),
            "--out",
            str(out),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "wrote AQUA-SLAM progress report" in proc.stdout
    assert "AQUA-SLAM Progress Report" in out.read_text(encoding="utf-8")
