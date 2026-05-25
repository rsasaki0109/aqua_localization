"""Tests for AQUA-SLAM head-to-head diagnosis reports."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "aqua_slam_head_to_head_report.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("aqua_slam_head_to_head_report", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sample_markdown() -> str:
    return """
# Tank Dataset AQUA-SLAM Head-to-Head

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | P95 m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|------:|-------:|------:|------|
| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0500 | 0.0194 | 0.0579 | baseline |
| Tank Dataset | short_test | aqua_visual_frontend | SE(3) | 200 | 11.25 | 0.0815 | 0.0792 | 0.2100 | 0.0947 | 0.2416 | same-sequence scale fit |
| Tank Dataset | short_test | aqua_dvl_prior_visual | SE(3) | 218 | 11.10 | 0.0141 | 0.0132 | 0.0300 | 0.0154 | 0.0342 | diagnostic override |
| Tank Dataset | Medium | AQUA-SLAM | SE(3) | 3 | 2.00 | 0.0200 | 0.0190 | TBD | 0.0210 | 0.0500 | smoke row |
| Tank Dataset | Medium | aqua_visual_frontend | SE(3) | 42 | 12.50 | 0.0180 | 0.0170 | TBD | 0.0185 | 0.0420 | held-out validation |
| Tank Dataset | Structure_Easy | AQUA-SLAM | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | record output topic |
| Tank Dataset | Structure_Easy | aqua_localization | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | run closest input mode |
"""


def test_parse_metric_rows_keeps_optional_metrics_and_pending_rows():
    module = load_module()

    rows = module.parse_metric_rows(sample_markdown())

    assert len(rows) == 7
    assert rows[0].p95_m == 0.05
    assert rows[4].p95_m is None
    assert rows[-1].measured is False


def test_collect_comparisons_selects_best_current_row_per_case():
    module = load_module()
    rows = module.parse_metric_rows(sample_markdown())

    comparisons = module.collect_comparisons(
        rows,
        baseline_system="AQUA-SLAM",
        target_prefixes=["aqua_"],
    )
    by_sequence = {comparison.sequence: comparison for comparison in comparisons}

    assert by_sequence["short_test"].target.system == "aqua_dvl_prior_visual"
    assert by_sequence["short_test"].gap_x == 0.0154 / 0.0194
    assert by_sequence["Medium"].target.system == "aqua_visual_frontend"
    assert by_sequence["Structure_Easy"].target is None
    assert len(by_sequence["Structure_Easy"].pending_targets) == 1


def test_reasons_and_verdicts_distinguish_diagnostic_and_short_baseline():
    module = load_module()
    rows = module.parse_metric_rows(sample_markdown())
    args = module.parse_args(["bench.md"])
    comparisons = module.collect_comparisons(
        rows,
        baseline_system="AQUA-SLAM",
        target_prefixes=["aqua_"],
    )
    by_sequence = {comparison.sequence: comparison for comparison in comparisons}

    short_reasons = module.comparison_reasons(by_sequence["short_test"], args)
    medium_reasons = module.comparison_reasons(by_sequence["Medium"], args)

    assert "current row is diagnostic" in short_reasons
    assert "held-out validation not established" in short_reasons
    assert module.verdict(by_sequence["short_test"], short_reasons) == "diagnostic win"
    assert "baseline row too short: 3 samples < 10" in medium_reasons
    assert "baseline matched duration too short: 2.00s < 10.00s" in medium_reasons
    assert module.verdict(by_sequence["Medium"], medium_reasons) == "win, evidence blocked"


def test_format_report_includes_claim_table_and_github_summary(tmp_path):
    module = load_module()
    rows = module.parse_metric_rows(sample_markdown())
    args = module.parse_args(["bench.md"])
    comparisons = module.collect_comparisons(
        rows,
        baseline_system="AQUA-SLAM",
        target_prefixes=["aqua_"],
    )

    report = module.format_report(
        comparisons,
        source_paths=[tmp_path / "bench.md"],
        baseline_system="AQUA-SLAM",
        target_prefixes=["aqua_"],
        args=args,
    )

    assert "# AQUA-SLAM Head-to-Head Diagnosis" in report
    assert "| Tank Dataset | short_test | SE(3) | aqua_dvl_prior_visual | diagnostic win" in report
    assert "current row is diagnostic; held-out validation not established" in report
    assert "0.0141 / 0.0173" in report
    assert "## GitHub-Safe Summary" in report
    assert "not claimable yet" in report


def test_cli_writes_report(tmp_path):
    src = tmp_path / "benchmarks.md"
    out = tmp_path / "head_to_head.md"
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

    assert "wrote AQUA-SLAM head-to-head report" in proc.stdout
    assert "AQUA-SLAM Head-to-Head Diagnosis" in out.read_text(encoding="utf-8")
