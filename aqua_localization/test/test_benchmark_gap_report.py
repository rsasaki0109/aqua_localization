"""Tests for benchmark_gap_report.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "benchmark_gap_report.py"


def load_module():
    spec = importlib.util.spec_from_file_location("benchmark_gap_report", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sample_markdown() -> str:
    return """
# Benchmarks

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | baseline |
| Tank Dataset | short_test | aqua_visual_frontend | SE(3) | 200 | 11.25 | 0.0815 | 0.0792 | 0.0947 | 0.2416 | target |
| Tank Dataset | Medium | aqua_visual_frontend | TBD | TBD | TBD | TBD | TBD | TBD | TBD | pending |

| Dataset | Sequence | System | Inputs | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | `aqua_visual_frontend` | stereo only | Sim(3) | 200 | 11.35 | 0.0826 | 0.0786 | 0.0958 | 0.2458 | diagnostic |
"""


def test_parse_markdown_benchmark_rows_handles_optional_inputs_column():
    module = load_module()

    rows = module.parse_markdown_benchmark_rows(sample_markdown())

    assert len(rows) == 3
    assert rows[0].system == "AQUA-SLAM"
    assert rows[2].system == "aqua_visual_frontend"
    assert rows[2].alignment == "Sim(3)"
    assert rows[2].rmse_m == 0.0958


def test_compute_gaps_reports_matching_alignment_only():
    module = load_module()
    rows = module.parse_markdown_benchmark_rows(sample_markdown())

    gaps = module.compute_gaps(rows, "aqua_visual_frontend", "AQUA-SLAM")

    assert len(gaps) == 1
    assert gaps[0].sequence == "short_test"
    assert gaps[0].ratio == 0.0947 / 0.0194
    assert gaps[0].improvement_to_tie_percent > 79.0


def test_format_report_includes_gap_table():
    module = load_module()
    rows = module.parse_markdown_benchmark_rows(sample_markdown())
    gaps = module.compute_gaps(rows, "aqua_visual_frontend", "AQUA-SLAM")

    report = module.format_report(gaps, "aqua_visual_frontend", "AQUA-SLAM")

    assert "# Benchmark Gap Report" in report
    assert "| Tank Dataset | short_test | SE(3) | 0.0947 | 0.0194 | 4.88 | 79.5%" in report
    assert "Closest case" in report


def test_cli_writes_report(tmp_path):
    src = tmp_path / "benchmarks.md"
    out = tmp_path / "gap.md"
    src.write_text(sample_markdown(), encoding="utf-8")

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(src),
            "--target-system",
            "aqua_visual_frontend",
            "--baseline-system",
            "AQUA-SLAM",
            "--out",
            str(out),
        ],
        check=True,
    )

    assert "Benchmark Gap Report" in out.read_text(encoding="utf-8")


def test_gate_failures_report_threshold_misses():
    module = load_module()
    rows = module.parse_markdown_benchmark_rows(sample_markdown())
    gaps = module.compute_gaps(rows, "aqua_visual_frontend", "AQUA-SLAM")

    failures = module.gate_failures(
        gaps,
        max_gap_x=4.0,
        max_improvement_to_tie_percent=70.0,
    )

    assert len(failures) == 2
    assert "gap" in failures[0]
    assert "improvement-to-tie" in failures[1]


def test_cli_gap_gate_passes(tmp_path):
    src = tmp_path / "benchmarks.md"
    src.write_text(sample_markdown(), encoding="utf-8")

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(src),
            "--target-system",
            "aqua_visual_frontend",
            "--baseline-system",
            "AQUA-SLAM",
            "--max-gap-x",
            "5.0",
            "--max-improvement-to-tie-percent",
            "80.0",
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 0
    assert "4.88" in proc.stdout
    assert proc.stderr == ""


def test_cli_gap_gate_fails(tmp_path):
    src = tmp_path / "benchmarks.md"
    src.write_text(sample_markdown(), encoding="utf-8")

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(src),
            "--target-system",
            "aqua_visual_frontend",
            "--baseline-system",
            "AQUA-SLAM",
            "--max-gap-x",
            "4.0",
        ],
        text=True,
        capture_output=True,
    )

    assert proc.returncode == 2
    assert "benchmark gap gate failed" in proc.stderr
    assert "exceeds --max-gap-x" in proc.stderr
