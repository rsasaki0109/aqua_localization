"""Tests for AQUA-SLAM error-budget reporting."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "aqua_slam_error_budget.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("aqua_slam_error_budget", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def benchmark_markdown() -> str:
    return """
# Tank Dataset AQUA-SLAM Head-to-Head

| Dataset | Sequence | System | Alignment | Samples | Matched s | Mean m | Median m | RMSE m | Max m | Note |
|---------|----------|--------|-----------|--------:|----------:|-------:|---------:|-------:|------:|------|
| Tank Dataset | short_test | AQUA-SLAM | SE(3) | 234 | 11.65 | 0.0173 | 0.0165 | 0.0194 | 0.0579 | baseline |
| Tank Dataset | short_test | aqua_localization | SE(3) | 5399 | 14.94 | 0.3796 | 0.4014 | 0.4291 | 0.7652 | DVL anchor |
| Tank Dataset | short_test | aqua_visual_frontend | SE(3) | 200 | 11.25 | 0.0815 | 0.0792 | 0.0947 | 0.2416 | visual |
| Tank Dataset | short_test | aqua_localization+visual | SE(3) | 5400 | 14.95 | 0.1793 | 0.1394 | 0.2175 | 0.8564 | fused |
"""


def drift_report() -> str:
    return """
# Visual Drift Analysis

- Overall SE(3) RMSE: 0.0947 m
- Overall Sim(3) RMSE: 0.0520 m
- Overall Sim(3) scale: 0.169623465
- Relative std: 0.0300

## Window Drift

| Start s | End s | Samples | SE(3) RMSE m | Sim(3) RMSE m | Sim(3) scale | SE(3) max m | Sim(3) max m |
|--------:|------:|--------:|-------------:|--------------:|-------------:|------------:|-------------:|
| 0.00 | 3.00 | 40 | 0.0600 | 0.0400 | 0.170000000 | 0.1000 | 0.0800 |
| 1.00 | 4.00 | 40 | 0.1200 | 0.0500 | 0.168000000 | 0.2000 | 0.0900 |
"""


def motion_report() -> str:
    return """
# Visual Motion Segment Analysis

## Summary

| Metric | Count | Min | Median | Mean | P95 | Max | Std |
|--------|------:|----:|-------:|-----:|----:|----:|----:|
| visual/reference length ratio | 20 | 5.0 | 5.9 | 6.0 | 6.3 | 6.4 | 0.2 |
| reference/visual correction scale | 20 | 0.1500 | 0.1696 | 0.1700 | 0.1800 | 0.1900 | 0.0100 |
"""


def test_parse_optional_drift_and_motion_reports(tmp_path):
    module = load_module()
    drift = tmp_path / "drift.md"
    motion = tmp_path / "motion.md"
    drift.write_text(drift_report(), encoding="utf-8")
    motion.write_text(motion_report(), encoding="utf-8")

    drift_metrics = module.parse_drift_report(drift)
    motion_metrics = module.parse_motion_report(motion)

    assert drift_metrics.se3_rmse_m == 0.0947
    assert drift_metrics.sim3_rmse_m == 0.0520
    assert drift_metrics.worst_window_se3_rmse_m == 0.1200
    assert motion_metrics.median_correction_scale == 0.1696
    assert motion_metrics.correction_scale_std == 0.0100


def test_error_budget_report_includes_fusion_and_drift_buckets(tmp_path):
    module = load_module()
    bench = tmp_path / "tank.md"
    drift = tmp_path / "drift.md"
    motion = tmp_path / "motion.md"
    bench.write_text(benchmark_markdown(), encoding="utf-8")
    drift.write_text(drift_report(), encoding="utf-8")
    motion.write_text(motion_report(), encoding="utf-8")
    args = module.parse_args([
        str(bench),
        "--drift-report",
        str(drift),
        "--motion-report",
        str(motion),
    ])

    context = module.load_context(args)
    report = module.format_report(context, [bench])

    assert "Gap to tie: 0.0753 m" in report
    assert "Anchor improvement already banked" in report
    assert "Fusion regression budget" in report
    assert "Scale/extrinsic removable component" in report
    assert "Post-Sim(3) drift floor" in report
    assert "Worst drift window" in report
    assert "Motion scale bias" in report


def test_cli_writes_error_budget(tmp_path):
    bench = tmp_path / "tank.md"
    out = tmp_path / "budget.md"
    bench.write_text(benchmark_markdown(), encoding="utf-8")

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(bench),
            "--out",
            str(out),
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "wrote AQUA-SLAM error budget" in proc.stdout
    assert "AQUA-SLAM Error Budget" in out.read_text(encoding="utf-8")
