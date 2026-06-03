"""Tests for compare_mbes_loop_metric_reports.py."""

import importlib.util
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "compare_mbes_loop_metric_reports.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("compare_mbes_loop_metric_reports", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_report(path: Path, input_rmse: float, graph_rmse: float, graph_median: float):
    path.write_text(
        "\n".join([
            "# MBES Loop Closure Trajectory Metrics",
            "",
            "| Estimate | Matched samples | Matched s | Mean m | Median m | RMSE m | Max m |",
            "|----------|----------------:|----------:|-------:|---------:|-------:|------:|",
            f"| Input odometry | 10 | 9.00 | 1.0 | 0.8 | {input_rmse:.4f} | 2.0 |",
            f"| Pose graph path | 8 | 7.00 | 0.9 | {graph_median:.4f} | {graph_rmse:.4f} | 1.8 |",
            "",
        ]),
        encoding="utf-8",
    )


def test_parse_metric_report_reads_two_metric_rows(tmp_path):
    module = load_module()
    report = tmp_path / "metrics.md"
    write_report(report, input_rmse=2.0, graph_rmse=1.5, graph_median=1.1)

    metrics = module.parse_metric_report(report)

    assert metrics["Input odometry"].rmse_m == 2.0
    assert metrics["Pose graph path"].matched_samples == 8
    assert metrics["Pose graph path"].median_m == 1.1


def test_format_comparison_reports_selected_vs_normal_delta(tmp_path):
    module = load_module()
    normal_path = tmp_path / "normal.md"
    selected_path = tmp_path / "selected.md"
    write_report(normal_path, input_rmse=2.0, graph_rmse=1.6, graph_median=1.2)
    write_report(selected_path, input_rmse=2.0, graph_rmse=1.1, graph_median=0.9)
    normal = module.parse_metric_report(normal_path)
    selected = module.parse_metric_report(selected_path)

    text = module.format_comparison(normal_path, selected_path, normal, selected)

    assert "# MBES Selected-Loop Replay Comparison" in text
    assert "| normal | 2.0000 | 1.6000 | improved by 0.4000 m |" in text
    assert "| selected-loop | 2.0000 | 1.1000 | improved by 0.9000 m |" in text
    assert "Pose graph RMSE: improved by 0.5000 m" in text
    assert "Pose graph median: improved by 0.3000 m" in text
    assert "## Replay Baseline Drift" in text
    assert "Input odometry RMSE: unchanged" in text
    assert "WARNING:" not in text


def test_format_comparison_warns_when_input_baselines_differ(tmp_path):
    module = load_module()
    normal_path = tmp_path / "normal.md"
    selected_path = tmp_path / "selected.md"
    write_report(normal_path, input_rmse=10.0, graph_rmse=8.0, graph_median=5.0)
    write_report(selected_path, input_rmse=2.0, graph_rmse=7.0, graph_median=4.0)
    normal = module.parse_metric_report(normal_path)
    selected = module.parse_metric_report(selected_path)

    text = module.format_comparison(normal_path, selected_path, normal, selected)

    assert "Input odometry RMSE: improved by 8.0000 m" in text
    assert "WARNING: normal and selected replays do not share" in text
