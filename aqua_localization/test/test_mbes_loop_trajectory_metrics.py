"""Tests for MBES loop trajectory metric formatting helpers."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "mbes_loop_trajectory_metrics.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("mbes_loop_trajectory_metrics", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    import sys
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def stats(rmse, median):
    return {
        "count": 5,
        "matched_seconds": 4.0,
        "mean": rmse + 0.1,
        "median": median,
        "rmse": rmse,
        "max": rmse + 1.0,
        "alignment": {"applied": True, "with_scale": False},
    }


def test_improvement_label_reports_direction():
    module = load_module()

    assert module.improvement_label(0.25) == "improved by 0.2500 m"
    assert module.improvement_label(-0.5) == "worse by 0.5000 m"
    assert module.improvement_label(0.0) == "unchanged"


def test_format_report_contains_loop_impact_summary(tmp_path):
    module = load_module()
    args = SimpleNamespace(
        dataset="MBES-SLAM",
        sequence="beach_pond",
        bag=Path("/tmp/bag"),
        reference_topic="/nav/processed/odometry",
        input_odometry_topic="/aqua_imu_loc/odometry",
        pose_graph_path_topic="/aqua_pose_graph/path",
        scale=False,
        no_align=False,
    )
    paths = {
        "reference": tmp_path / "reference.tum",
        "input_odometry": tmp_path / "input.tum",
        "pose_graph": tmp_path / "graph.tum",
    }
    counts = {"reference": 10, "input_odometry": 9, "pose_graph": 5}

    text = module.format_report(
        args,
        paths,
        counts,
        baseline_stats=stats(rmse=2.0, median=1.5),
        pose_graph_stats=stats(rmse=1.25, median=1.0),
    )

    assert "# MBES Loop Closure Trajectory Metrics" in text
    assert "| Input odometry | 5 | 4.00 |" in text
    assert "| Pose graph path | 5 | 4.00 |" in text
    assert "RMSE delta: improved by 0.7500 m" in text
    assert "Median delta: improved by 0.5000 m" in text
    assert "Alignment: SE(3)" in text
