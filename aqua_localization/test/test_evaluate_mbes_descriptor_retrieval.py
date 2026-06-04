"""Tests for evaluate_mbes_descriptor_retrieval.py."""

import importlib.util
from pathlib import Path
import sys


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "evaluate_mbes_descriptor_retrieval.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location(
        "evaluate_mbes_descriptor_retrieval", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, text: str):
    path.write_text(text.strip() + "\n", encoding="utf-8")


def test_descriptor_ranking_recalls_endpoint_positive_at_top1(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    write_csv(
        selected_path,
        """
timestamp,current_keyframe_timestamp,candidate_keyframe_timestamp,current_id,candidate_id,descriptor_centroid_distance_m,descriptor_extent_ratio,descriptor_point_count_ratio
10.0,10.0,5.0,100,4,0.20,1.10,0.90
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_keyframe_timestamp,candidate_keyframe_timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad,descriptor_centroid_distance_m,descriptor_extent_ratio,descriptor_point_count_ratio
10.1,10.1,8.0,101,7,0,loop selection rejected,0.10,0.5,0.05,0.80,1.10,0.90
10.2,10.2,5.1,102,8,0,loop selection rejected,0.11,0.5,0.05,0.21,1.09,0.91
""",
    )

    signatures = module.read_selected(selected_path)
    status_rows = module.read_status(status_path)
    results = module.evaluate_retrieval(
        signatures,
        status_rows,
        current_timestamp_window_s=1.0,
        candidate_timestamp_window_s=0.25,
        centroid_scale_m=0.6,
        extent_scale=0.15,
        point_ratio_scale=0.10,
    )

    assert len(results) == 1
    assert results[0].endpoint_rank == 1
    assert results[0].exact_rank is None


def test_descriptor_ranking_treats_zero_fitness_as_best_tie_breaker(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    write_csv(
        selected_path,
        """
timestamp,current_keyframe_timestamp,candidate_keyframe_timestamp,current_id,candidate_id,descriptor_centroid_distance_m,descriptor_extent_ratio,descriptor_point_count_ratio
10.0,10.0,5.0,100,4,0.20,1.10,0.90
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_keyframe_timestamp,candidate_keyframe_timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad,descriptor_centroid_distance_m,descriptor_extent_ratio,descriptor_point_count_ratio
10.0,10.0,5.2,101,20,0,loop selection rejected,0.50,0.5,0.05,0.20,1.10,0.90
10.0,10.0,5.1,102,30,0,loop selection rejected,0.00,0.5,0.05,0.20,1.10,0.90
""",
    )

    signatures = module.read_selected(selected_path)
    status_rows = module.read_status(status_path)
    results = module.evaluate_retrieval(
        signatures,
        status_rows,
        current_timestamp_window_s=1.0,
        candidate_timestamp_window_s=0.25,
        centroid_scale_m=0.6,
        extent_scale=0.15,
        point_ratio_scale=0.10,
    )

    assert results[0].candidates[0].row.candidate_id == 30


def test_format_report_marks_no_endpoint_positive_available(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    write_csv(
        selected_path,
        """
timestamp,current_keyframe_timestamp,candidate_keyframe_timestamp,current_id,candidate_id,descriptor_centroid_distance_m,descriptor_extent_ratio,descriptor_point_count_ratio
10.0,10.0,5.0,100,4,0.20,1.10,0.90
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_keyframe_timestamp,candidate_keyframe_timestamp,current_id,candidate_id,accepted,status,fitness_score,correction_translation_m,correction_rotation_rad,descriptor_centroid_distance_m,descriptor_extent_ratio,descriptor_point_count_ratio
10.1,10.1,8.0,101,7,0,loop selection rejected,0.10,0.5,0.05,0.21,1.09,0.91
""",
    )

    signatures = module.read_selected(selected_path)
    status_rows = module.read_status(status_path)
    results = module.evaluate_retrieval(
        signatures,
        status_rows,
        current_timestamp_window_s=1.0,
        candidate_timestamp_window_s=0.25,
        centroid_scale_m=0.6,
        extent_scale=0.15,
        point_ratio_scale=0.10,
    )
    text = module.format_report(
        selected_path,
        status_path,
        results,
        top_k=[1, 3],
        current_timestamp_window_s=1.0,
        candidate_timestamp_window_s=0.25,
        centroid_scale_m=0.6,
        extent_scale=0.15,
        point_ratio_scale=0.10,
        max_rows=10,
        top_examples=3,
    )

    assert "- Endpoint timestamp queries with ranked positive: 0/1" in text
    assert "- Endpoint timestamp recall@1: 0/1 (0.0%)" in text
    assert "| 1 | 10.000 | 4 -> 100 | 1 | miss | miss | 7 -> 101 |" in text


def test_main_writes_descriptor_retrieval_report(tmp_path):
    module = load_module()
    selected_path = tmp_path / "selected.csv"
    status_path = tmp_path / "status.csv"
    out = tmp_path / "retrieval.md"
    write_csv(
        selected_path,
        """
timestamp,current_id,candidate_id,descriptor_centroid_distance_m,descriptor_extent_ratio,descriptor_point_count_ratio
10.0,100,4,0.20,1.10,0.90
""",
    )
    write_csv(
        status_path,
        """
timestamp,current_id,candidate_id,accepted,status,descriptor_centroid_distance_m,descriptor_extent_ratio,descriptor_point_count_ratio
10.1,100,4,1,accepted,0.20,1.10,0.90
""",
    )

    rc = module.main([
        "--selected", str(selected_path),
        "--status", str(status_path),
        "--out", str(out),
        "--current-timestamp-window-s", "1.0",
        "--top-k", "1,5",
    ])

    assert rc == 0
    text = out.read_text(encoding="utf-8")
    assert "# MBES Descriptor Retrieval Evaluation" in text
    assert "- Exact ID recall@1: 1/1 (100.0%)" in text
