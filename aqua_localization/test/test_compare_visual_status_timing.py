"""Tests for compare_visual_status_timing.py."""

import csv
import importlib.util
from pathlib import Path
import sys

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "compare_visual_status_timing.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("compare_visual_status_timing", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


FIELDS = [
    "timestamp",
    "right_timestamp",
    "stereo_sync_delta_ms",
    "frame_index",
    "accepted",
    "status",
    "left_features",
    "right_features",
    "stereo_matches",
    "stereo_points",
    "temporal_matches",
    "pnp_inliers",
    "step_translation_m",
]


def row(**kwargs):
    data = {
        "timestamp": "10.0",
        "right_timestamp": "10.001",
        "stereo_sync_delta_ms": "1.0",
        "frame_index": "1",
        "accepted": "1",
        "status": "accepted",
        "left_features": "100",
        "right_features": "90",
        "stereo_matches": "80",
        "stereo_points": "70",
        "temporal_matches": "60",
        "pnp_inliers": "50",
        "step_translation_m": "0.10",
    }
    data.update({key: str(value) for key, value in kwargs.items()})
    return data


def write_status(path: Path, rows):
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def test_compare_status_timing_matches_nearest_timestamp_and_reports_deltas():
    module = load_module()
    baseline = [
        module.StatusSample(10.00, 10.001, 1.0, 1, True, "accepted", 100, 90, 80, 70, 0, 0, 0.0),
        module.StatusSample(10.05, 10.052, 2.0, 2, True, "accepted", 110, 95, 85, 75, 60, 50, 0.1),
        module.StatusSample(10.10, 10.101, 1.0, 3, False, "pnp failed", 120, 96, 86, 76, 61, 20, 0.0),
    ]
    target = [
        module.StatusSample(10.051, 10.0525, 1.5, 2, True, "accepted", 112, 95, 85, 78, 64, 51, 0.11),
        module.StatusSample(10.101, 10.1015, 0.5, 3, True, "accepted", 121, 96, 86, 80, 63, 45, 0.12),
    ]

    summary, rows = module.compare_status_timing(baseline, target, timestamp_slop_s=0.0005)

    assert summary.matched_samples == 2
    assert summary.unmatched_baseline_samples == 1
    assert summary.over_slop_samples == 2
    assert summary.acceptance_mismatches == 1
    assert summary.status_mismatches == 1
    assert summary.median_abs_timestamp_delta_ms == pytest.approx(1.0)
    assert summary.median_abs_sync_delta_diff_ms == pytest.approx(0.5)
    assert rows[0]["baseline_frame_index"] == 2
    assert rows[0]["timestamp_delta_ms"] == pytest.approx(1.0)
    assert rows[0]["stereo_points_delta"] == 3


def test_read_status_csv_accepts_missing_new_sync_columns(tmp_path):
    module = load_module()
    path = tmp_path / "old_status.csv"
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=[field for field in FIELDS if field not in (
            "right_timestamp",
            "stereo_sync_delta_ms",
        )])
        writer.writeheader()
        old_row = row()
        old_row.pop("right_timestamp")
        old_row.pop("stereo_sync_delta_ms")
        writer.writerow(old_row)

    samples = module.read_status_csv(path)

    assert len(samples) == 1
    assert samples[0].timestamp == pytest.approx(10.0)
    assert samples[0].right_timestamp != samples[0].right_timestamp
    assert samples[0].stereo_sync_delta_ms != samples[0].stereo_sync_delta_ms


def test_write_correspondence_csv(tmp_path):
    module = load_module()
    out = tmp_path / "timing.csv"
    module.write_correspondence_csv(out, [{
        "target_frame_index": 1,
        "baseline_frame_index": 1,
        "frame_index_delta": 0,
        "target_timestamp": 10.0,
        "baseline_timestamp": 10.0,
        "target_offset_s": 0.0,
        "timestamp_delta_ms": 0.0,
        "target_right_timestamp": 10.001,
        "baseline_right_timestamp": 10.001,
        "target_sync_delta_ms": 1.0,
        "baseline_sync_delta_ms": 1.0,
        "sync_delta_diff_ms": 0.0,
        "target_accepted": True,
        "baseline_accepted": True,
        "accepted_match": True,
        "target_status": "accepted",
        "baseline_status": "accepted",
        "status_match": True,
        "target_left_features": 10,
        "baseline_left_features": 10,
        "left_features_delta": 0,
        "target_stereo_points": 9,
        "baseline_stereo_points": 9,
        "stereo_points_delta": 0,
        "target_temporal_matches": 8,
        "baseline_temporal_matches": 8,
        "temporal_matches_delta": 0,
        "target_pnp_inliers": 7,
        "baseline_pnp_inliers": 7,
        "pnp_inliers_delta": 0,
        "target_step_translation_m": 0.1,
        "baseline_step_translation_m": 0.1,
        "step_translation_delta_m": 0.0,
    }])

    text = out.read_text(encoding="utf-8")
    assert text.startswith("target_frame_index,baseline_frame_index")
    assert "accepted" in text
