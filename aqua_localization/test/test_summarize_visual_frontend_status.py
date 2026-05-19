"""Tests for visual frontend status CSV summary helpers."""

import csv
import importlib.util
import math
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "summarize_visual_frontend_status.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("summarize_visual_frontend_status", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_csv(path: Path, rows: list[dict]):
    with path.open("w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=[
            "timestamp",
            "frame_index",
            "accepted_count",
            "rejected_count",
            "left_features",
            "right_features",
            "stereo_matches",
            "stereo_points",
            "disparity_min_px",
            "disparity_median_px",
            "disparity_p95_px",
            "depth_min_m",
            "depth_median_m",
            "depth_p95_m",
            "temporal_matches",
            "pnp_inliers",
            "inlier_ratio",
            "step_translation_m",
            "accepted",
            "status",
        ])
        writer.writeheader()
        writer.writerows(rows)


def sample_row(**kwargs):
    row = {
        "timestamp": "1.0",
        "frame_index": "1",
        "accepted_count": "1",
        "rejected_count": "0",
        "left_features": "100",
        "right_features": "95",
        "stereo_matches": "80",
        "stereo_points": "70",
        "disparity_min_px": "2.0",
        "disparity_median_px": "6.0",
        "disparity_p95_px": "18.0",
        "depth_min_m": "1.0",
        "depth_median_m": "4.0",
        "depth_p95_m": "8.0",
        "temporal_matches": "60",
        "pnp_inliers": "50",
        "inlier_ratio": "0.8",
        "step_translation_m": "0.05",
        "accepted": "1",
        "status": "accepted",
    }
    row.update({key: str(value) for key, value in kwargs.items()})
    return row


def test_read_csv_parses_status_rows(tmp_path):
    module = load_module()
    path = tmp_path / "status.csv"
    write_csv(path, [sample_row(timestamp="2.5", accepted="0", status="pnp failed")])

    samples = module.read_csv(path)

    assert len(samples) == 1
    assert samples[0].timestamp == 2.5
    assert samples[0].accepted is False
    assert samples[0].status == "pnp failed"


def test_summarize_counts_acceptance_and_quantiles():
    module = load_module()
    samples = [
        module.sample_from_row(sample_row(timestamp="1.0", stereo_points="10", accepted="1")),
        module.sample_from_row(sample_row(
            timestamp="2.0",
            stereo_points="20",
            temporal_matches="10",
            inlier_ratio="0.25",
            accepted="0",
            status="too few pnp inliers",
        )),
        module.sample_from_row(sample_row(timestamp="3.0", stereo_points="30", accepted="1")),
    ]

    summary = module.summarize(samples)

    assert summary["total"] == 3
    assert summary["accepted"] == 2
    assert summary["rejected"] == 1
    assert math.isclose(summary["acceptance_ratio"], 2.0 / 3.0)
    assert summary["duration_s"] == 2.0
    assert summary["rejection_counts"]["too few pnp inliers"] == 1
    assert summary["numeric"]["stereo_points"]["median"] == 20
    assert summary["numeric"]["disparity_median_px"]["median"] == 6.0


def test_format_summary_contains_tuning_hints_for_weak_tracking():
    module = load_module()
    samples = [
        module.sample_from_row(sample_row(
            timestamp=i,
            stereo_points="30",
            temporal_matches="15",
            inlier_ratio="0.2",
            step_translation_m="0.4",
            disparity_median_px="2.0",
            depth_p95_m="12.0",
            accepted="0",
            status="too few pnp inliers",
        ))
        for i in range(5)
    ]
    summary = module.summarize(samples)

    text = module.format_summary_markdown(summary, "/tmp/status.csv")

    assert "# Visual Frontend Status Summary" in text
    assert "Accepted: 0 (0.0%)" in text
    assert "| stereo_points |" in text
    assert "Low stereo point count" in text
    assert "Median disparity is low" in text
    assert "Depth tail is near the maximum range" in text
    assert "Low temporal match count" in text
    assert "Low PnP inlier ratio" in text
    assert "Large step-translation tail" in text
    assert "`too few pnp inliers`" in text


def test_tuning_hints_for_healthy_tracking_focuses_on_calibration():
    module = load_module()
    samples = [
        module.sample_from_row(sample_row(
            timestamp=i,
            stereo_points="300",
            temporal_matches="180",
            inlier_ratio="0.8",
            step_translation_m="0.03",
            accepted="1",
            status="accepted",
        ))
        for i in range(5)
    ]

    hints = module.tuning_hints(module.summarize(samples))

    assert hints == ["No rejected frames were observed; focus next on scale, extrinsics, and drift."]


def test_main_writes_summary_file(tmp_path, capsys):
    module = load_module()
    csv_path = tmp_path / "status.csv"
    out = tmp_path / "summary.md"
    write_csv(csv_path, [sample_row()])

    rc = module.main([str(csv_path), "--summary-out", str(out)])

    assert rc == 0
    assert out.exists()
    assert "Visual Frontend Status Summary" in out.read_text(encoding="utf-8")
    assert "Visual Frontend Status Summary" in capsys.readouterr().out
