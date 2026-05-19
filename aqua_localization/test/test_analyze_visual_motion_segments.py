"""Tests for analyze_visual_motion_segments.py."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "analyze_visual_motion_segments.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("analyze_visual_motion_segments", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{value:.9f}" for value in row) + "\n")


def make_rows(scale=1.0, n=80):
    rows = []
    for i in range(n):
        t = float(i) * 0.1
        x = 0.04 * float(i)
        y = 0.1 * math.sin(0.1 * float(i))
        z = 0.02 * math.cos(0.05 * float(i))
        rows.append([t, x * scale, y * scale, z * scale, 0.0, 0.0, 0.0, 1.0])
    return rows


def test_build_segments_recovers_known_length_ratio(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0))
    times, est_xyz, ref_xyz = module.load_matched_positions(ref, est)

    segments = module.build_segments(
        times, est_xyz, ref_xyz, segment_s=1.0, stride_s=0.5, min_reference_motion_m=0.01)

    assert segments
    assert segments[0].visual_to_reference_ratio == pytest.approx(4.0)
    assert segments[0].correction_scale == pytest.approx(0.25)
    assert segments[0].reference_speed_mps > 0.0


def test_heading_bucket_cardinal_labels():
    module = load_module()

    assert module.heading_bucket(0.0) == "E"
    assert module.heading_bucket(45.0) == "NE"
    assert module.heading_bucket(90.0) == "N"
    assert module.heading_bucket(float("nan")) == "unknown"


def test_format_markdown_contains_summary_and_segment_rows(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0))
    args = module.parse_args([
        str(ref),
        str(est),
        "--segment-s", "1.0",
        "--stride-s", "0.5",
        "--min-reference-motion-m", "0.01",
    ])

    text = module.run_analysis(args)

    assert "# Visual Motion Segment Analysis" in text
    assert "visual/reference length ratio" in text
    assert "reference/visual correction scale" in text
    assert "| Start s | End s | Samples |" in text
    assert "Median correction scale" in text


def test_main_writes_output_file(tmp_path, capsys):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    out = tmp_path / "segments.md"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0))

    rc = module.main([
        str(ref),
        str(est),
        "--segment-s", "1.0",
        "--stride-s", "0.5",
        "--min-reference-motion-m", "0.01",
        "--out", str(out),
    ])

    assert rc == 0
    assert out.exists()
    assert "Visual Motion Segment Analysis" in out.read_text(encoding="utf-8")
    assert "Visual Motion Segment Analysis" in capsys.readouterr().out
