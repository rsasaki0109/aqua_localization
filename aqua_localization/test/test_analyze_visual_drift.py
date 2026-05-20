"""Tests for analyze_visual_drift.py."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "analyze_visual_drift.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("analyze_visual_drift", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_tum(path: Path, rows):
    with path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(" ".join(f"{value:.9f}" for value in row) + "\n")


def make_rows(scale=1.0, n=80, drift=0.0):
    rows = []
    for i in range(n):
        t = float(i) * 0.1
        x = 0.05 * float(i)
        y = 0.2 * math.sin(0.1 * float(i))
        z = 0.02 * math.cos(0.07 * float(i))
        local_scale = scale + drift * float(i) / max(1.0, float(n - 1))
        rows.append([t, x * local_scale, y * local_scale, z * local_scale, 0.0, 0.0, 0.0, 1.0])
    return rows


def test_sliding_windows_reports_scale_per_window(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0))
    times, est_xyz, ref_xyz, compare = module.matched_positions(ref, est)

    windows = module.sliding_windows(
        times, est_xyz, ref_xyz, window_s=2.0, stride_s=1.0, min_samples=10, compare=compare)

    assert windows
    assert windows[0].samples >= 10
    assert windows[0].sim3_scale == pytest.approx(0.25, abs=1.0e-9)
    assert windows[0].sim3_rmse_m == pytest.approx(0.0, abs=1.0e-9)


def test_scale_stats_reports_relative_std():
    module = load_module()
    windows = [
        module.WindowDrift(0.0, 1.0, 10, 1.0, 0.1, 0.2, 1.0, 0.1),
        module.WindowDrift(1.0, 2.0, 10, 1.0, 0.1, 0.3, 1.0, 0.1),
    ]

    stats = module.scale_stats(windows)

    assert stats["mean"] == pytest.approx(0.25)
    assert stats["relative_std"] == pytest.approx(0.2)


def test_format_markdown_contains_window_table(tmp_path):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0, drift=0.5))
    args = module.parse_args([
        str(ref),
        str(est),
        "--window-s", "2.0",
        "--stride-s", "1.0",
        "--min-samples", "10",
    ])

    text = module.run_analysis(args)

    assert "# Visual Drift Analysis" in text
    assert "Overall Sim(3) scale" in text
    assert "| Start s | End s | Samples |" in text
    assert "## Interpretation" in text


def test_main_writes_output_file(tmp_path, capsys):
    module = load_module()
    ref = tmp_path / "ref.tum"
    est = tmp_path / "est.tum"
    out = tmp_path / "drift.md"
    write_tum(ref, make_rows(scale=1.0))
    write_tum(est, make_rows(scale=4.0))

    rc = module.main([
        str(ref),
        str(est),
        "--window-s", "2.0",
        "--stride-s", "1.0",
        "--min-samples", "10",
        "--out", str(out),
    ])

    assert rc == 0
    assert out.exists()
    assert "Visual Drift Analysis" in out.read_text(encoding="utf-8")
    assert "Visual Drift Analysis" in capsys.readouterr().out
