"""Tests for acquire_mbes_beach_pond.py command construction."""

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "acquire_mbes_beach_pond.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("acquire_mbes_beach_pond", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_dry_run_prints_resumable_download_and_conversion(tmp_path, capsys):
    module = load_module()

    archive, result = module.acquire(tmp_path, dry_run=True)
    output = capsys.readouterr().out

    assert archive == tmp_path / "beach_pond.tar.gz"
    assert result == tmp_path / "beach_pond_ros2"
    assert "--continue" in output
    assert module.DEFAULT_URL in output
    assert "tar xzf" in output
    assert "rosbags-convert" in output
    assert "/norbit/detections" in output
    assert "/nav/processed/odometry" in output


def test_no_convert_stops_after_extract_plan(tmp_path, capsys):
    module = load_module()

    _, result = module.acquire(tmp_path, convert=False, dry_run=True)
    output = capsys.readouterr().out

    assert result is None
    assert "tar xzf" in output
    assert "rosbags-convert" not in output

