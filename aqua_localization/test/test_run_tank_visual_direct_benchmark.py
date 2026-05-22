"""Tests for run_tank_visual_direct_benchmark.py."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "run_tank_visual_direct_benchmark.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_visual_direct_benchmark", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_resolve_sqlite_db_accepts_file_and_single_db_directory(tmp_path):
    module = load_module()
    db = tmp_path / "bag.db3"
    db.write_bytes(b"")

    assert module.resolve_sqlite_db(db) == db
    assert module.resolve_sqlite_db(tmp_path) == db


def test_pair_stereo_records_matches_in_time_order():
    module = load_module()
    left = [
        module.ImageRecord(1.00, b"left0"),
        module.ImageRecord(1.05, b"left1"),
        module.ImageRecord(1.10, b"left2"),
    ]
    right = [
        module.ImageRecord(1.049, b"right1"),
        module.ImageRecord(1.101, b"right2"),
    ]

    pairs = module.pair_stereo_records(left, right, sync_slop_s=0.02)

    assert [(l.data, r.data) for l, r in pairs] == [
        (b"left1", b"right1"),
        (b"left2", b"right2"),
    ]


def test_resolve_record_window_uses_first_left_stamp_for_offset_duration():
    module = load_module()
    args = SimpleNamespace(
        start_offset_s=1.5,
        duration_s=2.0,
        start_stamp_s=None,
        end_stamp_s=None,
    )
    records = [
        module.ImageRecord(10.0, b"left0"),
        module.ImageRecord(11.0, b"left1"),
    ]

    assert module.resolve_record_window(args, records) == (11.5, 13.5)


def test_filter_records_by_window_uses_exclusive_end():
    module = load_module()
    records = [
        module.ImageRecord(10.0, b"before"),
        module.ImageRecord(11.0, b"start"),
        module.ImageRecord(12.0, b"inside"),
        module.ImageRecord(13.0, b"end"),
    ]

    filtered = module.filter_records_by_window(records, start_s=11.0, end_s=13.0)

    assert [record.data for record in filtered] == [b"start", b"inside"]


def test_resolve_record_window_rejects_ambiguous_options():
    module = load_module()
    records = [module.ImageRecord(10.0, b"left")]
    args = SimpleNamespace(
        start_offset_s=1.0,
        duration_s=None,
        start_stamp_s=10.5,
        end_stamp_s=None,
    )

    with pytest.raises(ValueError, match="start-offset-s"):
        module.resolve_record_window(args, records)


def test_format_tum_pose_line_uses_position_and_quaternion():
    module = load_module()
    pose = np.eye(4)
    pose[:3, 3] = [1.0, 2.0, 3.0]

    line = module.format_tum_pose_line(12.5, pose)

    assert line == "12.500000000 1.000000000 2.000000000 3.000000000 0.000000000 0.000000000 0.000000000 1.000000000\n"


def test_parse_args_defaults_to_direct_system():
    module = load_module()

    args = module.parse_args([
        "--bag",
        "/tmp/bag",
        "--reference",
        "/tmp/ref.tum",
    ])

    assert args.system == "aqua_visual_frontend_direct"
    assert args.left_topic == module.DEFAULT_LEFT_TOPIC
    assert args.right_topic == module.DEFAULT_RIGHT_TOPIC
    assert args.sync_slop_s == 0.02
    assert args.start_offset_s is None
    assert args.duration_s is None
    assert args.start_stamp_s is None
    assert args.end_stamp_s is None
    assert args.min_pnp_inliers == 12
    assert args.min_inlier_ratio == 0.25
    assert args.ransac_iterations == 100
    assert args.ransac_reprojection_error_px == 3.0
    assert args.ransac_confidence == 0.99
    assert args.max_step_translation_m == 2.0
