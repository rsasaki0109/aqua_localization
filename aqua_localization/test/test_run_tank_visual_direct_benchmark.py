"""Tests for run_tank_visual_direct_benchmark.py."""

import importlib.util
import sys
from pathlib import Path

import numpy as np


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
