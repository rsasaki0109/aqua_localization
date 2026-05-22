"""Dependency-localization tests for tank_rosbag_motion_inputs.py."""

import importlib.util
import sqlite3
import sys
from pathlib import Path

import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "tank_rosbag_motion_inputs.py"
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("tank_rosbag_motion_inputs", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_resolve_sqlite_db_accepts_direct_file(tmp_path):
    module = load_module()
    db = tmp_path / "bag.db3"
    db.write_bytes(b"")

    assert module.resolve_sqlite_db(db) == db


def test_resolve_sqlite_db_requires_single_db3_in_directory(tmp_path):
    module = load_module()

    with pytest.raises(FileNotFoundError, match="expected exactly one"):
        module.resolve_sqlite_db(tmp_path)

    one = tmp_path / "one.db3"
    one.write_bytes(b"")
    assert module.resolve_sqlite_db(tmp_path) == one

    (tmp_path / "two.db3").write_bytes(b"")
    with pytest.raises(FileNotFoundError, match="expected exactly one"):
        module.resolve_sqlite_db(tmp_path)


def test_topic_id_and_type_reports_missing_topic(tmp_path):
    module = load_module()
    db = tmp_path / "bag.db3"
    with sqlite3.connect(str(db)) as con:
        con.execute("create table topics (id integer, name text, type text)")

    with pytest.raises(ValueError, match="topic not found"):
        module.topic_id_and_type(db, "/missing")


def test_topic_id_and_type_reads_existing_topic(tmp_path):
    module = load_module()
    db = tmp_path / "bag.db3"
    with sqlite3.connect(str(db)) as con:
        con.execute("create table topics (id integer, name text, type text)")
        con.execute("insert into topics values (7, '/imu/data', 'sensor_msgs/msg/Imu')")

    assert module.topic_id_and_type(db, "/imu/data") == (7, "sensor_msgs/msg/Imu")


def test_stamp_to_seconds_uses_sec_and_nanosec():
    module = load_module()

    class Stamp:
        sec = 3
        nanosec = 250_000_000

    assert module.stamp_to_seconds(Stamp()) == 3.25
