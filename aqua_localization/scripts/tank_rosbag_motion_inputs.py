"""ROS bag input adapters for Tank motion-prior diagnostics."""

from __future__ import annotations

import math
from pathlib import Path
import sqlite3

import numpy as np

from tank_dvl_prior_core import DvlRecord, ImuYawRecord, quaternion_to_yaw


DEFAULT_DVL_TOPIC = "/dvl/twist"
DEFAULT_IMU_TOPIC = "/imu/data"


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def resolve_sqlite_db(bag: Path) -> Path:
    if bag.is_file():
        return bag
    candidates = sorted(bag.glob("*.db3"))
    if len(candidates) != 1:
        raise FileNotFoundError(f"expected exactly one sqlite .db3 in {bag}, found {len(candidates)}")
    return candidates[0]


def topic_id_and_type(db_path: Path, topic: str) -> tuple[int, str]:
    with sqlite3.connect(str(db_path)) as con:
        row = con.execute("select id, type from topics where name = ?", (topic,)).fetchone()
    if row is None:
        raise ValueError(f"topic not found in {db_path}: {topic}")
    return int(row[0]), str(row[1])


def read_dvl_records(bag: Path, topic: str) -> list[DvlRecord]:
    from geometry_msgs.msg import TwistStamped
    from rclpy.serialization import deserialize_message

    db_path = resolve_sqlite_db(bag)
    topic_id, msg_type = topic_id_and_type(db_path, topic)
    if msg_type != "geometry_msgs/msg/TwistStamped":
        raise ValueError(f"{topic}: expected geometry_msgs/msg/TwistStamped, got {msg_type}")
    with sqlite3.connect(str(db_path)) as con:
        rows = con.execute(
            "select data from messages where topic_id = ? order by timestamp",
            (topic_id,),
        ).fetchall()
    records = []
    for (raw,) in rows:
        msg = deserialize_message(raw, TwistStamped)
        records.append(DvlRecord(
            stamp_s=stamp_to_seconds(msg.header.stamp),
            velocity_mps=np.asarray(
                [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
                dtype=np.float64,
            ),
        ))
    if not records:
        raise ValueError(f"no DVL records found on {topic}")
    return records


def read_imu_yaw_records(bag: Path, topic: str) -> list[ImuYawRecord]:
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu

    db_path = resolve_sqlite_db(bag)
    topic_id, msg_type = topic_id_and_type(db_path, topic)
    if msg_type != "sensor_msgs/msg/Imu":
        raise ValueError(f"{topic}: expected sensor_msgs/msg/Imu, got {msg_type}")
    with sqlite3.connect(str(db_path)) as con:
        rows = con.execute(
            "select data from messages where topic_id = ? order by timestamp",
            (topic_id,),
        ).fetchall()
    records = []
    for (raw,) in rows:
        msg = deserialize_message(raw, Imu)
        q = msg.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm <= 1.0e-12:
            continue
        records.append(ImuYawRecord(
            stamp_s=stamp_to_seconds(msg.header.stamp),
            yaw_rad=quaternion_to_yaw(q.x / norm, q.y / norm, q.z / norm, q.w / norm),
        ))
    if not records:
        raise ValueError(f"no usable IMU yaw records found on {topic}")
    return records
