"""Independent byte/content check of the new supported ROS1 input format.

This is not a recovered verifier for the unavailable old native stream format.
No reference trajectory is accepted or accessed.
"""
import argparse
import hashlib
import json
from pathlib import Path

from audit_native_imu_support import CAMERA_TOPIC, IMU_TOPIC, PRESSURE_TOPIC, sha256, write_new_json


def verify(native, derived, offset_ns=-40380655):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS1_NOETIC)
    topics = {IMU_TOPIC, CAMERA_TOPIC, PRESSURE_TOPIC}
    expected = {topic: [] for topic in topics}
    ordinals, excluded = {topic: 0 for topic in topics}, []
    # First determine the native IMU domain independently of the converter audit.
    with Reader(Path(native)) as reader:
        imu_connections = [c for c in reader.connections if c.topic == IMU_TOPIC]
        stamps = []
        for c, _, raw in reader.messages(connections=imu_connections):
            msg = store.deserialize_ros1(raw, c.msgtype)
            stamps.append(int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec))
    if len(stamps) < 2 or any(b <= a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("native IMU domain is not strictly ordered")
    first, last = stamps[0], stamps[-1]
    with Reader(Path(native)) as reader:
        for c, _, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            msg = store.deserialize_ros1(raw, c.msgtype)
            stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            corrected = stamp + (offset_ns if c.topic == CAMERA_TOPIC else 0)
            if first <= corrected <= last:
                expected[c.topic].append((corrected, hashlib.sha256(raw).hexdigest()))
            else:
                excluded.append({"topic": c.topic, "native_ordinal": ordinals[c.topic],
                                 "native_stamp_ns": stamp, "corrected_stamp_ns": corrected})
            ordinals[c.topic] += 1
    seen = {topic: 0 for topic in topics}
    previous = None
    with Reader(Path(derived)) as reader:
        if any(c.topic not in topics for c in reader.connections):
            raise ValueError("unexpected topic in supported bag")
        for c, bag_stamp, raw in reader.messages():
            msg = store.deserialize_ros1(raw, c.msgtype)
            stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if bag_stamp != stamp or not first <= stamp <= last or (previous is not None and stamp < previous):
                raise ValueError("derived clock/order/support mismatch")
            if c.topic == CAMERA_TOPIC:
                # Restore only the header clock; every other serialized byte must
                # match the exact eligible native camera message.
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp - offset_ns, 1_000_000_000)
                raw = store.serialize_ros1(msg, c.msgtype)
            index = seen[c.topic]
            if index >= len(expected[c.topic]) or expected[c.topic][index] != (stamp, hashlib.sha256(raw).hexdigest()):
                raise ValueError("derived content/mask differs from native sensor data")
            seen[c.topic] += 1
            previous = stamp
    if any(seen[t] != len(expected[t]) for t in topics):
        raise ValueError("derived bag omitted eligible sensor data")
    return {"schema": "independent-supported-bag-verification-v2", "status": "verified",
            "native_bag_sha256": sha256(native), "supported_bag_sha256": sha256(derived),
            "native_imu_support_ns": [first, last], "camera_offset_ns": offset_ns,
            "verified_topic_counts": seen, "independently_excluded": excluded,
            "native_imu_and_pressure_raw_bytes_match": True,
            "eligible_camera_bytes_match_after_restoring_header_clock": True,
            "estimator_execution": "not_executed", "reference_access": "not_performed"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--native", type=Path, required=True)
    parser.add_argument("--derived", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = verify(args.native, args.derived)
    write_new_json(args.out, result)
    print(json.dumps(result, sort_keys=True))
