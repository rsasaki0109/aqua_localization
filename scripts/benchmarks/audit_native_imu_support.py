"""Read a native ROS1 bag without reference access and audit input support."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile

from openvins_input_support import Observation, audit_support

IMU_TOPIC = "/rtimulib_node/imu"
CAMERA_TOPIC = "/camera/image_raw"
PRESSURE_TOPIC = "/barometer_node/pressure"


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_new_json(path, data):
    """Atomic exclusive publication: never replace an earlier result/registration."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        stream.write((json.dumps(data, sort_keys=True, indent=2) + "\n").encode())
        stream.flush()
        os.fsync(stream.fileno())
    try:
        os.link(temporary, path)
    finally:
        temporary.unlink()


def scan_native_bag(path):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    store = get_typestore(Stores.ROS1_NOETIC)
    topic_counts, ordinals, imu, observations = {}, {}, [], []
    with Reader(Path(path)) as reader:
        for connection in reader.connections:
            if connection.msgtype not in store.types:
                store.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
        for connection, bag_stamp, raw in reader.messages():
            topic = connection.topic
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            # Decode every ROS message, including the unselected sensor topics.
            msg = store.deserialize_ros1(raw, connection.msgtype)
            if topic not in {IMU_TOPIC, CAMERA_TOPIC, PRESSURE_TOPIC}:
                continue
            stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            if topic == IMU_TOPIC:
                imu.append(stamp)
            else:
                sensor = "camera" if topic == CAMERA_TOPIC else "pressure"
                ordinal = ordinals.get(sensor, 0)
                observations.append(Observation(sensor, ordinal, stamp,
                                                hashlib.sha256(raw).hexdigest(), int(bag_stamp)))
                ordinals[sensor] = ordinal + 1
    return topic_counts, imu, observations


def audit_native_bag(path):
    counts, imu, observations = scan_native_bag(path)
    result = audit_support(imu, observations)
    result.update(native_bag_bytes=Path(path).stat().st_size,
                  native_bag_sha256=sha256(path),
                  all_ros_message_count=sum(counts.values()), topic_counts=counts,
                  estimator_stream_conversion="not_executed",
                  reference_access="not_performed")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = audit_native_bag(args.bag)
    write_new_json(args.out, result)
    print(json.dumps({k: result[k] for k in ("policy_version", "all_ros_message_count",
                                           "native_observation_counts", "excluded_observation_counts")}))


if __name__ == "__main__":
    main()
