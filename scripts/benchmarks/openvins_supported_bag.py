"""New common ROS1 input contract; not the missing legacy OpenVINS runner.

Keep the original bag unchanged and record every included/excluded observation.
Both arms consume this same bag. The baseline ignores its pressure topic.
"""
import argparse
import json
from pathlib import Path

from audit_native_imu_support import (CAMERA_TOPIC, IMU_TOPIC, PRESSURE_TOPIC,
                                     audit_native_bag, sha256, write_new_json)
from openvins_input_support import POLICY_VERSION


def convert_supported_bag(native, output, contract=None):
    from rosbags.rosbag1 import Reader, Writer
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    native, output = Path(native), Path(output)
    audit = audit_native_bag(native)
    if contract is not None:
        if (contract["camera_offset_ns"] != audit["camera_offset_ns"] or
            contract["pressure_offset_ns"] != 0 or contract["imu_extrapolation"] is not False or
            contract["shared_camera_mask"] is not True):
            raise ValueError("registered clock/support conditions differ from converter")
        if {"imu": audit["native_imu_count"], **audit["native_observation_counts"]} != contract["expected_native_counts"]:
            raise ValueError("native sensor counts differ from registered contract")
        keys = ("sensor", "native_ordinal", "exclusion_reason", "distance_outside_support_ns")
        excluded = [{key: r[key] for key in keys} for r in audit["records"] if not r["included"]]
        if excluded != contract["expected_exclusions"]:
            raise ValueError("boundary differs from registration; preserve input and register a new version")
    output.mkdir(parents=True, exist_ok=False)
    # Publish the complete exclusion ledger before processing any derived data.
    write_new_json(output / "input_audit.json", audit)
    store = get_typestore(Stores.ROS1_NOETIC)
    decisions = {(r["sensor"], r["native_ordinal"]): r for r in audit["records"]}
    ordinals, counts, selected = {}, {}, {IMU_TOPIC, CAMERA_TOPIC, PRESSURE_TOPIC}
    bag = output / "supported.bag"
    try:
        with Reader(native) as reader, Writer(bag) as writer:
            connections = {}
            for connection in reader.connections:
                if connection.topic not in selected:
                    continue
                if connection.msgtype not in store.types:
                    store.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
                connections[connection.id] = writer.add_connection(
                    connection.topic, connection.msgtype, msgdef=connection.msgdef.data,
                    md5sum=connection.digest)
            for connection, _, raw in reader.messages():
                topic = connection.topic
                if topic not in selected:
                    continue
                msg = store.deserialize_ros1(raw, connection.msgtype)
                stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
                if topic != IMU_TOPIC:
                    sensor = "camera" if topic == CAMERA_TOPIC else "pressure"
                    ordinal = ordinals.get(sensor, 0)
                    ordinals[sensor] = ordinal + 1
                    decision = decisions[(sensor, ordinal)]
                    if not decision["included"]:
                        continue
                    stamp = decision["corrected_stamp_ns"]
                    if sensor == "camera":
                        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                        raw = store.serialize_ros1(msg, connection.msgtype)
                writer.write(connections[connection.id], stamp, raw)
                counts[topic] = counts.get(topic, 0) + 1
        # Independent reread verifies writer output, corrected headers, and ordering.
        derived_counts = {}
        with Reader(bag) as reader:
            last = None
            for connection, stamp, raw in reader.messages():
                msg = store.deserialize_ros1(raw, connection.msgtype)
                header_stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
                if header_stamp != stamp or (last is not None and stamp < last):
                    raise ValueError("derived bag header/ordering verification failed")
                if not audit["native_imu_support_ns"][0] <= stamp <= audit["native_imu_support_ns"][1]:
                    raise ValueError("derived observation outside native IMU support")
                derived_counts[connection.topic] = derived_counts.get(connection.topic, 0) + 1
                last = stamp
        expected = {IMU_TOPIC: audit["native_imu_count"],
                    CAMERA_TOPIC: audit["eligible_observation_counts"].get("camera", 0),
                    PRESSURE_TOPIC: audit["eligible_observation_counts"].get("pressure", 0)}
        if counts != {t: n for t, n in expected.items() if n} or derived_counts != counts:
            raise ValueError("derived sensor counts differ from registered support mask")
        if sha256(native) != audit["native_bag_sha256"]:
            raise ValueError("native input changed during conversion")
        result = {"schema": "openvins-supported-ros1-bag-v2", "status": "converted",
                  "policy_version": POLICY_VERSION,
                  "native_bag_sha256": audit["native_bag_sha256"],
                  "supported_bag_sha256": sha256(bag), "supported_topic_counts": counts,
                  "native_topic_counts": audit["topic_counts"],
                  "excluded_observation_counts": audit["excluded_observation_counts"],
                  "camera_offset_ns": audit["camera_offset_ns"],
                  "derived_header_and_bag_clock": "corrected native sensor header nanoseconds",
                  "camera_mask_shared_by_both_arms": True,
                  "estimator_execution": "not_executed", "reference_access": "not_performed"}
        write_new_json(output / "conversion.json", result)
        return result
    except BaseException as error:
        # Preserve the audit, partial derived bag and failure, with no success marker.
        write_new_json(output / "failure.json", {"status": "failed", "type": type(error).__name__,
                       "error": str(error), "reference_access": "not_performed"})
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--plan", type=Path, required=True)
    args = parser.parse_args()
    from prepare_vio_comparison import SOURCE_FILES
    plan = json.loads(args.plan.read_text())
    registration = json.loads((args.plan.parent / "registration.json").read_text())
    if (sha256(args.plan) != registration["plan_sha256"] or
        sha256(args.plan.parent / "scoring-plan.json") != registration["scoring_plan_sha256"]):
        raise ValueError("registration hashes changed; preserve it and register a new version")
    if (plan.get("input_contract", {}).get("policy_version") != POLICY_VERSION or
        set(plan["source_sha256"]) != set(SOURCE_FILES)):
        raise ValueError("a new input-contract registration is required before conversion")
    if sha256(args.bag) != plan["native_bag_sha256"]:
        raise ValueError("native bag differs from registered input")
    for relative, digest in plan["source_sha256"].items():
        if sha256(Path(__file__).resolve().parents[2] / relative) != digest:
            raise ValueError(f"registered source changed: {relative}; register a new version")
    print(json.dumps(convert_supported_bag(args.bag, args.out, plan["input_contract"]), sort_keys=True))
