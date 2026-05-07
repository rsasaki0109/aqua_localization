#!/usr/bin/env python3
"""Convert a Tank Dataset ROS 1 bag (Xu et al. 2025, IJRR) to a ROS 2 mcap.

The original bag carries `/dvl/data` as the custom
`waterlinked_a50_ros_driver/DVL` message, which `rosbags-convert` cannot
preserve without the upstream package installed. This script:

1. Reads the ROS 1 bag through the `rosbags` Python API (no ROS 1 install
   needed).
2. Decodes the WaterLinked DVL message inline using the definition stored
   in the bag itself.
3. Re-publishes the body-frame `velocity` field as `/dvl/twist`
   (`geometry_msgs/TwistStamped`) — the topic + type that
   `aqua_imu_loc` already accepts via `topics.dvl_velocity`.
4. Passes every other topic through unchanged.
5. Writes a single ROS 2 mcap bag at the destination path.

By default the camera topics are dropped because they are large and not used
by `aqua_imu_loc`; pass `--include-cameras` to keep them.

Typical usage:

    pip install --user "rosbags>=0.10"
    ros2 run aqua_localization convert_tank_dataset_bag.py \
        --src aqua_localization/datasets/public/tank_dataset/short_test.bag \
        --dst aqua_localization/datasets/public/tank_dataset/short_test_ros2

The destination is created if missing; the mcap file lives inside it.
"""

import argparse
import sys
from pathlib import Path

from rosbags.rosbag1 import Reader as Ros1Reader
from rosbags.rosbag2 import StoragePlugin
from rosbags.rosbag2 import Writer as Ros2Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

# Topic + type the destination bag will publish for the DVL track.
DVL_OUT_TOPIC = "/dvl/twist"
DVL_OUT_TYPE = "geometry_msgs/msg/TwistStamped"
# Synthetic FluidPressure derived from the source bag's depth Odometry.
PRESSURE_OUT_TOPIC = "/pressure"
PRESSURE_OUT_TYPE = "sensor_msgs/msg/FluidPressure"
# Source bag uses positive-up z, so depth_m = -z. Convert to fluid pressure
# in pascals using freshwater density and standard gravity. aqua_imu_loc
# `pressure.use_first_pressure_as_reference: true` will zero the offset, so
# the absolute reference does not matter, but we still emit a physically
# plausible value so downstream tools see meaningful units.
WATER_DENSITY_KGM3 = 1000.0
GRAVITY_MPS2 = 9.80665
ATMOSPHERIC_PA = 101325.0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Convert a Tank Dataset ROS 1 bag to a ROS 2 mcap, decoding the "
            "WaterLinked DVL message into geometry_msgs/TwistStamped."
        )
    )
    parser.add_argument("--src", required=True, type=Path)
    parser.add_argument("--dst", required=True, type=Path)
    parser.add_argument(
        "--include-cameras", action="store_true",
        help="Keep /camera/* image topics in the output (large).",
    )
    parser.add_argument(
        "--dvl-frame", default="dvl_link",
        help="frame_id stamped on the emitted /dvl/twist messages.",
    )
    return parser.parse_args(argv)


def topic_should_pass_through(topic: str, include_cameras: bool) -> bool:
    if not include_cameras and topic.startswith("/camera/"):
        return False
    return True


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if not args.src.exists():
        print(f"source bag not found: {args.src}", file=sys.stderr)
        return 1
    if args.dst.exists():
        print(f"destination already exists: {args.dst}", file=sys.stderr)
        return 1

    ros1_typestore = get_typestore(Stores.ROS1_NOETIC)
    ros2_typestore = get_typestore(Stores.LATEST)

    converted = 0
    passthrough = 0
    dropped_dvl_invalid = 0
    skipped_camera = 0

    with Ros1Reader(args.src) as reader, Ros2Writer(
        args.dst,
        version=Ros2Writer.VERSION_LATEST,
        storage_plugin=StoragePlugin.MCAP,
    ) as writer:
        # Register every connection's message definition into both typestores
        # (the source typestore needs the custom DVL definition to deserialize
        # input; the destination typestore sees only stock TwistStamped + the
        # other passthrough types it already knows).
        for connection in reader.connections:
            if connection.msgtype not in ros1_typestore.types and connection.msgdef:
                ros1_typestore.register(
                    get_types_from_msg(connection.msgdef.data, connection.msgtype)
                )

        # Allocate destination connections up front.
        out_conns: dict[str, object] = {}
        for connection in reader.connections:
            if connection.topic == "/dvl/data":
                if DVL_OUT_TOPIC not in out_conns:
                    out_conns[DVL_OUT_TOPIC] = writer.add_connection(
                        DVL_OUT_TOPIC, DVL_OUT_TYPE, typestore=ros2_typestore
                    )
                continue
            if connection.topic == "/depth/data":
                # Pass /depth/data through (it stays nav_msgs/Odometry) AND emit
                # a derived /pressure FluidPressure track so aqua_imu_loc can
                # consume the depth without a live adapter.
                if connection.topic not in out_conns:
                    out_conns[connection.topic] = writer.add_connection(
                        connection.topic, connection.msgtype, typestore=ros2_typestore
                    )
                if PRESSURE_OUT_TOPIC not in out_conns:
                    out_conns[PRESSURE_OUT_TOPIC] = writer.add_connection(
                        PRESSURE_OUT_TOPIC, PRESSURE_OUT_TYPE, typestore=ros2_typestore
                    )
                continue
            if not topic_should_pass_through(connection.topic, args.include_cameras):
                continue
            if connection.topic in out_conns:
                continue
            out_conns[connection.topic] = writer.add_connection(
                connection.topic, connection.msgtype, typestore=ros2_typestore
            )

        # Stream messages.
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == "/dvl/data":
                msg = ros1_typestore.deserialize_ros1(rawdata, connection.msgtype)
                # Drop samples where the driver flags the velocity as invalid.
                if hasattr(msg, "velocity_valid") and not msg.velocity_valid:
                    dropped_dvl_invalid += 1
                    continue
                # Build a TwistStamped using the destination typestore.
                twist_type = ros2_typestore.types["geometry_msgs/msg/TwistStamped"]
                vector3_type = ros2_typestore.types["geometry_msgs/msg/Vector3"]
                twist3_type = ros2_typestore.types["geometry_msgs/msg/Twist"]
                header_type = ros2_typestore.types["std_msgs/msg/Header"]
                time_type = ros2_typestore.types["builtin_interfaces/msg/Time"]
                stamp = time_type(sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec)
                header = header_type(stamp=stamp, frame_id=args.dvl_frame)
                linear = vector3_type(
                    x=float(msg.velocity.x),
                    y=float(msg.velocity.y),
                    z=float(msg.velocity.z),
                )
                angular = vector3_type(x=0.0, y=0.0, z=0.0)
                twist = twist3_type(linear=linear, angular=angular)
                out_msg = twist_type(header=header, twist=twist)
                writer.write(
                    out_conns[DVL_OUT_TOPIC],
                    timestamp,
                    ros2_typestore.serialize_cdr(out_msg, DVL_OUT_TYPE),
                )
                converted += 1
                continue
            if not topic_should_pass_through(connection.topic, args.include_cameras):
                skipped_camera += 1
                continue
            # Re-serialize: input is ROS 1 binary, output expects CDR. Stock
            # ROS 2 message types live in the destination typestore so the
            # round-trip is straight-forward.
            msg = ros1_typestore.deserialize_ros1(rawdata, connection.msgtype)
            writer.write(
                out_conns[connection.topic],
                timestamp,
                ros2_typestore.serialize_cdr(msg, connection.msgtype),
            )
            passthrough += 1
            # Emit synthetic /pressure derived from /depth/data Odometry.
            if connection.topic == "/depth/data":
                z = float(msg.pose.pose.position.z)
                depth_m = -z  # source uses positive-up
                pressure_pa = ATMOSPHERIC_PA + WATER_DENSITY_KGM3 * GRAVITY_MPS2 * depth_m
                fp_type = ros2_typestore.types["sensor_msgs/msg/FluidPressure"]
                header_type = ros2_typestore.types["std_msgs/msg/Header"]
                time_type = ros2_typestore.types["builtin_interfaces/msg/Time"]
                stamp = time_type(
                    sec=msg.header.stamp.sec, nanosec=msg.header.stamp.nanosec
                )
                header = header_type(stamp=stamp, frame_id="pressure_link")
                fp_msg = fp_type(header=header, fluid_pressure=pressure_pa, variance=0.0)
                writer.write(
                    out_conns[PRESSURE_OUT_TOPIC],
                    timestamp,
                    ros2_typestore.serialize_cdr(fp_msg, PRESSURE_OUT_TYPE),
                )

    print(
        f"wrote {args.dst}: {converted} DVL→TwistStamped, "
        f"{passthrough} passthrough, {dropped_dvl_invalid} DVL invalid skipped, "
        f"{skipped_camera} camera frames skipped"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
