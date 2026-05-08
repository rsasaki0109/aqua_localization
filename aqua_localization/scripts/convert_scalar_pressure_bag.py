#!/usr/bin/env python3
"""Convert a scalar pressure/depth/barometer topic in a rosbag2 to FluidPressure.

The runtime `scalar_to_pressure_node` republishes scalar messages live, but it stamps
each FluidPressure with `now()` because the scalar messages have no `Header`. For
high-fidelity bag replay this loses the original sensor timestamp. This script reads
a source rosbag2, copies all messages through, and replaces (or appends) the chosen
scalar topic with a `sensor_msgs/msg/FluidPressure` topic stamped with the original
recorded time.

Three conversion modes mirror `aqua_imu_loc/src/scalar_to_pressure_node.cpp`:

- `pressure_pa`  : scalar is already absolute pressure in pascals.
- `depth_m`      : scalar is positive-down depth in meters.
- `ntnu_barometer` (default): scalar is converted with the NTNU barometer formula
  ``depth_m = -((scalar - barometer_pressure_offset) / barometer_pressure_scale)``.
"""

import argparse
import math
import shutil
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore


SCALAR_TYPES = {"std_msgs/msg/Float32", "std_msgs/msg/Float64"}


def scalar_to_pressure_pa(
    scalar: float,
    *,
    mode: str,
    pressure_offset_pa: float = 0.0,
    reference_pressure_pa: float = 101325.0,
    water_density_kg_m3: float = 1025.0,
    gravity_mps2: float = 9.80665,
    depth_offset_m: float = 0.0,
    barometer_pressure_offset: float = 0.0,
    barometer_pressure_scale: float = 1.0,
    min_depth_m: float = -1.0,
    max_depth_m: float = 10000.0,
) -> float:
    """Convert one scalar reading to absolute pressure in pascals.

    Returns NaN when the scalar is not finite, scale is zero, or the implied depth
    leaves the [min_depth_m, max_depth_m] window — callers should drop those.
    """
    if not math.isfinite(scalar):
        return math.nan

    if mode == "pressure_pa":
        return scalar + pressure_offset_pa

    if mode == "depth_m":
        depth_m = scalar
    elif mode == "ntnu_barometer":
        if abs(barometer_pressure_scale) <= 1.0e-12 or not math.isfinite(barometer_pressure_scale):
            return math.nan
        depth_m = -((scalar - barometer_pressure_offset) / barometer_pressure_scale)
    else:
        raise ValueError(f"unknown mode: {mode}")

    corrected = depth_m + depth_offset_m
    if not math.isfinite(corrected):
        return math.nan
    if corrected < min_depth_m or corrected > max_depth_m:
        return math.nan
    return reference_pressure_pa + water_density_kg_m3 * gravity_mps2 * corrected


def make_pressure_message(typestore, frame_id: str, pressure_pa: float, variance_pa2: float):
    """Build a rosbags FluidPressure message for the destination typestore."""
    FluidPressure = typestore.types["sensor_msgs/msg/FluidPressure"]
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]
    # The header stamp is overwritten per message before write; default placeholder.
    header = Header(stamp=Time(sec=0, nanosec=0), frame_id=frame_id)
    return FluidPressure(header=header, fluid_pressure=pressure_pa, variance=variance_pa2)


def convert(args) -> int:
    src = Path(args.src)
    dst = Path(args.dst)
    if dst.exists():
        if args.overwrite:
            shutil.rmtree(dst)
        else:
            print(f"refusing to overwrite existing {dst}; pass --overwrite", file=sys.stderr)
            return 2

    typestore = get_typestore(Stores.ROS2_JAZZY)

    with Reader(src) as reader, Writer(dst, version=9) as writer:
        scalar_connection = None
        scalar_msgtype = None
        out_connections = {}
        pressure_writer_conn = None

        # Find scalar source connection.
        for conn in reader.connections:
            if conn.topic == args.scalar_topic:
                scalar_connection = conn
                scalar_msgtype = conn.msgtype
                if conn.msgtype not in SCALAR_TYPES:
                    print(
                        f"{args.scalar_topic} has unsupported type {conn.msgtype}",
                        file=sys.stderr,
                    )
                    return 3

        if scalar_connection is None:
            print(
                f"scalar topic {args.scalar_topic} not found in {src}", file=sys.stderr
            )
            return 4

        # Mirror all source connections into the destination, except optionally
        # the scalar topic when --replace is set.
        for conn in reader.connections:
            if args.replace and conn.topic == args.scalar_topic:
                continue
            out_connections[conn.id] = writer.add_connection(
                topic=conn.topic,
                msgtype=conn.msgtype,
                typestore=typestore,
            )

        pressure_writer_conn = writer.add_connection(
            topic=args.pressure_topic,
            msgtype="sensor_msgs/msg/FluidPressure",
            typestore=typestore,
        )

        kept = 0
        emitted = 0
        rejected = 0
        for conn, timestamp_ns, raw in reader.messages():
            if conn.topic == args.scalar_topic:
                msg = typestore.deserialize_cdr(raw, scalar_msgtype)
                pa = scalar_to_pressure_pa(
                    float(msg.data),
                    mode=args.mode,
                    pressure_offset_pa=args.pressure_offset_pa,
                    reference_pressure_pa=args.reference_pressure_pa,
                    water_density_kg_m3=args.water_density_kg_m3,
                    gravity_mps2=args.gravity_mps2,
                    depth_offset_m=args.depth_offset_m,
                    barometer_pressure_offset=args.barometer_pressure_offset,
                    barometer_pressure_scale=args.barometer_pressure_scale,
                    min_depth_m=args.min_depth_m,
                    max_depth_m=args.max_depth_m,
                )
                if not math.isfinite(pa):
                    rejected += 1
                else:
                    pressure_msg = make_pressure_message(
                        typestore, args.frame_id, pa, args.pressure_variance_pa2
                    )
                    pressure_msg.header.stamp.sec = timestamp_ns // 1_000_000_000
                    pressure_msg.header.stamp.nanosec = timestamp_ns % 1_000_000_000
                    writer.write(
                        pressure_writer_conn,
                        timestamp_ns,
                        typestore.serialize_cdr(
                            pressure_msg, "sensor_msgs/msg/FluidPressure"
                        ),
                    )
                    emitted += 1

                if args.replace:
                    continue  # do not also copy the scalar through

            if conn.id in out_connections:
                writer.write(out_connections[conn.id], timestamp_ns, raw)
                kept += 1

        print(
            f"copied={kept} pressure_emitted={emitted} rejected={rejected} "
            f"src={src} dst={dst}"
        )
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Convert a scalar pressure/depth/barometer topic in a rosbag2 to FluidPressure."
    )
    parser.add_argument("--src", required=True, help="Source rosbag2 directory.")
    parser.add_argument("--dst", required=True, help="Destination rosbag2 directory (must not exist unless --overwrite).")
    parser.add_argument("--scalar-topic", required=True, help="Scalar topic name to read from src.")
    parser.add_argument(
        "--pressure-topic",
        default="/pressure",
        help="FluidPressure topic name to write into dst (default: /pressure).",
    )
    parser.add_argument(
        "--mode",
        choices=["pressure_pa", "depth_m", "ntnu_barometer"],
        default="ntnu_barometer",
        help="Conversion mode (default: ntnu_barometer).",
    )
    parser.add_argument("--frame-id", default="pressure_link", help="header.frame_id for FluidPressure.")
    parser.add_argument("--pressure-variance-pa2", type=float, default=0.0)
    parser.add_argument("--pressure-offset-pa", type=float, default=0.0)
    parser.add_argument("--reference-pressure-pa", type=float, default=101325.0)
    parser.add_argument("--water-density-kg-m3", type=float, default=1025.0)
    parser.add_argument("--gravity-mps2", type=float, default=9.80665)
    parser.add_argument("--depth-offset-m", type=float, default=0.0)
    parser.add_argument("--barometer-pressure-offset", type=float, default=0.0)
    parser.add_argument("--barometer-pressure-scale", type=float, default=1.0)
    parser.add_argument("--min-depth-m", type=float, default=-1.0)
    parser.add_argument("--max-depth-m", type=float, default=10000.0)
    parser.add_argument(
        "--replace",
        action="store_true",
        help="Drop the scalar topic from the destination instead of keeping both.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Remove the destination directory if it already exists.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    return convert(args)


if __name__ == "__main__":
    sys.exit(main())
