#!/usr/bin/env python3
"""Convert ROS 1 `rostopic echo -p` Odometry CSV output to TUM format.

This is intended for external ROS 1 baselines such as AQUA-SLAM. Record its
`nav_msgs/Odometry` topic with:

    rostopic echo -p /AQUA_SLAM/orb_odom > aqua_slam_orb_odom.csv

Then convert the CSV to the same TUM format used by compare_trajectories.py.
"""

import argparse
import csv
import sys
from pathlib import Path


POSITION_SUFFIXES = {
    "x": ("pose.pose.position.x", "pose.position.x", "position.x"),
    "y": ("pose.pose.position.y", "pose.position.y", "position.y"),
    "z": ("pose.pose.position.z", "pose.position.z", "position.z"),
}
ORIENTATION_SUFFIXES = {
    "x": ("pose.pose.orientation.x", "pose.orientation.x", "orientation.x"),
    "y": ("pose.pose.orientation.y", "pose.orientation.y", "orientation.y"),
    "z": ("pose.pose.orientation.z", "pose.orientation.z", "orientation.z"),
    "w": ("pose.pose.orientation.w", "pose.orientation.w", "orientation.w"),
}


def normalize_header(name: str) -> str:
    name = name.strip()
    if name.startswith("field."):
        name = name[len("field."):]
    return name


def find_column(headers, suffixes):
    normalized = {normalize_header(h): h for h in headers}
    for suffix in suffixes:
        if suffix in normalized:
            return normalized[suffix]
    for header in headers:
        clean = normalize_header(header)
        if any(clean.endswith(suffix) for suffix in suffixes):
            return header
    raise ValueError(f"missing CSV column matching one of: {', '.join(suffixes)}")


def parse_float(value: str) -> float:
    value = value.strip()
    if not value:
        raise ValueError("empty numeric value")
    return float(value)


def normalize_timestamp(raw: float, time_unit: str) -> float:
    if time_unit == "nanoseconds":
        return raw * 1e-9
    if time_unit == "seconds":
        return raw
    return raw * 1e-9 if raw > 1e12 else raw


def timestamp_from_row(row: dict, headers, time_unit: str) -> float:
    if "field.header.stamp" in row and row["field.header.stamp"].strip():
        return normalize_timestamp(parse_float(row["field.header.stamp"]), time_unit)
    if "header.stamp" in row and row["header.stamp"].strip():
        return normalize_timestamp(parse_float(row["header.stamp"]), time_unit)
    if "%time" in row and row["%time"].strip():
        return normalize_timestamp(parse_float(row["%time"]), time_unit)

    normalized = {normalize_header(h): h for h in headers}
    sec_col = normalized.get("header.stamp.secs") or normalized.get("header.stamp.sec")
    nsec_col = normalized.get("header.stamp.nsecs") or normalized.get("header.stamp.nanosec")
    if sec_col and nsec_col:
        return parse_float(row[sec_col]) + parse_float(row[nsec_col]) * 1e-9

    raise ValueError("missing timestamp column; expected %time or header stamp fields")


def detect_columns(headers):
    columns = {}
    for axis, suffixes in POSITION_SUFFIXES.items():
        columns[f"t{axis}"] = find_column(headers, suffixes)
    for axis, suffixes in ORIENTATION_SUFFIXES.items():
        columns[f"q{axis}"] = find_column(headers, suffixes)
    return columns


def convert_rows(csv_path: Path, time_unit: str = "auto"):
    with csv_path.open("r", encoding="utf-8", newline="") as fp:
        reader = csv.DictReader(fp)
        if not reader.fieldnames:
            raise ValueError(f"{csv_path}: empty CSV")
        headers = reader.fieldnames
        columns = detect_columns(headers)
        rows = []
        for line_number, row in enumerate(reader, start=2):
            try:
                timestamp = timestamp_from_row(row, headers, time_unit)
                values = [
                    timestamp,
                    parse_float(row[columns["tx"]]),
                    parse_float(row[columns["ty"]]),
                    parse_float(row[columns["tz"]]),
                    parse_float(row[columns["qx"]]),
                    parse_float(row[columns["qy"]]),
                    parse_float(row[columns["qz"]]),
                    parse_float(row[columns["qw"]]),
                ]
            except ValueError as exc:
                raise ValueError(f"{csv_path}:{line_number}: {exc}") from exc
            rows.append(values)
    rows.sort(key=lambda r: r[0])
    return rows


def write_tum(rows, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as fp:
        for row in rows:
            fp.write(
                f"{row[0]:.9f} {row[1]:.9f} {row[2]:.9f} {row[3]:.9f} "
                f"{row[4]:.9f} {row[5]:.9f} {row[6]:.9f} {row[7]:.9f}\n"
            )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Convert ROS 1 rostopic echo -p Odometry CSV output to TUM trajectory format."
    )
    parser.add_argument("--csv", required=True, type=Path, help="Input CSV from rostopic echo -p.")
    parser.add_argument("--out", required=True, type=Path, help="Output TUM trajectory path.")
    parser.add_argument(
        "--time-unit",
        choices=("auto", "seconds", "nanoseconds"),
        default="auto",
        help="Unit for %%time when header stamp fields are absent. Default auto treats very large values as ns.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rows = convert_rows(args.csv, time_unit=args.time_unit)
    if not rows:
        raise ValueError(f"{args.csv}: no odometry rows found")
    write_tum(rows, args.out)
    print(f"wrote {len(rows)} poses to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
