"""Pure Python tests for ros1_odometry_csv_to_tum.py."""

import importlib.util
import subprocess
import sys
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "ros1_odometry_csv_to_tum.py"


def load_module():
    spec = importlib.util.spec_from_file_location("ros1_odometry_csv_to_tum", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_convert_ros1_rostopic_csv_with_time(tmp_path):
    module = load_module()
    csv_path = tmp_path / "odom.csv"
    csv_path.write_text(
        "\n".join(
            [
                "%time,field.header.seq,field.header.stamp,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w",
                "1000000000,1,1.000000000,0.0,1.0,2.0,0.0,0.0,0.0,1.0",
                "2000000000,2,2.000000000,1.0,2.0,3.0,0.1,0.2,0.3,0.9",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    rows = module.convert_rows(csv_path)

    assert len(rows) == 2
    assert rows[0] == [1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0]
    assert rows[1] == [2.0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3, 0.9]


def test_convert_csv_with_split_stamp_and_short_headers(tmp_path):
    module = load_module()
    csv_path = tmp_path / "odom.csv"
    csv_path.write_text(
        "\n".join(
            [
                "header.stamp.secs,header.stamp.nsecs,pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w",
                "10,250000000,4.0,5.0,6.0,0.0,0.0,0.7071,0.7071",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    rows = module.convert_rows(csv_path)

    assert rows == [[10.25, 4.0, 5.0, 6.0, 0.0, 0.0, 0.7071, 0.7071]]


def test_cli_writes_tum_file(tmp_path):
    csv_path = tmp_path / "odom.csv"
    out_path = tmp_path / "odom.tum"
    csv_path.write_text(
        "\n".join(
            [
                "%time,field.pose.pose.position.x,field.pose.pose.position.y,field.pose.pose.position.z,field.pose.pose.orientation.x,field.pose.pose.orientation.y,field.pose.pose.orientation.z,field.pose.pose.orientation.w",
                "3000000000,7.0,8.0,9.0,0.0,0.0,0.0,1.0",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    proc = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            "--csv",
            str(csv_path),
            "--out",
            str(out_path),
            "--time-unit",
            "nanoseconds",
        ],
        check=True,
        text=True,
        capture_output=True,
    )

    assert "wrote 1 poses" in proc.stdout
    assert out_path.read_text(encoding="utf-8").strip() == (
        "3.000000000 7.000000000 8.000000000 9.000000000 "
        "0.000000000 0.000000000 0.000000000 1.000000000"
    )
