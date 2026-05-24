"""Pure helper tests for export_rosbag_odometry_tum.py."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "export_rosbag_odometry_tum.py"


def load_module():
    spec = importlib.util.spec_from_file_location("export_rosbag_odometry_tum", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def odom_msg():
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=345000000)),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=-3.5),
                orientation=SimpleNamespace(x=0.0, y=0.1, z=0.2, w=0.97),
            )
        ),
    )


def test_bag_reader_path_uses_parent_for_mcap():
    module = load_module()

    assert module.bag_reader_path(Path("/tmp/demo/out.mcap")) == Path("/tmp/demo")
    assert module.bag_reader_path(Path("/tmp/demo_ros2")) == Path("/tmp/demo_ros2")


def test_odometry_to_tum_line_uses_header_stamp():
    module = load_module()

    line = module.odometry_to_tum_line(odom_msg(), fallback_timestamp_ns=99_000_000_000)

    assert line == "12.345000000 1.000000000 2.000000000 -3.500000000 0.000000000 0.100000000 0.200000000 0.970000000\n"
