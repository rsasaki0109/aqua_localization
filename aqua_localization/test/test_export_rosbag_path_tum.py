"""Pure helper tests for export_rosbag_path_tum.py."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "export_rosbag_path_tum.py"


def load_module():
    spec = importlib.util.spec_from_file_location("export_rosbag_path_tum", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stamp(sec, nanosec):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def pose_stamped(sec, nanosec, x):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(sec, nanosec)),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=2.0, z=-3.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.1, w=0.99),
        ),
    )


def test_bag_reader_path_uses_parent_for_mcap():
    module = load_module()

    assert module.bag_reader_path(Path("/tmp/demo/out.mcap")) == Path("/tmp/demo")
    assert module.bag_reader_path(Path("/tmp/demo_ros2")) == Path("/tmp/demo_ros2")


def test_pose_stamped_to_tum_line_uses_pose_stamp():
    module = load_module()

    line = module.pose_stamped_to_tum_line(pose_stamped(12, 345000000, 1.0), 99.0)

    assert line == "12.345000000 1.000000000 2.000000000 -3.000000000 0.000000000 0.000000000 0.100000000 0.990000000\n"


def test_path_to_tum_lines_falls_back_to_path_stamp():
    module = load_module()
    msg = SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(44, 500000000)),
        poses=[
            pose_stamped(0, 0, 1.0),
            pose_stamped(45, 0, 2.0),
        ],
    )

    lines = module.path_to_tum_lines(msg, fallback_timestamp_ns=99_000_000_000)

    assert lines[0].startswith("44.500000000 1.000000000")
    assert lines[1].startswith("45.000000000 2.000000000")
