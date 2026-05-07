"""Pure-format tests for record_odometry.py — no ROS context required.

These tests stub out rclpy/nav_msgs imports so the module's formatting helpers can
be exercised without spinning a node.
"""

import importlib.util
import sys
import types
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "record_odometry.py"


def install_stub_modules():
    """Install lightweight stand-ins so import of record_odometry succeeds."""
    if "rclpy" not in sys.modules:
        rclpy_stub = types.ModuleType("rclpy")
        rclpy_stub.init = lambda *a, **k: None
        rclpy_stub.shutdown = lambda *a, **k: None
        rclpy_stub.spin = lambda *a, **k: None
        node_stub = types.ModuleType("rclpy.node")

        class _Node:
            def __init__(self, *a, **k):
                pass

            def create_subscription(self, *a, **k):
                return None

            def get_logger(self):
                class _L:
                    def info(self, *a, **k):
                        pass

                return _L()

            def destroy_node(self):
                return True

        node_stub.Node = _Node
        qos_stub = types.ModuleType("rclpy.qos")

        class _QoSProfile:
            def __init__(self, depth=10):
                self.depth = depth
                self.reliability = None

        class _ReliabilityPolicy:
            RELIABLE = "reliable"
            BEST_EFFORT = "best_effort"

        qos_stub.QoSProfile = _QoSProfile
        qos_stub.ReliabilityPolicy = _ReliabilityPolicy

        executors_stub = types.ModuleType("rclpy.executors")

        class _ExternalShutdownException(Exception):
            pass

        executors_stub.ExternalShutdownException = _ExternalShutdownException

        rclpy_stub.ok = lambda *a, **k: True
        rclpy_stub.__path__ = []  # mark as package for "from rclpy.X import Y"
        sys.modules["rclpy"] = rclpy_stub
        sys.modules["rclpy.node"] = node_stub
        sys.modules["rclpy.qos"] = qos_stub
        sys.modules["rclpy.executors"] = executors_stub

    if "nav_msgs" not in sys.modules:
        nav_msgs = types.ModuleType("nav_msgs")
        nav_msgs_msg = types.ModuleType("nav_msgs.msg")

        class _Odometry:
            pass

        nav_msgs_msg.Odometry = _Odometry
        sys.modules["nav_msgs"] = nav_msgs
        sys.modules["nav_msgs.msg"] = nav_msgs_msg


def load_module():
    install_stub_modules()
    spec = importlib.util.spec_from_file_location("record_odometry", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Vec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Quat:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class _Pose:
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class _PoseWithCov:
    def __init__(self, pose, covariance):
        self.pose = pose
        self.covariance = covariance


class _Twist:
    def __init__(self, linear, angular):
        self.linear = linear
        self.angular = angular


class _TwistWithCov:
    def __init__(self, twist):
        self.twist = twist
        self.covariance = [0.0] * 36


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, stamp, frame_id):
        self.stamp = stamp
        self.frame_id = frame_id


class _Odometry:
    def __init__(self, sec, nanosec, frame, child, position, orientation, linear, angular, cov):
        self.header = _Header(_Stamp(sec, nanosec), frame)
        self.child_frame_id = child
        self.pose = _PoseWithCov(_Pose(position, orientation), cov)
        self.twist = _TwistWithCov(_Twist(linear, angular))


def make_message():
    return _Odometry(
        sec=1700604800,
        nanosec=500_000_000,
        frame="odom",
        child="base_link",
        position=_Vec(1.5, -2.25, -0.05),
        orientation=_Quat(0.0, 0.0, 0.7071, 0.7071),
        linear=_Vec(0.5, 0.1, 0.0),
        angular=_Vec(0.0, 0.0, 0.2),
        cov=[
            0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.04, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.002,
        ],
    )


def test_tum_line_has_eight_fields_and_correct_timestamp():
    module = load_module()
    line = module.format_tum_line(make_message()).strip()
    fields = line.split(" ")
    assert len(fields) == 8
    assert float(fields[0]) == 1700604800.5
    assert float(fields[1]) == 1.5
    assert float(fields[2]) == -2.25
    assert float(fields[3]) == -0.05
    assert float(fields[7]) == 0.7071


def test_csv_header_and_line_field_counts_match():
    module = load_module()
    header_fields = module.CSV_HEADER.strip().split(",")
    line = module.format_csv_line(make_message()).strip()
    line_fields = line.split(",")
    assert len(line_fields) == len(header_fields)
    assert header_fields[0] == "timestamp"
    assert line_fields[1] == "odom"
    assert line_fields[2] == "base_link"
    assert float(line_fields[3]) == 1.5
    assert float(line_fields[16]) == 0.04  # cov_xx
    assert float(line_fields[17]) == 0.04  # cov_yy
    assert float(line_fields[18]) == 0.01  # cov_zz


def test_stamp_to_seconds_handles_subnano():
    module = load_module()
    seconds = module.stamp_to_seconds(_Stamp(10, 250_000_000))
    assert abs(seconds - 10.25) < 1e-9
