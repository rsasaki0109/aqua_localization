"""Pure-format tests for record_status.py. Stubs out rclpy/aqua_msgs imports."""

import importlib.util
import math
import sys
import types
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "record_status.py"


def install_stub_modules():
    if "rclpy" not in sys.modules:
        rclpy_stub = types.ModuleType("rclpy")
        rclpy_stub.init = lambda *a, **k: None
        rclpy_stub.shutdown = lambda *a, **k: None
        rclpy_stub.spin = lambda *a, **k: None
        rclpy_stub.ok = lambda *a, **k: True
        rclpy_stub.__path__ = []

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

        sys.modules["rclpy"] = rclpy_stub
        sys.modules["rclpy.node"] = node_stub
        sys.modules["rclpy.qos"] = qos_stub
        sys.modules["rclpy.executors"] = executors_stub

    if "aqua_msgs" not in sys.modules:
        aqua_msgs = types.ModuleType("aqua_msgs")
        aqua_msgs_msg = types.ModuleType("aqua_msgs.msg")

        class _EstimatorStatus:
            pass

        aqua_msgs_msg.EstimatorStatus = _EstimatorStatus
        sys.modules["aqua_msgs"] = aqua_msgs
        sys.modules["aqua_msgs.msg"] = aqua_msgs_msg


def load_module():
    install_stub_modules()
    spec = importlib.util.spec_from_file_location("record_status", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, stamp, frame_id):
        self.stamp = stamp
        self.frame_id = frame_id


class _EstimatorStatus:
    def __init__(self, **kwargs):
        self.header = _Header(_Stamp(1700604800, 250_000_000), kwargs.get("frame_id", "odom"))
        self.estimator_name = "aqua_imu_loc"
        self.backend = "additive_ukf"
        self.initialized = True
        self.update_count = kwargs.get("update_count", 1234)
        self.last_prediction_dt = kwargs.get("last_prediction_dt", 0.04)
        self.position_covariance_trace = kwargs.get("pcov", 7.5)
        self.orientation_covariance_trace = kwargs.get("ocov", 0.5)
        self.status = "running"
        self.accel_bias = kwargs.get("accel_bias", [0.05, -0.01, -1.8])
        self.gyro_bias = kwargs.get("gyro_bias", [-0.02, 0.01, 0.015])
        self.ahrs_gyro_bias_z_enabled = kwargs.get("ahrs_enabled", True)
        self.ahrs_gyro_bias_z_active = kwargs.get("ahrs_active", True)
        self.ahrs_gyro_bias_z_last_observed = kwargs.get("ahrs_last", 0.0067)


def test_csv_header_lists_expected_fields():
    module = load_module()
    headers = module.CSV_HEADER.strip().split(",")
    assert "timestamp" in headers
    assert "gyro_bias_z" in headers
    assert "ahrs_gyro_bias_z_active" in headers
    assert "ahrs_gyro_bias_z_last_observed" in headers
    assert "accel_bias_z" in headers


def test_csv_line_field_count_matches_header():
    module = load_module()
    line = module.format_csv_line(_EstimatorStatus()).strip()
    header_count = len(module.CSV_HEADER.strip().split(","))
    line_count = len(line.split(","))
    assert line_count == header_count


def test_csv_line_serializes_bias_and_flags():
    module = load_module()
    msg = _EstimatorStatus(
        gyro_bias=[0.001, -0.002, 0.0167],
        accel_bias=[0.0, 0.0, -1.5],
        ahrs_active=True, ahrs_last=0.012,
    )
    line = module.format_csv_line(msg).strip().split(",")
    timestamp = float(line[0])
    assert math.isclose(timestamp, 1700604800.25, rel_tol=1e-9)
    # Find columns by index from header.
    headers = module.CSV_HEADER.strip().split(",")
    gyro_z_idx = headers.index("gyro_bias_z")
    assert float(line[gyro_z_idx]) == 0.0167
    active_idx = headers.index("ahrs_gyro_bias_z_active")
    assert int(line[active_idx]) == 1
    last_idx = headers.index("ahrs_gyro_bias_z_last_observed")
    assert math.isclose(float(line[last_idx]), 0.012, rel_tol=1e-9)


def test_stamp_to_seconds_combines_sec_and_nanosec():
    module = load_module()
    seconds = module.stamp_to_seconds(_Stamp(5, 750_000_000))
    assert math.isclose(seconds, 5.75, rel_tol=1e-9)
