"""Helper tests for rerun_export_mbes.py without requiring rerun/rosbags."""

import importlib.util
import math
import sys
import types
from pathlib import Path

import numpy as np


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "rerun_export_mbes.py"


def install_stub_modules():
    if "rerun" not in sys.modules:
        rerun_stub = types.ModuleType("rerun")
        rerun_stub.ViewCoordinates = types.SimpleNamespace(RIGHT_HAND_Z_UP="right_hand_z_up")
        rerun_stub.init = lambda *a, **k: None
        rerun_stub.save = lambda *a, **k: None
        rerun_stub.log = lambda *a, **k: None
        rerun_stub.set_time = lambda *a, **k: None
        rerun_stub.Scalars = lambda *a, **k: ("Scalars", a, k)
        rerun_stub.Points3D = lambda *a, **k: ("Points3D", a, k)
        rerun_stub.LineStrips3D = lambda *a, **k: ("LineStrips3D", a, k)
        sys.modules["rerun"] = rerun_stub

    if "rerun.blueprint" not in sys.modules:
        blueprint_stub = types.ModuleType("rerun.blueprint")
        blueprint_stub.Blueprint = lambda *a, **k: ("Blueprint", a, k)
        blueprint_stub.Horizontal = lambda *a, **k: ("Horizontal", a, k)
        blueprint_stub.Vertical = lambda *a, **k: ("Vertical", a, k)
        blueprint_stub.Spatial3DView = lambda *a, **k: ("Spatial3DView", a, k)
        blueprint_stub.TimeSeriesView = lambda *a, **k: ("TimeSeriesView", a, k)
        blueprint_stub.SelectionPanel = lambda *a, **k: ("SelectionPanel", a, k)
        blueprint_stub.TimePanel = lambda *a, **k: ("TimePanel", a, k)
        blueprint_stub.BlueprintPanel = lambda *a, **k: ("BlueprintPanel", a, k)
        blueprint_stub.archetypes = types.SimpleNamespace(
            EyeControls3D=lambda *a, **k: ("EyeControls3D", a, k)
        )
        sys.modules["rerun.blueprint"] = blueprint_stub

    if "rosbags" not in sys.modules:
        rosbags_stub = types.ModuleType("rosbags")
        rosbags_highlevel_stub = types.ModuleType("rosbags.highlevel")

        class _AnyReader:
            def __init__(self, *a, **k):
                raise RuntimeError("AnyReader stub should not be constructed in helper tests")

        rosbags_highlevel_stub.AnyReader = _AnyReader
        sys.modules["rosbags"] = rosbags_stub
        sys.modules["rosbags.highlevel"] = rosbags_highlevel_stub


def load_module():
    install_stub_modules()
    spec = importlib.util.spec_from_file_location("rerun_export_mbes", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _Stamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, stamp):
        self.stamp = stamp


class _Position:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class _Pose:
    def __init__(self, x, y, z):
        self.position = _Position(x, y, z)


class _PoseStamped:
    def __init__(self, stamp, x, y, z):
        self.header = _Header(stamp)
        self.pose = _Pose(x, y, z)


class _Path:
    def __init__(self, poses):
        self.poses = poses


def test_stamp_seconds_combines_sec_and_nanosec():
    module = load_module()
    assert math.isclose(module.stamp_seconds(_Stamp(12, 500_000_000)), 12.5)


def test_path_samples_uses_fallback_time_for_zero_stamp():
    module = load_module()
    msg = _Path([
        _PoseStamped(_Stamp(0, 0), 1.0, 2.0, 3.0),
        _PoseStamped(_Stamp(4, 250_000_000), 4.0, 5.0, 6.0),
    ])

    samples = module.path_samples(msg, fallback_time=99.0)

    assert len(samples) == 2
    assert samples[0][0] == 99.0
    np.testing.assert_allclose(samples[0][1], [1.0, 2.0, 3.0])
    assert math.isclose(samples[1][0], 4.25)
    np.testing.assert_allclose(samples[1][1], [4.0, 5.0, 6.0])


def test_loop_edge_segments_deduplicates_and_skips_invalid_ids():
    module = load_module()
    pose_graph = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
    ])
    constraints = [
        (1.0, 0, 2),
        (2.0, 0, 2),
        (3.0, -1, 2),
        (4.0, 1, 99),
        (5.0, 1, 2),
    ]

    segments = module.loop_edge_segments(pose_graph, constraints)

    assert len(segments) == 2
    np.testing.assert_allclose(segments[0], [[0.0, 0.0, 0.0], [2.0, 1.0, 0.0]])
    np.testing.assert_allclose(segments[1], [[1.0, 0.0, 0.0], [2.0, 1.0, 0.0]])
