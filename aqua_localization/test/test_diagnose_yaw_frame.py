"""Pure-function tests for diagnose_yaw_frame.py."""

import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "diagnose_yaw_frame.py"


def load_module():
    pytest.importorskip("rosbags.rosbag2")
    spec = importlib.util.spec_from_file_location("diagnose_yaw_frame", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_quaternion_to_yaw_known_values():
    module = load_module()
    # Identity quaternion -> yaw 0
    assert module.quaternion_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)

    # 90 deg yaw about Z: q = (0, 0, sin(45), cos(45))
    sin45 = math.sin(math.pi / 4.0)
    cos45 = math.cos(math.pi / 4.0)
    assert module.quaternion_to_yaw(0.0, 0.0, sin45, cos45) == pytest.approx(math.pi / 2.0, rel=1e-9)

    # -45 deg yaw
    assert module.quaternion_to_yaw(
        0.0, 0.0, math.sin(-math.pi / 8.0), math.cos(-math.pi / 8.0)
    ) == pytest.approx(-math.pi / 4.0, rel=1e-9)


def test_normalize_angle_wraps_to_minus_pi_to_pi():
    module = load_module()
    assert module.normalize_angle(0.0) == pytest.approx(0.0)
    # atan2 returns values in (-pi, pi]; +pi and -pi are equivalent here.
    assert abs(abs(module.normalize_angle(3.0 * math.pi)) - math.pi) < 1.0e-9
    assert abs(abs(module.normalize_angle(-3.0 * math.pi)) - math.pi) < 1.0e-9
    assert module.normalize_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1, abs=1e-9)


def test_unwrap_yaw_delta_handles_pi_crossing():
    module = load_module()
    # Walking yaw across +pi: previous raw = pi - 0.05, current raw = -pi + 0.05.
    new_unwrapped = module.unwrap_yaw_delta(0.0, math.pi - 0.05, -math.pi + 0.05)
    assert new_unwrapped == pytest.approx(0.10, abs=1e-9)

    # And across -pi the other way: prev = -pi + 0.05, current = pi - 0.05.
    new_unwrapped = module.unwrap_yaw_delta(0.0, -math.pi + 0.05, math.pi - 0.05)
    assert new_unwrapped == pytest.approx(-0.10, abs=1e-9)


def test_unwrap_yaw_delta_continuous_segment_accumulates():
    module = load_module()
    yaws_raw = []
    angle = 0.0
    for _ in range(50):
        angle += 0.2  # 10 rad over the loop
        yaws_raw.append(module.normalize_angle(angle))

    unwrapped = 0.0
    prev_raw = yaws_raw[0]
    for raw in yaws_raw[1:]:
        unwrapped = module.unwrap_yaw_delta(unwrapped, prev_raw, raw)
        prev_raw = raw

    # 49 steps of +0.2 = 9.8 rad. unwrap_yaw_delta should track that without modulo.
    assert unwrapped == pytest.approx(9.8, abs=1e-6)


def test_least_squares_slope_recovers_known_slope():
    module = load_module()
    x = np.linspace(0, 10, 100)
    y_pos = 0.7 * x
    y_neg = -1.4 * x
    assert module.least_squares_slope(x, y_pos) == pytest.approx(0.7)
    assert module.least_squares_slope(x, y_neg) == pytest.approx(-1.4)
    assert math.isnan(module.least_squares_slope(np.zeros(5), np.zeros(5)))
