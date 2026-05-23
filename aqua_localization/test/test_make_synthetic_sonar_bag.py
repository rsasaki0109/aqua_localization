"""Pure-function tests for make_synthetic_sonar_bag.py.

The bag I/O path needs rosbags; if it isn't available we skip. The geometric helpers
are exercised without any I/O.
"""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "make_synthetic_sonar_bag.py"
)


def load_module():
    pytest.importorskip("rosbags.rosbag2")
    spec = importlib.util.spec_from_file_location("make_synthetic_sonar_bag", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_linear_trajectory_increments_along_x():
    module = load_module()
    traj = module.linear_trajectory(num_steps=5, dt=0.1, speed_m_s=2.0)
    assert traj.shape == (5, 3)
    np.testing.assert_allclose(traj[:, 0], [0.0, 0.2, 0.4, 0.6, 0.8])
    np.testing.assert_allclose(traj[:, 1], 0.0)
    np.testing.assert_allclose(traj[:, 2], 0.0)


def test_sample_world_points_within_box():
    module = load_module()
    rng = np.random.default_rng(0)
    pts = module.sample_world_points(rng, 200, (1.0, 2.0), (-1.0, 1.0), (0.0, 0.5))
    assert pts.shape == (200, 3)
    assert (pts[:, 0] >= 1.0).all() and (pts[:, 0] <= 2.0).all()
    assert (pts[:, 1] >= -1.0).all() and (pts[:, 1] <= 1.0).all()
    assert (pts[:, 2] >= 0.0).all() and (pts[:, 2] <= 0.5).all()


def test_transform_world_to_body_subtracts_robot_position():
    module = load_module()
    world = np.array([[5.0, 1.0, 0.0], [10.0, -2.0, 1.0]])
    pose = np.array([3.0, 0.0, 0.5])
    body = module.transform_world_to_body(world, pose)
    np.testing.assert_allclose(body, [[2.0, 1.0, -0.5], [7.0, -2.0, 0.5]])


def test_filter_by_range_drops_far_points():
    module = load_module()
    pts = np.array([[1.0, 0.0, 0.0], [50.0, 0.0, 0.0], [3.0, 4.0, 0.0]])  # |p| = 1, 50, 5
    kept = module.filter_by_range(pts, max_range_m=10.0)
    np.testing.assert_allclose(kept, [[1.0, 0.0, 0.0], [3.0, 4.0, 0.0]])


def test_encode_message_packs_three_floats_per_point():
    module = load_module()
    from rosbags.typesys import get_typestore
    typestore = get_typestore(module.default_ros2_store())
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float64)
    msg = module.encode_xyz_float32_message(
        typestore, pts, stamp_sec=10, stamp_nsec=250_000_000, frame_id="sonar_link",
    )
    assert msg.height == 1
    assert msg.width == 2
    assert msg.point_step == 12
    assert msg.row_step == 24
    assert msg.is_dense is True
    assert msg.header.frame_id == "sonar_link"
    floats = np.frombuffer(msg.data, dtype=np.float32)
    np.testing.assert_allclose(floats, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
