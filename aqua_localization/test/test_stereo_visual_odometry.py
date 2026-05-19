"""Pure helper tests for stereo_visual_odometry.py."""

import importlib.util
import sys
from pathlib import Path

import cv2
import numpy as np


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "stereo_visual_odometry.py"


def load_module():
    spec = importlib.util.spec_from_file_location("stereo_visual_odometry", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_update_world_pose_inverts_pnp_transform():
    module = load_module()
    world_from_prev = np.eye(4)
    rotation = np.eye(3)
    translation = np.array([0.5, 0.0, 0.0])

    world_from_curr = module.update_world_pose(world_from_prev, rotation, translation)

    np.testing.assert_allclose(world_from_curr[:3, 3], [-0.5, 0.0, 0.0])


def test_rotation_matrix_to_quaternion_identity():
    module = load_module()

    quat = module.rotation_matrix_to_quaternion(np.eye(3))

    np.testing.assert_allclose(quat, [0.0, 0.0, 0.0, 1.0])


def test_decode_compressed_image_roundtrip():
    module = load_module()
    image = np.zeros((16, 16), dtype=np.uint8)
    image[4:12, 5:10] = 200
    ok, encoded = cv2.imencode(".png", image)
    assert ok

    decoded = module.decode_compressed_image(encoded.tobytes())

    assert decoded.shape == image.shape
    assert int(decoded[8, 8]) == 200


def test_covariance_diagonal_decreases_with_inliers():
    module = load_module()
    config = module.VisualFrontendConfig(position_variance_floor_m2=0.01)

    low = module.covariance_diagonal(config, 10)
    high = module.covariance_diagonal(config, 1000)

    assert high[0] < low[0]
    assert high[0] == 0.01


def test_failed_motion_keeps_requested_counts():
    module = load_module()

    estimate = module.failed_motion("bad frame", matches=7, inliers=3)

    assert not estimate.success
    assert estimate.matches == 7
    assert estimate.inliers == 3
