"""Pure helper tests for stereo_visual_odometry.py."""

import importlib.util
import csv
import io
import json
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


def test_world_from_base_pose_applies_base_from_camera_lever_arm():
    module = load_module()
    extrinsics = module.VisualExtrinsics(base_from_camera_x_m=1.0)
    world_from_camera = np.eye(4)
    world_from_camera[:3, 3] = [2.0, 0.0, 0.0]

    world_from_base = module.world_from_base_pose(
        world_from_camera,
        module.base_from_camera_transform(extrinsics),
        publish_base_pose=True,
    )

    np.testing.assert_allclose(world_from_base[:3, 3], [1.0, 0.0, 0.0])


def test_world_from_base_pose_can_keep_camera_pose():
    module = load_module()
    extrinsics = module.VisualExtrinsics(base_from_camera_x_m=1.0)
    world_from_camera = np.eye(4)
    world_from_camera[:3, 3] = [2.0, 0.0, 0.0]

    publish_pose = module.world_from_base_pose(
        world_from_camera,
        module.base_from_camera_transform(extrinsics),
        publish_base_pose=False,
    )

    np.testing.assert_allclose(publish_pose[:3, 3], [2.0, 0.0, 0.0])


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


def test_filter_descriptor_matches_applies_hamming_threshold():
    module = load_module()
    matches = [
        cv2.DMatch(_queryIdx=0, _trainIdx=0, _distance=12.0),
        cv2.DMatch(_queryIdx=1, _trainIdx=1, _distance=96.0),
        cv2.DMatch(_queryIdx=2, _trainIdx=2, _distance=97.0),
    ]

    filtered = module.filter_descriptor_matches(matches, max_distance=96.0)
    disabled = module.filter_descriptor_matches(matches, max_distance=0.0)

    assert [match.queryIdx for match in filtered] == [0, 1]
    assert len(disabled) == 3


def test_warm_up_feature_pipeline_runs_orb_and_matcher():
    module = load_module()
    orb = cv2.ORB_create(nfeatures=200)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    elapsed_ms = module.warm_up_feature_pipeline(orb, matcher, iterations=1)

    assert elapsed_ms >= 0.0


def test_make_status_reports_tracking_diagnostics():
    module = load_module()
    frame = module.VisualFrame(
        stamp_s=12.5,
        points3d=np.zeros((11, 3), dtype=np.float32),
        keypoints_xy=np.zeros((11, 2), dtype=np.float32),
        descriptors=np.zeros((11, 32), dtype=np.uint8),
        disparities_px=np.array([2.0, 4.0, 8.0], dtype=np.float32),
        left_features=40,
        right_features=35,
        stereo_matches=20,
    )
    estimate = module.MotionEstimate(
        True,
        np.eye(3),
        np.array([0.3, 0.4, 0.0]),
        inliers=8,
        matches=16,
    )
    times = module.ProcessingTimes(
        decode_time_ms=1.5,
        stereo_time_ms=12.0,
        tracking_time_ms=3.25,
        total_time_ms=16.75,
    )

    status = module.make_status(12.5, 3, 2, 1, frame, estimate, times)
    payload = json.loads(module.status_to_json(status))

    assert payload["left_features"] == 40
    assert payload["stereo_points"] == 11
    assert payload["disparity_median_px"] == 4.0
    assert payload["depth_median_m"] == 0.0
    assert payload["temporal_matches"] == 16
    assert payload["pnp_inliers"] == 8
    assert payload["inlier_ratio"] == 0.5
    assert payload["step_translation_m"] == 0.5
    assert payload["decode_time_ms"] == 1.5
    assert payload["stereo_time_ms"] == 12.0
    assert payload["tracking_time_ms"] == 3.25
    assert payload["total_time_ms"] == 16.75
    assert payload["status"] == "accepted"


def test_status_csv_writer_emits_stable_columns():
    module = load_module()
    status = module.VisualFrontendStatus(
        stamp_s=1.25,
        frame_index=2,
        accepted_count=1,
        rejected_count=1,
        left_features=10,
        right_features=9,
        stereo_matches=8,
        stereo_points=7,
        disparity_min_px=2.0,
        disparity_median_px=4.0,
        disparity_p95_px=8.0,
        depth_min_m=1.0,
        depth_median_m=2.0,
        depth_p95_m=3.0,
        temporal_matches=6,
        pnp_inliers=5,
        inlier_ratio=0.75,
        step_translation_m=0.125,
        decode_time_ms=1.25,
        stereo_time_ms=10.5,
        tracking_time_ms=2.75,
        total_time_ms=14.5,
        accepted=False,
        status="too few pnp inliers",
    )
    fp = io.StringIO()

    module.write_status_csv_header(fp)
    module.write_status_csv_row(fp, status)

    rows = list(csv.DictReader(io.StringIO(fp.getvalue())))
    assert rows[0]["timestamp"] == "1.250000000"
    assert rows[0]["disparity_median_px"] == "4.000000000"
    assert rows[0]["depth_p95_m"] == "3.000000000"
    assert rows[0]["decode_time_ms"] == "1.250"
    assert rows[0]["stereo_time_ms"] == "10.500"
    assert rows[0]["tracking_time_ms"] == "2.750"
    assert rows[0]["total_time_ms"] == "14.500"
    assert rows[0]["accepted"] == "0"
    assert rows[0]["status"] == "too few pnp inliers"
