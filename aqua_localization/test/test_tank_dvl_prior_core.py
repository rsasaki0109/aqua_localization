"""Pure-function tests for tank_dvl_prior_core.py."""

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "tank_dvl_prior_core.py"


def load_module():
    spec = importlib.util.spec_from_file_location("tank_dvl_prior_core", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def tum_row(stamp_s: float, yaw_rad: float = 0.0):
    return [
        stamp_s,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        math.sin(0.5 * yaw_rad),
        math.cos(0.5 * yaw_rad),
    ]


def test_rotate_body_velocity_by_yaw_turns_xy_velocity():
    module = load_module()
    velocity_body = np.asarray([[1.0, 0.0, 0.0]], dtype=np.float64)

    rotated = module.rotate_body_velocity_by_yaw(
        velocity_body,
        np.asarray([math.pi / 2.0], dtype=np.float64),
    )

    np.testing.assert_allclose(rotated, [[0.0, 1.0, 0.0]], atol=1.0e-12)


def test_integrate_dvl_step_body_raw_constant_velocity():
    module = load_module()
    dvl_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    dvl_velocities = np.asarray([[1.0, 0.0, 0.0]] * 3, dtype=np.float64)
    reference_tum = np.asarray([tum_row(0.0), tum_row(2.0)], dtype=np.float64)

    delta, covered, samples = module.integrate_dvl_step(
        0.5,
        1.5,
        dvl_times,
        dvl_velocities,
        reference_tum,
        "body_raw",
    )

    assert covered is True
    assert samples == 3
    np.testing.assert_allclose(delta, [1.0, 0.0, 0.0], atol=1.0e-12)


def test_integrate_dvl_step_gt_yaw_rotates_body_velocity():
    module = load_module()
    dvl_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    dvl_velocities = np.asarray([[1.0, 0.0, 0.0]] * 3, dtype=np.float64)
    reference_tum = np.asarray(
        [tum_row(0.0, math.pi / 2.0), tum_row(2.0, math.pi / 2.0)],
        dtype=np.float64,
    )

    delta, covered, samples = module.integrate_dvl_step(
        0.5,
        1.5,
        dvl_times,
        dvl_velocities,
        reference_tum,
        "gt_yaw",
    )

    assert covered is True
    assert samples == 3
    np.testing.assert_allclose(delta, [0.0, 1.0, 0.0], atol=1.0e-12)


def test_integrate_dvl_step_applies_dvl_frame_yaw_offset():
    module = load_module()
    dvl_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    dvl_velocities = np.asarray([[1.0, 0.0, 0.0]] * 3, dtype=np.float64)
    reference_tum = np.asarray([tum_row(0.0), tum_row(2.0)], dtype=np.float64)

    delta, covered, samples = module.integrate_dvl_step(
        0.5,
        1.5,
        dvl_times,
        dvl_velocities,
        reference_tum,
        "gt_yaw",
        math.pi / 2.0,
    )

    assert covered is True
    assert samples == 3
    np.testing.assert_allclose(delta, [0.0, 1.0, 0.0], atol=1.0e-12)


def test_integrate_dvl_step_uses_imu_yaw_records():
    module = load_module()
    dvl_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    dvl_velocities = np.asarray([[1.0, 0.0, 0.0]] * 3, dtype=np.float64)
    reference_tum = np.asarray([tum_row(0.0), tum_row(2.0)], dtype=np.float64)
    imu_times = np.asarray([0.0, 2.0], dtype=np.float64)
    imu_yaw = np.asarray([math.pi / 2.0, math.pi / 2.0], dtype=np.float64)

    delta, covered, samples = module.integrate_dvl_step(
        0.5,
        1.5,
        dvl_times,
        dvl_velocities,
        reference_tum,
        "imu_yaw",
        yaw_times=imu_times,
        yaw_rad=imu_yaw,
    )

    assert covered is True
    assert samples == 3
    np.testing.assert_allclose(delta, [0.0, 1.0, 0.0], atol=1.0e-12)


def test_integrate_dvl_step_combines_imu_and_dvl_yaw_offsets():
    module = load_module()
    dvl_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    dvl_velocities = np.asarray([[1.0, 0.0, 0.0]] * 3, dtype=np.float64)
    reference_tum = np.asarray([tum_row(0.0), tum_row(2.0)], dtype=np.float64)
    imu_times = np.asarray([0.0, 2.0], dtype=np.float64)
    imu_yaw = np.asarray([0.0, 0.0], dtype=np.float64)

    delta, covered, samples = module.integrate_dvl_step(
        0.5,
        1.5,
        dvl_times,
        dvl_velocities,
        reference_tum,
        "imu_yaw",
        dvl_frame_yaw_offset_rad=-math.pi / 2.0,
        yaw_times=imu_times,
        yaw_rad=imu_yaw,
        imu_yaw_offset_rad=math.pi,
    )

    assert covered is True
    assert samples == 3
    np.testing.assert_allclose(delta, [0.0, 1.0, 0.0], atol=1.0e-12)


def test_build_dvl_prior_steps_matches_reference_motion():
    module = load_module()
    records = [
        module.DvlRecord(0.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
        module.DvlRecord(1.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
        module.DvlRecord(2.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
    ]
    visual_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    reference_xyz = np.asarray(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        dtype=np.float64,
    )
    reference_tum = np.asarray([tum_row(0.0), tum_row(2.0)], dtype=np.float64)

    steps = module.build_dvl_prior_steps(
        visual_times,
        reference_xyz,
        records,
        reference_tum,
        "body_raw",
        0.0,
    )

    assert len(steps) == 2
    assert all(step.covered for step in steps)
    np.testing.assert_allclose([step.length_ratio for step in steps], [1.0, 1.0])
    np.testing.assert_allclose([step.direction_cosine for step in steps], [1.0, 1.0])
    np.testing.assert_allclose(steps[-1].dvl_cumulative_m, 2.0)
    np.testing.assert_allclose(steps[-1].reference_cumulative_m, 2.0)


def test_build_dvl_prior_steps_accepts_imu_yaw_records():
    module = load_module()
    records = [
        module.DvlRecord(0.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
        module.DvlRecord(1.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
        module.DvlRecord(2.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
    ]
    imu_records = [
        module.ImuYawRecord(0.0, math.pi / 2.0),
        module.ImuYawRecord(2.0, math.pi / 2.0),
    ]
    visual_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    reference_xyz = np.asarray(
        [[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 2.0, 0.0]],
        dtype=np.float64,
    )
    reference_tum = np.asarray([tum_row(0.0), tum_row(2.0)], dtype=np.float64)

    steps = module.build_dvl_prior_steps(
        visual_times,
        reference_xyz,
        records,
        reference_tum,
        "imu_yaw",
        0.0,
        imu_yaw_records=imu_records,
    )

    assert len(steps) == 2
    np.testing.assert_allclose([step.length_ratio for step in steps], [1.0, 1.0])
    np.testing.assert_allclose([step.direction_cosine for step in steps], [1.0, 1.0])


def test_build_dvl_prior_deltas_applies_prior_scale():
    module = load_module()
    records = [
        module.DvlRecord(0.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
        module.DvlRecord(1.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
        module.DvlRecord(2.0, np.asarray([1.0, 0.0, 0.0], dtype=np.float64)),
    ]
    visual_times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    reference_tum = np.asarray([tum_row(0.0), tum_row(2.0)], dtype=np.float64)

    deltas = module.build_dvl_prior_deltas(
        visual_times,
        records,
        reference_tum,
        "body_raw",
        prior_scale=2.0,
    )

    assert len(deltas) == 2
    assert all(delta.covered for delta in deltas)
    np.testing.assert_allclose([delta.delta_xyz for delta in deltas], [[2.0, 0.0, 0.0], [2.0, 0.0, 0.0]])


def test_positions_from_deltas_accumulates_scaled_steps():
    module = load_module()
    times = np.asarray([0.0, 1.0, 2.0], dtype=np.float64)
    deltas = [
        module.DvlPriorDelta(0.0, 1.0, np.asarray([1.0, 0.0, 0.0]), 2, True),
        module.DvlPriorDelta(1.0, 2.0, np.asarray([0.0, 2.0, 0.0]), 2, True),
    ]

    xyz = module.positions_from_deltas(times, np.asarray([10.0, 20.0, 0.0]), deltas)

    np.testing.assert_allclose(xyz, [[10.0, 20.0, 0.0], [11.0, 20.0, 0.0], [11.0, 22.0, 0.0]])


def test_stats_handles_empty_and_interpolated_percentiles():
    module = load_module()

    empty = module.stats([])
    assert empty["count"] == 0
    assert math.isnan(empty["median"])
    assert module.percentile([0.0, 10.0], 0.25) == 2.5
