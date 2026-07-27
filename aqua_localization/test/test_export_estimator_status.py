"""Pure helper tests for export_estimator_status.py."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "export_estimator_status.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("export_estimator_status", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def estimator_status_msg(*, zero_stamp=False, include_counters=True):
    msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=0 if zero_stamp else 12,
                nanosec=0 if zero_stamp else 345_000_000,
            ),
            frame_id="odom",
        ),
        estimator_name="aqua_imu_loc",
        backend="additive_ukf",
        initialized=True,
        update_count=123,
        last_prediction_dt=0.01,
        position_covariance_trace=1.5,
        orientation_covariance_trace=0.25,
        status="running",
        accel_bias=[0.1, 0.2, 0.3],
        gyro_bias=[-0.1, -0.2, -0.3],
        ahrs_gyro_bias_z_enabled=True,
        ahrs_gyro_bias_z_active=False,
        ahrs_gyro_bias_z_last_observed=0.02,
    )
    if include_counters:
        msg.sonar_feedback_received = 9
        msg.sonar_feedback_applied = 7
        msg.sonar_feedback_skipped_stale = 1
        msg.sonar_feedback_skipped_nonfinite = 1
        msg.sonar_feedback_pending = 0
    return msg


def test_bag_reader_path_uses_parent_for_mcap():
    module = load_module()

    assert module.bag_reader_path(Path("/tmp/demo/out.mcap")) == Path("/tmp/demo")
    assert module.bag_reader_path(Path("/tmp/demo_ros2")) == Path("/tmp/demo_ros2")


def test_status_row_matches_recorder_columns_and_values():
    module = load_module()

    row = module.status_row(estimator_status_msg(), 99_000_000_000)

    assert list(row) == module.CSV_FIELDS
    assert row["timestamp"] == "12.345000000"
    assert row["update_count"] == 123
    assert row["accel_bias_z"] == "0.300000000"
    assert row["sonar_feedback_received"] == 9
    assert row["sonar_feedback_applied"] == 7


def test_status_row_uses_bag_timestamp_for_zero_header_stamp():
    module = load_module()

    row = module.status_row(
        estimator_status_msg(zero_stamp=True),
        99_250_000_000,
    )

    assert row["timestamp"] == "99.250000000"


def test_status_row_defaults_missing_sonar_counters_to_zero():
    module = load_module()

    row = module.status_row(
        estimator_status_msg(include_counters=False),
        99_000_000_000,
    )

    assert row["sonar_feedback_received"] == 0
    assert row["sonar_feedback_applied"] == 0
    assert row["sonar_feedback_skipped_stale"] == 0
    assert row["sonar_feedback_skipped_nonfinite"] == 0
    assert row["sonar_feedback_pending"] == 0

