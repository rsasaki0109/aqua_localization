"""Tests for run_tank_visual_fusion_benchmark.py."""

import importlib.util
import sys
from pathlib import Path


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "run_tank_visual_fusion_benchmark.py"
)
SCRIPTS_DIR = SCRIPT_PATH.parent


def load_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    spec = importlib.util.spec_from_file_location("run_tank_visual_fusion_benchmark", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_default_paths_use_sequence_stem(tmp_path):
    module = load_module()

    paths = module.default_paths(tmp_path, "short/test visual")

    assert paths.fused_tum == tmp_path / "short_test_visual_visual_fused.tum"
    assert paths.visual_status_csv == tmp_path / "short_test_visual_visual_status.csv"
    assert paths.visual_coverage_report == tmp_path / "short_test_visual_visual_coverage.md"
    assert paths.benchmark_row == tmp_path / "short_test_visual_visual_fusion_benchmark.md"
    assert paths.replay_script == tmp_path / "short_test_visual_visual_fusion_replay.sh"
    assert paths.visual_log == tmp_path / "short_test_visual_visual_frontend.log"
    assert paths.imu_log == tmp_path / "short_test_visual_imu_loc.log"
    assert paths.record_log == tmp_path / "short_test_visual_record_odometry.log"
    assert paths.bag_play_log == tmp_path / "short_test_visual_bag_play.log"


def test_build_commands_wire_visual_topic_and_extrinsics(tmp_path):
    module = load_module()
    args = module.parse_args([
        "--bag",
        "/tmp/tank_bag",
        "--reference",
        "/tmp/ref.tum",
        "--out-dir",
        str(tmp_path),
        "--imu-params",
        "/tmp/tank_dataset.yaml",
        "--visual-odom-topic",
        "/visual/fusion/odometry",
        "--fused-odom-topic",
        "/fusion/output",
        "--translation-scale",
        "0.169623465",
        "--visual-position-variance-floor",
        "0.01",
        "--visual-max-age-s",
        "0.25",
        "--base-from-camera-x-m",
        "-0.25",
        "--base-from-camera-y-m",
        "-0.45",
        "--max-stereo-descriptor-distance",
        "64",
        "--max-temporal-descriptor-distance",
        "64",
        "--orb-n-features",
        "700",
        "--orb-fast-threshold",
        "16",
        "--opencv-threads",
        "2",
    ])
    paths = module.default_paths(tmp_path, args.sequence)

    visual, imu, recorder, bag = module.build_commands(args, paths)

    assert "topics.odometry:=/visual/fusion/odometry" in visual
    assert "tracking.translation_scale:=0.169623465" in visual
    assert "extrinsics.base_from_camera.x_m:=-0.25" in visual
    assert "extrinsics.base_from_camera.y_m:=-0.45" in visual
    assert "matching.max_stereo_descriptor_distance:=64.0" in visual
    assert "orb.n_features:=700" in visual
    assert "orb.fast_threshold:=16" in visual
    assert "opencv.threads:=2" in visual
    assert "--params-file" in imu
    assert "/tmp/tank_dataset.yaml" in imu
    assert "topics.odometry:=/fusion/output" in imu
    assert "topics.visual_odometry:=/visual/fusion/odometry" in imu
    assert "imu.visual.position_variance_floor:=0.01" in imu
    assert "imu.visual.max_age_s:=0.25" in imu
    assert args.post_play_delay == 2.0
    assert args.visual_ready_timeout == 10.0
    assert args.visual_ready_poll_s == 0.1
    assert recorder[:3] == ["ros2", "run", "aqua_localization"]
    assert str(paths.fused_tum) in recorder
    assert bag == ["ros2", "bag", "play", "/tmp/tank_bag", "--clock"]


def test_visual_readiness_detects_status_csv_header(tmp_path):
    module = load_module()
    paths = module.default_paths(tmp_path, "short_test")
    paths.visual_status_csv.write_text(
        "timestamp,frame_index,accepted_count,rejected_count\n",
        encoding="utf-8",
    )

    assert module.visual_status_csv_ready(paths.visual_status_csv)

    class Process:
        def poll(self):
            return None

    readiness = module.wait_for_visual_frontend_ready(
        paths, Process(), timeout_s=1.0, poll_s=0.01
    )

    assert readiness.source.startswith("status csv")
    assert readiness.elapsed_s >= 0.0


def test_visual_readiness_detects_started_log(tmp_path):
    module = load_module()
    paths = module.default_paths(tmp_path, "short_test")
    paths.visual_log.write_text(
        "some line\nstereo visual odometry started: left=/left right=/right\n",
        encoding="utf-8",
    )

    assert module.visual_log_ready(paths.visual_log)

    class Process:
        def poll(self):
            return None

    readiness = module.wait_for_visual_frontend_ready(
        paths, Process(), timeout_s=1.0, poll_s=0.01
    )

    assert readiness.source.startswith("log marker")


def test_visual_readiness_reports_early_process_exit(tmp_path):
    module = load_module()
    paths = module.default_paths(tmp_path, "short_test")

    class Process:
        def poll(self):
            return 2

    try:
        module.wait_for_visual_frontend_ready(paths, Process(), timeout_s=1.0, poll_s=0.01)
    except RuntimeError as exc:
        assert "exited before readiness gate" in str(exc)
    else:
        raise AssertionError("expected RuntimeError")


def test_clear_stale_run_outputs_removes_readiness_and_estimate_files(tmp_path):
    module = load_module()
    paths = module.default_paths(tmp_path, "short_test")
    paths.visual_status_csv.write_text("timestamp,frame_index,\n", encoding="utf-8")
    paths.fused_tum.write_text("1 0 0 0 0 0 0 1\n", encoding="utf-8")
    paths.benchmark_row.write_text("keep\n", encoding="utf-8")

    module.clear_stale_run_outputs(paths)

    assert not paths.visual_status_csv.exists()
    assert not paths.fused_tum.exists()
    assert paths.benchmark_row.exists()


def test_visual_calibration_profile_supplies_defaults(tmp_path):
    module = load_module()
    profile = tmp_path / "visual_profile.yaml"
    profile.write_text(
        """
name: short_test_diag
frontend:
  translation_scale: 0.2
  max_stereo_descriptor_distance: 64
  max_temporal_descriptor_distance: 72
  orb_n_features: 700
  orb_fast_threshold: 16
  opencv_threads: 2
base_from_camera:
  x_m: -0.25
  y_m: -0.45
  z_m: 0.0
  roll_rad: 0.0
  pitch_rad: 0.0
  yaw_rad: 0.0
fusion:
  visual_position_variance_floor: 0.01
""",
        encoding="utf-8",
    )

    args = module.parse_args([
        "--visual-calibration-profile",
        str(profile),
        "--bag",
        "/tmp/tank_bag",
        "--reference",
        "/tmp/ref.tum",
    ])
    paths = module.default_paths(tmp_path, args.sequence)
    visual, imu, _recorder, _bag = module.build_commands(args, paths)

    assert args.translation_scale == 0.2
    assert args.base_from_camera_x_m == -0.25
    assert args.orb_n_features == 700
    assert "tracking.translation_scale:=0.2" in visual
    assert "matching.max_temporal_descriptor_distance:=72" in visual
    assert "imu.visual.position_variance_floor:=0.01" in imu


def test_visual_coverage_counts_status_rows_and_formats_note(tmp_path):
    module = load_module()
    status_csv = tmp_path / "visual_status.csv"
    status_csv.write_text(
        "timestamp,frame_index,accepted,status\n"
        "1.0,0,1,accepted\n"
        "2.0,1,1,accepted\n"
        "3.0,2,0,pnp failed\n",
        encoding="utf-8",
    )

    assert module.count_visual_status_rows(status_csv) == 3

    coverage = module.VisualCoverage(processed_frames=3, expected_frames=4, min_coverage=0.8)

    assert coverage.ratio == 0.75
    assert coverage.below_gate
    assert module.format_visual_coverage_note(coverage) == (
        "visual coverage=3/4 (75.0%), below 80.0% gate"
    )
    report = module.format_visual_coverage_report(coverage, status_csv)
    assert "coverage: 3/4 (75.0%)" in report
    assert "WARNING" in report


def test_visual_coverage_without_expected_frames_reports_processed_only(tmp_path):
    module = load_module()
    coverage = module.VisualCoverage(processed_frames=273, expected_frames=None, min_coverage=0.98)

    assert coverage.ratio is None
    assert not coverage.below_gate
    assert module.format_visual_coverage_note(coverage) == "visual frames=273"
    assert "expected visual frames" not in module.format_visual_coverage_report(
        coverage, tmp_path / "status.csv"
    )


def test_visual_coverage_report_includes_processing_timing(tmp_path):
    module = load_module()
    status_csv = tmp_path / "visual_status.csv"
    status_csv.write_text(
        "timestamp,frame_index,accepted,status,total_time_ms,stereo_time_ms,"
        "tracking_time_ms,decode_time_ms\n"
        "1.0,0,1,accepted,10.0,7.0,2.0,1.0\n"
        "2.0,1,1,accepted,20.0,14.0,4.0,2.0\n",
        encoding="utf-8",
    )
    coverage = module.VisualCoverage(processed_frames=2, expected_frames=2, min_coverage=0.98)

    report = module.format_visual_coverage_report(coverage, status_csv)

    assert "## Processing Time" in report
    assert "| total | 2 | 15.000 | 19.500 | 20.000 |" in report


def test_rejects_non_positive_visual_variance():
    module = load_module()

    try:
        module.main([
            "--bag",
            "/tmp/tank_bag",
            "--reference",
            "/tmp/ref.tum",
            "--visual-position-variance-floor",
            "0.0",
        ])
    except ValueError as exc:
        assert "visual-position-variance-floor" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_rejects_non_positive_visual_max_age():
    module = load_module()

    try:
        module.main([
            "--bag",
            "/tmp/tank_bag",
            "--reference",
            "/tmp/ref.tum",
            "--visual-max-age-s",
            "0.0",
        ])
    except ValueError as exc:
        assert "visual-max-age-s" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_rejects_invalid_visual_coverage_gate():
    module = load_module()

    try:
        module.main([
            "--bag",
            "/tmp/tank_bag",
            "--reference",
            "/tmp/ref.tum",
            "--min-visual-coverage",
            "1.5",
        ])
    except ValueError as exc:
        assert "min-visual-coverage" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_rejects_invalid_visual_speed_options():
    module = load_module()

    try:
        module.main([
            "--bag",
            "/tmp/tank_bag",
            "--reference",
            "/tmp/ref.tum",
            "--orb-n-features",
            "0",
        ])
    except ValueError as exc:
        assert "orb-n-features" in str(exc)
    else:
        raise AssertionError("expected ValueError")
