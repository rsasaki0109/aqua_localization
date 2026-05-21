#!/usr/bin/env python3
"""Run the Tank visual-aided IMU/DVL fusion benchmark.

This orchestrates three ROS processes around one bag replay:

* ``stereo_visual_odometry.py`` publishes a calibrated visual odometry track.
* ``aqua_imu_loc`` consumes IMU, pressure, DVL, and the visual odometry topic.
* ``record_odometry.py`` records the fused ``/aqua_imu_loc/odometry`` to TUM.

The script then evaluates the fused trajectory against the AprilTag reference.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys
import time

import run_tank_visual_benchmark
import trajectory_benchmark_row
import visual_calibration_profile


DEFAULT_VISUAL_TOPIC = "/aqua_visual_frontend/fusion_visual/odometry"
DEFAULT_FUSED_TOPIC = "/aqua_imu_loc/odometry"
DEFAULT_IMU_PARAMS = Path("install/aqua_imu_loc/share/aqua_imu_loc/config/tank_dataset.yaml")


@dataclass(frozen=True)
class FusionBenchmarkPaths:
    fused_tum: Path
    visual_status_csv: Path
    visual_coverage_report: Path
    benchmark_row: Path
    replay_script: Path
    visual_log: Path
    imu_log: Path
    record_log: Path
    bag_play_log: Path


def default_paths(out_dir: Path, sequence: str) -> FusionBenchmarkPaths:
    stem = run_tank_visual_benchmark.sanitize_name(sequence)
    return FusionBenchmarkPaths(
        fused_tum=out_dir / f"{stem}_visual_fused.tum",
        visual_status_csv=out_dir / f"{stem}_visual_status.csv",
        visual_coverage_report=out_dir / f"{stem}_visual_coverage.md",
        benchmark_row=out_dir / f"{stem}_visual_fusion_benchmark.md",
        replay_script=out_dir / f"{stem}_visual_fusion_replay.sh",
        visual_log=out_dir / f"{stem}_visual_frontend.log",
        imu_log=out_dir / f"{stem}_imu_loc.log",
        record_log=out_dir / f"{stem}_record_odometry.log",
        bag_play_log=out_dir / f"{stem}_bag_play.log",
    )


@dataclass(frozen=True)
class VisualCoverage:
    processed_frames: int
    expected_frames: int | None
    min_coverage: float

    @property
    def ratio(self) -> float | None:
        if self.expected_frames is None:
            return None
        if self.expected_frames <= 0:
            return None
        return self.processed_frames / self.expected_frames

    @property
    def below_gate(self) -> bool:
        ratio = self.ratio
        return ratio is not None and ratio < self.min_coverage


def count_visual_status_rows(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open(newline="", encoding="utf-8") as fp:
        return sum(1 for _row in csv.DictReader(fp))


def read_status_float_values(path: Path, field: str) -> list[float]:
    if not path.exists():
        return []
    values = []
    with path.open(newline="", encoding="utf-8") as fp:
        for row in csv.DictReader(fp):
            value = row.get(field, "")
            if value == "":
                continue
            try:
                parsed = float(value)
            except ValueError:
                continue
            if math.isfinite(parsed):
                values.append(parsed)
    return values


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    if q <= 0.0:
        return ordered[0]
    if q >= 1.0:
        return ordered[-1]
    pos = (len(ordered) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return ordered[lo]
    alpha = pos - lo
    return ordered[lo] * (1.0 - alpha) + ordered[hi] * alpha


def status_timing_stats(path: Path) -> dict[str, dict[str, float | int]]:
    result = {}
    for field in (
        "decode_time_ms",
        "stereo_time_ms",
        "tracking_time_ms",
        "total_time_ms",
    ):
        values = read_status_float_values(path, field)
        result[field] = {
            "count": len(values),
            "mean": sum(values) / len(values) if values else math.nan,
            "p95": percentile(values, 0.95),
            "max": max(values) if values else math.nan,
        }
    return result


def format_float(value) -> str:
    value = float(value)
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.3f}"


def summarize_visual_coverage(args, paths: FusionBenchmarkPaths) -> VisualCoverage:
    expected = args.expected_visual_frames if args.expected_visual_frames > 0 else None
    return VisualCoverage(
        processed_frames=count_visual_status_rows(paths.visual_status_csv),
        expected_frames=expected,
        min_coverage=args.min_visual_coverage,
    )


def format_visual_coverage_note(coverage: VisualCoverage) -> str:
    if coverage.expected_frames is None:
        return f"visual frames={coverage.processed_frames}"
    ratio = coverage.ratio
    percent = 0.0 if ratio is None else ratio * 100.0
    note = (
        f"visual coverage={coverage.processed_frames}/{coverage.expected_frames} "
        f"({percent:.1f}%)"
    )
    if coverage.below_gate:
        note += f", below {coverage.min_coverage * 100.0:.1f}% gate"
    return note


def format_visual_coverage_report(coverage: VisualCoverage, status_csv: Path) -> str:
    timing = status_timing_stats(status_csv)
    lines = [
        "# Visual Frame Coverage",
        "",
        f"- status csv: `{status_csv}`",
        f"- processed visual frames: {coverage.processed_frames}",
    ]
    if coverage.expected_frames is not None:
        percent = 0.0 if coverage.ratio is None else coverage.ratio * 100.0
        lines.extend([
            f"- expected visual frames: {coverage.expected_frames}",
            f"- coverage: {coverage.processed_frames}/{coverage.expected_frames} ({percent:.1f}%)",
            f"- coverage gate: {coverage.min_coverage * 100.0:.1f}%",
        ])
        if coverage.below_gate:
            lines.extend([
                "",
                (
                    "WARNING: visual frame coverage is below the configured gate; "
                    "treat this benchmark run as throughput-limited."
                ),
            ])
    if any(stats["count"] for stats in timing.values()):
        lines.extend([
            "",
            "## Processing Time",
            "",
            "| Stage | Count | Mean ms | P95 ms | Max ms |",
            "|-------|------:|--------:|-------:|-------:|",
        ])
        for field, label in [
            ("decode_time_ms", "decode"),
            ("stereo_time_ms", "stereo"),
            ("tracking_time_ms", "tracking"),
            ("total_time_ms", "total"),
        ]:
            stats = timing[field]
            lines.append(
                f"| {label} | {stats['count']} | {format_float(stats['mean'])} | "
                f"{format_float(stats['p95'])} | {format_float(stats['max'])} |"
            )
    return "\n".join(lines) + "\n"


def build_visual_command(args, paths: FusionBenchmarkPaths) -> list[str]:
    visual_args = SimpleNamespace(
        use_sim_time=args.use_sim_time,
        camera_fx=args.camera_fx,
        camera_fy=args.camera_fy,
        camera_cx=args.camera_cx,
        camera_cy=args.camera_cy,
        camera_bf=args.camera_bf,
        max_stereo_descriptor_distance=args.max_stereo_descriptor_distance,
        max_temporal_descriptor_distance=args.max_temporal_descriptor_distance,
        orb_n_features=args.orb_n_features,
        orb_fast_threshold=args.orb_fast_threshold,
        opencv_threads=args.opencv_threads,
        translation_scale=args.translation_scale,
        odom_topic=args.visual_odom_topic,
        base_from_camera_x_m=args.base_from_camera_x_m,
        base_from_camera_y_m=args.base_from_camera_y_m,
        base_from_camera_z_m=args.base_from_camera_z_m,
        base_from_camera_roll_rad=args.base_from_camera_roll_rad,
        base_from_camera_pitch_rad=args.base_from_camera_pitch_rad,
        base_from_camera_yaw_rad=args.base_from_camera_yaw_rad,
        status_csv=paths.visual_status_csv,
    )
    return run_tank_visual_benchmark.build_visual_command(visual_args)


def build_imu_command(args) -> list[str]:
    command = [
        "ros2",
        "run",
        "aqua_imu_loc",
        "imu_loc_node",
        "--ros-args",
        "--params-file",
        str(args.imu_params),
    ]
    if args.use_sim_time:
        command.extend(run_tank_visual_benchmark.ros_param("use_sim_time", "true"))
    command.extend(run_tank_visual_benchmark.ros_param("topics.visual_odometry", args.visual_odom_topic))
    command.extend(
        run_tank_visual_benchmark.ros_param(
            "imu.visual.position_variance_floor", args.visual_position_variance_floor
        )
    )
    command.extend(
        run_tank_visual_benchmark.ros_param(
            "imu.visual.max_age_s", args.visual_max_age_s
        )
    )
    return command


def build_commands(args, paths: FusionBenchmarkPaths) -> list[list[str]]:
    return [
        build_visual_command(args, paths),
        build_imu_command(args),
        run_tank_visual_benchmark.build_record_command(args.fused_odom_topic, paths.fused_tum),
        run_tank_visual_benchmark.build_bag_play_command(args),
    ]


def run_recording(args, paths: FusionBenchmarkPaths):
    commands = build_commands(args, paths)
    run_tank_visual_benchmark.write_replay_script(paths.replay_script, commands)

    with paths.visual_log.open("w", encoding="utf-8") as visual_log, \
            paths.imu_log.open("w", encoding="utf-8") as imu_log, \
            paths.record_log.open("w", encoding="utf-8") as record_log, \
            paths.bag_play_log.open("w", encoding="utf-8") as bag_log:
        visual = subprocess.Popen(
            commands[0], stdout=visual_log, stderr=subprocess.STDOUT, start_new_session=True
        )
        time.sleep(args.startup_delay)
        imu = subprocess.Popen(
            commands[1], stdout=imu_log, stderr=subprocess.STDOUT, start_new_session=True
        )
        time.sleep(args.startup_delay)
        recorder = subprocess.Popen(
            commands[2], stdout=record_log, stderr=subprocess.STDOUT, start_new_session=True
        )
        time.sleep(args.startup_delay)
        try:
            subprocess.run(commands[3], check=True, stdout=bag_log, stderr=subprocess.STDOUT)
            time.sleep(args.post_play_delay)
        finally:
            run_tank_visual_benchmark.terminate_process(recorder, args.stop_timeout)
            run_tank_visual_benchmark.terminate_process(imu, args.stop_timeout)
            run_tank_visual_benchmark.terminate_process(visual, args.stop_timeout)


def make_benchmark_row(args, paths: FusionBenchmarkPaths, coverage: VisualCoverage) -> str:
    compare_module = trajectory_benchmark_row.load_compare_module()
    stats, _ = compare_module.compare(args.reference, paths.fused_tum, with_scale=False, no_align=False)
    note = (
        f"visual position update; tracking.translation_scale={args.translation_scale:.9f}; "
        f"base_from_camera=({args.base_from_camera_x_m:g},{args.base_from_camera_y_m:g},"
        f"{args.base_from_camera_z_m:g}) m; "
        f"visual variance floor={args.visual_position_variance_floor:g}; "
        f"visual max age={args.visual_max_age_s:g}; "
        f"orb_n_features={args.orb_n_features}; "
        f"orb_fast_threshold={args.orb_fast_threshold}; "
        f"opencv_threads={args.opencv_threads}; "
        f"replay rate={args.play_rate:g}; "
        f"{format_visual_coverage_note(coverage)}"
    )
    if args.visual_calibration_profile:
        label = visual_calibration_profile.profile_label(
            args.visual_calibration_profile,
            visual_calibration_profile.load_profile(args.visual_calibration_profile),
        )
        note += f"; calibration profile={label}"
    row_args = SimpleNamespace(
        dataset=args.dataset,
        sequence=args.sequence,
        system=args.system,
        note=note,
    )
    return "\n".join([
        trajectory_benchmark_row.table_header(),
        trajectory_benchmark_row.format_row(row_args, stats),
    ])


def evaluate(args, paths: FusionBenchmarkPaths) -> str:
    coverage = summarize_visual_coverage(args, paths)
    paths.visual_coverage_report.write_text(
        format_visual_coverage_report(coverage, paths.visual_status_csv), encoding="utf-8"
    )
    row = make_benchmark_row(args, paths, coverage)
    paths.benchmark_row.write_text(row + "\n", encoding="utf-8")
    lines = [
        f"fused estimate: {paths.fused_tum}",
        f"visual status csv: {paths.visual_status_csv}",
        f"visual coverage report: {paths.visual_coverage_report}",
        f"benchmark row: {paths.benchmark_row}",
        "",
        row,
    ]
    if coverage.below_gate:
        lines.extend([
            "",
            (
                "WARNING: visual frame coverage is below the configured gate; "
                "this run is throughput-limited."
            ),
        ])
    return "\n".join(lines)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run Tank visual-aided IMU/DVL fusion and write a benchmark row."
    )
    parser.add_argument(
        "--visual-calibration-profile",
        type=Path,
        default=None,
        help="YAML profile from visual_calibration_profile.py. CLI arguments override profile defaults.",
    )
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_visual_fusion"))
    parser.add_argument("--imu-params", type=Path, default=DEFAULT_IMU_PARAMS)
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--sequence", default="short_test")
    parser.add_argument("--system", default="aqua_localization+visual")
    parser.add_argument("--visual-odom-topic", default=DEFAULT_VISUAL_TOPIC)
    parser.add_argument("--fused-odom-topic", default=DEFAULT_FUSED_TOPIC)
    parser.add_argument("--translation-scale", type=float, default=1.0)
    parser.add_argument("--visual-position-variance-floor", type=float, default=0.04)
    parser.add_argument("--visual-max-age-s", type=float, default=1.0)
    parser.add_argument("--camera-fx", type=float, default=run_tank_visual_benchmark.DEFAULT_FX)
    parser.add_argument("--camera-fy", type=float, default=run_tank_visual_benchmark.DEFAULT_FY)
    parser.add_argument("--camera-cx", type=float, default=run_tank_visual_benchmark.DEFAULT_CX)
    parser.add_argument("--camera-cy", type=float, default=run_tank_visual_benchmark.DEFAULT_CY)
    parser.add_argument("--camera-bf", type=float, default=run_tank_visual_benchmark.DEFAULT_BF)
    parser.add_argument("--base-from-camera-x-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-y-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-z-m", type=float, default=0.0)
    parser.add_argument("--base-from-camera-roll-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-pitch-rad", type=float, default=0.0)
    parser.add_argument("--base-from-camera-yaw-rad", type=float, default=0.0)
    parser.add_argument("--max-stereo-descriptor-distance", type=float, default=96.0)
    parser.add_argument("--max-temporal-descriptor-distance", type=float, default=96.0)
    parser.add_argument("--orb-n-features", type=int, default=1000)
    parser.add_argument("--orb-fast-threshold", type=int, default=12)
    parser.add_argument("--opencv-threads", type=int, default=0)
    parser.add_argument("--play-rate", type=float, default=1.0)
    parser.add_argument(
        "--expected-visual-frames",
        type=int,
        default=0,
        help="Expected stereo frames for coverage reporting. Use 0 to report processed frames only.",
    )
    parser.add_argument(
        "--min-visual-coverage",
        type=float,
        default=0.98,
        help="Warn when processed/expected visual frames falls below this ratio.",
    )
    parser.add_argument("--startup-delay", type=float, default=1.0)
    parser.add_argument("--post-play-delay", type=float, default=2.0)
    parser.add_argument("--stop-timeout", type=float, default=5.0)
    parser.add_argument("--no-sim-time", dest="use_sim_time", action="store_false")
    parser.set_defaults(use_sim_time=True)
    return parser


def parse_args(argv):
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--visual-calibration-profile", type=Path, default=None)
    pre_args, _ = pre_parser.parse_known_args(argv)
    parser = build_arg_parser()
    if pre_args.visual_calibration_profile is not None:
        parser.set_defaults(
            **visual_calibration_profile.profile_arg_defaults(pre_args.visual_calibration_profile)
        )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.translation_scale <= 0.0:
        raise ValueError("--translation-scale must be positive")
    if args.visual_position_variance_floor <= 0.0:
        raise ValueError("--visual-position-variance-floor must be positive")
    if args.visual_max_age_s <= 0.0:
        raise ValueError("--visual-max-age-s must be positive")
    if args.play_rate <= 0.0:
        raise ValueError("--play-rate must be positive")
    if args.orb_n_features <= 0:
        raise ValueError("--orb-n-features must be positive")
    if args.orb_fast_threshold < 0:
        raise ValueError("--orb-fast-threshold must be non-negative")
    if args.opencv_threads < 0:
        raise ValueError("--opencv-threads must be non-negative")
    if args.expected_visual_frames < 0:
        raise ValueError("--expected-visual-frames must be non-negative")
    if not 0.0 < args.min_visual_coverage <= 1.0:
        raise ValueError("--min-visual-coverage must be in (0, 1]")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    paths = default_paths(args.out_dir, args.sequence)
    run_recording(args, paths)
    if not paths.fused_tum.exists():
        raise FileNotFoundError(f"fused TUM was not created: {paths.fused_tum}")
    print(evaluate(args, paths))
    return 0


if __name__ == "__main__":
    sys.exit(main())
