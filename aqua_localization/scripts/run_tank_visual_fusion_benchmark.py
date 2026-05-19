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
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys
import time

import run_tank_visual_benchmark
import trajectory_benchmark_row


DEFAULT_VISUAL_TOPIC = "/aqua_visual_frontend/fusion_visual/odometry"
DEFAULT_FUSED_TOPIC = "/aqua_imu_loc/odometry"
DEFAULT_IMU_PARAMS = Path("install/aqua_imu_loc/share/aqua_imu_loc/config/tank_dataset.yaml")


@dataclass(frozen=True)
class FusionBenchmarkPaths:
    fused_tum: Path
    visual_status_csv: Path
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
        benchmark_row=out_dir / f"{stem}_visual_fusion_benchmark.md",
        replay_script=out_dir / f"{stem}_visual_fusion_replay.sh",
        visual_log=out_dir / f"{stem}_visual_frontend.log",
        imu_log=out_dir / f"{stem}_imu_loc.log",
        record_log=out_dir / f"{stem}_record_odometry.log",
        bag_play_log=out_dir / f"{stem}_bag_play.log",
    )


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


def make_benchmark_row(args, paths: FusionBenchmarkPaths) -> str:
    compare_module = trajectory_benchmark_row.load_compare_module()
    stats, _ = compare_module.compare(args.reference, paths.fused_tum, with_scale=False, no_align=False)
    note = (
        f"visual position update; tracking.translation_scale={args.translation_scale:.9f}; "
        f"base_from_camera=({args.base_from_camera_x_m:g},{args.base_from_camera_y_m:g},"
        f"{args.base_from_camera_z_m:g}) m; "
        f"visual variance floor={args.visual_position_variance_floor:g}"
    )
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
    row = make_benchmark_row(args, paths)
    paths.benchmark_row.write_text(row + "\n", encoding="utf-8")
    return "\n".join([
        f"fused estimate: {paths.fused_tum}",
        f"visual status csv: {paths.visual_status_csv}",
        f"benchmark row: {paths.benchmark_row}",
        "",
        row,
    ])


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Run Tank visual-aided IMU/DVL fusion and write a benchmark row."
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
    parser.add_argument("--play-rate", type=float, default=1.0)
    parser.add_argument("--startup-delay", type=float, default=1.0)
    parser.add_argument("--post-play-delay", type=float, default=2.0)
    parser.add_argument("--stop-timeout", type=float, default=5.0)
    parser.add_argument("--no-sim-time", dest="use_sim_time", action="store_false")
    parser.set_defaults(use_sim_time=True)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.translation_scale <= 0.0:
        raise ValueError("--translation-scale must be positive")
    if args.visual_position_variance_floor <= 0.0:
        raise ValueError("--visual-position-variance-floor must be positive")
    if args.play_rate <= 0.0:
        raise ValueError("--play-rate must be positive")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    paths = default_paths(args.out_dir, args.sequence)
    run_recording(args, paths)
    if not paths.fused_tum.exists():
        raise FileNotFoundError(f"fused TUM was not created: {paths.fused_tum}")
    print(evaluate(args, paths))
    return 0


if __name__ == "__main__":
    sys.exit(main())
