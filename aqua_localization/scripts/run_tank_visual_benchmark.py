#!/usr/bin/env python3
"""Run or evaluate the experimental Tank stereo visual benchmark.

This script ties together the Tank-style stereo visual frontend, trajectory
recording, and scale diagnostics. With ``--bag`` it records
``/aqua_visual_frontend/odometry`` from a ROS 2 bag; with ``--estimate`` it skips
recording and only evaluates an existing TUM trajectory.
"""

import argparse
from dataclasses import dataclass
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import calibrate_visual_scale
import analyze_visual_drift
import analyze_visual_motion_segments
import summarize_visual_frontend_status
import trajectory_benchmark_row


DEFAULT_FX = 655.0
DEFAULT_FY = 655.0
DEFAULT_CX = 306.0
DEFAULT_CY = 256.0
DEFAULT_BF = 78.89165891925023
DEFAULT_ODOM_TOPIC = "/aqua_visual_frontend/odometry"


@dataclass(frozen=True)
class BenchmarkPaths:
    estimate_tum: Path
    status_csv: Path
    status_summary: Path
    drift_report: Path
    motion_segments_report: Path
    scale_report: Path
    benchmark_row: Path
    replay_script: Path


def sanitize_name(value: str) -> str:
    return "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in value).strip("_") or "sequence"


def default_paths(out_dir: Path, sequence: str) -> BenchmarkPaths:
    stem = sanitize_name(sequence)
    return BenchmarkPaths(
        estimate_tum=out_dir / f"{stem}_visual_frontend.tum",
        status_csv=out_dir / f"{stem}_visual_frontend_status.csv",
        status_summary=out_dir / f"{stem}_visual_frontend_status.md",
        drift_report=out_dir / f"{stem}_visual_drift.md",
        motion_segments_report=out_dir / f"{stem}_visual_motion_segments.md",
        scale_report=out_dir / f"{stem}_visual_scale_report.txt",
        benchmark_row=out_dir / f"{stem}_visual_benchmark.md",
        replay_script=out_dir / f"{stem}_visual_replay.sh",
    )


def shell_join(command: list[str]) -> str:
    return shlex.join([str(part) for part in command])


def ros_param(name: str, value) -> list[str]:
    return ["-p", f"{name}:={value}"]


def build_visual_command(args) -> list[str]:
    command = ["ros2", "run", "aqua_localization", "stereo_visual_odometry.py", "--ros-args"]
    if args.use_sim_time:
        command.extend(ros_param("use_sim_time", "true"))
    command.extend(ros_param("camera.fx", args.camera_fx))
    command.extend(ros_param("camera.fy", args.camera_fy))
    command.extend(ros_param("camera.cx", args.camera_cx))
    command.extend(ros_param("camera.cy", args.camera_cy))
    command.extend(ros_param("camera.bf", args.camera_bf))
    command.extend(
        ros_param("matching.max_stereo_descriptor_distance", args.max_stereo_descriptor_distance)
    )
    command.extend(
        ros_param("matching.max_temporal_descriptor_distance", args.max_temporal_descriptor_distance)
    )
    command.extend(ros_param("tracking.translation_scale", args.translation_scale))
    command.extend(ros_param("topics.odometry", args.odom_topic))
    command.extend(ros_param("extrinsics.base_from_camera.x_m", args.base_from_camera_x_m))
    command.extend(ros_param("extrinsics.base_from_camera.y_m", args.base_from_camera_y_m))
    command.extend(ros_param("extrinsics.base_from_camera.z_m", args.base_from_camera_z_m))
    command.extend(ros_param("extrinsics.base_from_camera.roll_rad", args.base_from_camera_roll_rad))
    command.extend(ros_param("extrinsics.base_from_camera.pitch_rad", args.base_from_camera_pitch_rad))
    command.extend(ros_param("extrinsics.base_from_camera.yaw_rad", args.base_from_camera_yaw_rad))
    if args.status_csv:
        command.extend(ros_param("diagnostics.status_csv_path", args.status_csv))
    return command


def build_record_command(topic: str, estimate_tum: Path) -> list[str]:
    return [
        "ros2",
        "run",
        "aqua_localization",
        "record_odometry.py",
        "--topic",
        topic,
        "--out",
        str(estimate_tum),
        "--format",
        "tum",
    ]


def build_bag_play_command(args) -> list[str]:
    command = ["ros2", "bag", "play", str(args.bag)]
    if args.use_sim_time:
        command.append("--clock")
    if args.play_rate != 1.0:
        command.extend(["--rate", str(args.play_rate)])
    return command


def write_replay_script(path: Path, commands: list[list[str]]):
    text = "\n".join(["#!/usr/bin/env bash", "set -euo pipefail", ""] + [shell_join(c) for c in commands])
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text + "\n", encoding="utf-8")
    path.chmod(0o755)


def start_process(command: list[str]) -> subprocess.Popen:
    return subprocess.Popen(command, start_new_session=True)


def send_process_signal(process: subprocess.Popen, sig: signal.Signals):
    try:
        process_group = os.getpgid(process.pid)
    except ProcessLookupError:
        return
    if process_group != os.getpgrp():
        try:
            os.killpg(process_group, sig)
            return
        except ProcessLookupError:
            return
    process.send_signal(sig)


def terminate_process(process: subprocess.Popen, timeout_s: float):
    if process.poll() is not None:
        return
    send_process_signal(process, signal.SIGINT)
    try:
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        send_process_signal(process, signal.SIGTERM)
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            send_process_signal(process, signal.SIGKILL)
            process.wait(timeout=timeout_s)


def run_recording(args, paths: BenchmarkPaths):
    commands = [
        build_visual_command(args),
        build_record_command(args.odom_topic, paths.estimate_tum),
        build_bag_play_command(args),
    ]
    write_replay_script(paths.replay_script, commands)

    visual = start_process(commands[0])
    time.sleep(args.startup_delay)
    recorder = start_process(commands[1])
    time.sleep(args.startup_delay)
    try:
        subprocess.run(commands[2], check=True)
    finally:
        terminate_process(recorder, args.stop_timeout)
        terminate_process(visual, args.stop_timeout)


def make_benchmark_row(args, estimate_tum: Path, note: str) -> str:
    compare_module = trajectory_benchmark_row.load_compare_module()
    stats, _ = compare_module.compare(args.reference, estimate_tum, with_scale=False, no_align=False)
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


def evaluate(args, estimate_tum: Path, paths: BenchmarkPaths) -> str:
    result = calibrate_visual_scale.estimate_scale(
        args.reference, estimate_tum, current_scale=args.translation_scale)
    report = calibrate_visual_scale.format_report(result, ros_args=True)
    paths.scale_report.write_text(report + "\n", encoding="utf-8")

    note = (
        f"stereo ORB+PnP; current tracking.translation_scale={args.translation_scale:.9f}; "
        f"same-sequence Sim(3) scale diagnostic={result['sim3_alignment_scale']:.9f}"
    )
    row = make_benchmark_row(args, estimate_tum, note)
    paths.benchmark_row.write_text(row + "\n", encoding="utf-8")
    drift_report = analyze_visual_drift.run_analysis(SimpleNamespace(
        reference=args.reference,
        estimate=estimate_tum,
        window_s=args.drift_window_s,
        stride_s=args.drift_stride_s,
        min_samples=args.drift_min_samples,
    ))
    paths.drift_report.write_text(drift_report, encoding="utf-8")
    motion_segments_report = analyze_visual_motion_segments.run_analysis(SimpleNamespace(
        reference=args.reference,
        estimate=estimate_tum,
        segment_s=args.segment_s,
        stride_s=args.segment_stride_s,
        min_reference_motion_m=args.segment_min_reference_motion_m,
    ))
    paths.motion_segments_report.write_text(motion_segments_report, encoding="utf-8")
    status_summary_lines = []
    if args.status_csv is not None and args.status_csv.exists():
        status_summary = summarize_visual_frontend_status.format_summary_markdown(
            summarize_visual_frontend_status.summarize(
                summarize_visual_frontend_status.read_csv(args.status_csv)),
            str(args.status_csv),
        )
        paths.status_summary.write_text(status_summary, encoding="utf-8")
        status_summary_lines = [
            f"status csv: {args.status_csv}",
            f"status summary: {paths.status_summary}",
        ]
    return "\n".join([
        f"estimate: {estimate_tum}",
        f"scale report: {paths.scale_report}",
        f"benchmark row: {paths.benchmark_row}",
        f"drift report: {paths.drift_report}",
        f"motion segments report: {paths.motion_segments_report}",
        *status_summary_lines,
        "",
        report,
        "",
        row,
    ])


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Record and evaluate the experimental Tank stereo visual benchmark.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path, help="ROS 2 bag to replay and record from.")
    source.add_argument("--estimate", type=Path, help="Existing visual odometry TUM trajectory.")
    parser.add_argument("--reference", required=True, type=Path, help="Reference TUM trajectory.")
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_visual_benchmark"))
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--sequence", default="short_test")
    parser.add_argument("--system", default="aqua_visual_frontend")
    parser.add_argument("--odom-topic", default=DEFAULT_ODOM_TOPIC)
    parser.add_argument(
        "--status-csv",
        type=Path,
        default=None,
        help="Optional visual frontend diagnostics CSV. Defaults under --out-dir when --bag is used.",
    )
    parser.add_argument("--translation-scale", type=float, default=1.0)
    parser.add_argument("--drift-window-s", type=float, default=3.0)
    parser.add_argument("--drift-stride-s", type=float, default=1.0)
    parser.add_argument("--drift-min-samples", type=int, default=20)
    parser.add_argument("--segment-s", type=float, default=1.0)
    parser.add_argument("--segment-stride-s", type=float, default=0.5)
    parser.add_argument("--segment-min-reference-motion-m", type=float, default=0.01)
    parser.add_argument("--camera-fx", type=float, default=DEFAULT_FX)
    parser.add_argument("--camera-fy", type=float, default=DEFAULT_FY)
    parser.add_argument("--camera-cx", type=float, default=DEFAULT_CX)
    parser.add_argument("--camera-cy", type=float, default=DEFAULT_CY)
    parser.add_argument("--camera-bf", type=float, default=DEFAULT_BF)
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
    parser.add_argument("--stop-timeout", type=float, default=5.0)
    parser.add_argument("--no-sim-time", dest="use_sim_time", action="store_false")
    parser.set_defaults(use_sim_time=True)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if args.translation_scale <= 0.0:
        raise ValueError("--translation-scale must be positive")
    if args.play_rate <= 0.0:
        raise ValueError("--play-rate must be positive")

    paths = default_paths(args.out_dir, args.sequence)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if args.status_csv is None and args.bag is not None:
        args.status_csv = paths.status_csv

    estimate_tum = args.estimate if args.estimate is not None else paths.estimate_tum
    if args.bag is not None:
        run_recording(args, paths)
    if not estimate_tum.exists():
        raise FileNotFoundError(f"estimate TUM was not created: {estimate_tum}")
    print(evaluate(args, estimate_tum, paths))
    return 0


if __name__ == "__main__":
    sys.exit(main())
