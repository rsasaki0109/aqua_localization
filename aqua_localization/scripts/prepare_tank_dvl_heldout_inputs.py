#!/usr/bin/env python3
"""Prepare Tank held-out inputs for DVL-prior validation."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import shlex
import subprocess
import sys


DEFAULT_FX = 655.0 * 0.98
DEFAULT_FY = 655.0 * 0.98
DEFAULT_CX = 306.0
DEFAULT_CY = 256.0
DEFAULT_BF = 78.89165891925023 * 1.05


@dataclass(frozen=True)
class PrepPaths:
    ros2_bag: Path
    visual_out_dir: Path
    visual_tum: Path
    bundle_out_dir: Path
    manifest: Path


def sanitize_name(value: str) -> str:
    return "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in value).strip("_") or "sequence"


def default_benchmark_markdown() -> Path | None:
    candidate = Path(__file__).resolve().parents[2] / "docs" / "benchmarks" / "tank_aqua_slam.md"
    return candidate if candidate.exists() else None


def resolve_paths(args) -> PrepPaths:
    stem = sanitize_name(args.sequence)
    visual_out_dir = args.out_dir / "visual"
    ros2_bag = args.ros2_bag or (args.out_dir / f"{stem}_ros2_visual")
    visual_tum = args.visual or (visual_out_dir / f"{stem}_visual_frontend.tum")
    return PrepPaths(
        ros2_bag=ros2_bag,
        visual_out_dir=visual_out_dir,
        visual_tum=visual_tum,
        bundle_out_dir=args.out_dir / "validation_bundle",
        manifest=args.out_dir / "tank_dvl_heldout_inputs_manifest.md",
    )


def shell_join(command: list[str]) -> str:
    return shlex.join([str(part) for part in command])


def build_convert_command(args, paths: PrepPaths) -> list[str]:
    command = [
        "ros2",
        "run",
        "aqua_localization",
        "convert_tank_dataset_bag.py",
        "--src",
        str(args.ros1_bag),
        "--dst",
        str(paths.ros2_bag),
        "--include-cameras",
    ]
    return command


def build_visual_command(args, paths: PrepPaths) -> list[str]:
    return [
        "ros2",
        "run",
        "aqua_localization",
        "run_tank_visual_direct_benchmark.py",
        "--bag",
        str(paths.ros2_bag),
        "--reference",
        str(args.reference),
        "--out-dir",
        str(paths.visual_out_dir),
        "--sequence",
        args.sequence,
        "--system",
        args.visual_system,
        "--translation-scale",
        str(args.translation_scale),
        "--min-pnp-inliers",
        str(args.min_pnp_inliers),
        "--min-inlier-ratio",
        str(args.min_inlier_ratio),
        "--ransac-iterations",
        str(args.ransac_iterations),
        "--ransac-reprojection-error-px",
        str(args.ransac_reprojection_error_px),
        "--ransac-confidence",
        str(args.ransac_confidence),
        "--max-step-translation-m",
        str(args.max_step_translation_m),
        "--camera-fx",
        str(args.camera_fx),
        "--camera-fy",
        str(args.camera_fy),
        "--camera-cx",
        str(args.camera_cx),
        "--camera-cy",
        str(args.camera_cy),
        "--camera-bf",
        str(args.camera_bf),
        "--base-from-camera-x-m",
        str(args.base_from_camera_x_m),
        "--base-from-camera-y-m",
        str(args.base_from_camera_y_m),
        "--base-from-camera-z-m",
        str(args.base_from_camera_z_m),
        "--orb-n-features",
        str(args.orb_n_features),
        "--orb-fast-threshold",
        str(args.orb_fast_threshold),
        "--opencv-threads",
        str(args.opencv_threads),
    ]


def build_bundle_command(args, paths: PrepPaths) -> list[str]:
    command = [
        "ros2",
        "run",
        "aqua_localization",
        "run_tank_dvl_validation_bundle.py",
        "--profile",
        str(args.profile),
        "--sequence",
        args.sequence,
        "--bag",
        str(paths.ros2_bag),
        "--reference",
        str(args.reference),
        "--visual",
        str(paths.visual_tum),
        "--max-corrected-rmse-m",
        str(args.max_corrected_rmse_m),
        "--max-gap-x",
        str(args.max_gap_x),
        "--fail-on-gate-failure",
        "--out-dir",
        str(paths.bundle_out_dir),
    ]
    if args.benchmark_markdown is not None:
        command.extend(["--benchmark-markdown", str(args.benchmark_markdown)])
    if args.min_improvement_percent is not None:
        command.extend(["--min-improvement-percent", str(args.min_improvement_percent)])
    return command


def planned_commands(args, paths: PrepPaths) -> list[tuple[str, list[str]]]:
    commands = []
    if not args.skip_convert and args.ros1_bag is not None and not paths.ros2_bag.exists():
        commands.append(("convert_ros1_to_ros2", build_convert_command(args, paths)))
    if not args.skip_visual and args.visual is None:
        commands.append(("run_direct_visual", build_visual_command(args, paths)))
    if not args.skip_bundle:
        commands.append(("run_validation_bundle", build_bundle_command(args, paths)))
    return commands


def missing_inputs(args, paths: PrepPaths) -> list[str]:
    missing = []
    if not args.profile.exists():
        missing.append(f"profile: {args.profile}")
    if not args.reference.exists():
        missing.append(f"reference: {args.reference}")
    if args.benchmark_markdown is not None and not args.benchmark_markdown.exists():
        missing.append(f"benchmark_markdown: {args.benchmark_markdown}")
    if args.ros1_bag is None and not paths.ros2_bag.exists():
        missing.append(f"ros2_bag: {paths.ros2_bag}")
    if args.ros1_bag is not None and not args.ros1_bag.exists():
        missing.append(f"ros1_bag: {args.ros1_bag}")
    if args.skip_visual and args.visual is None:
        missing.append("--visual is required when --skip-visual is used")
    if args.visual is not None and not args.visual.exists():
        missing.append(f"visual: {args.visual}")
    return missing


def validate_args(args, paths: PrepPaths) -> None:
    if args.ros1_bag is None and args.ros2_bag is None:
        raise ValueError("provide --ros1-bag to convert or --ros2-bag to reuse an existing ROS 2 bag")
    if args.translation_scale <= 0.0:
        raise ValueError("--translation-scale must be positive")
    if args.min_pnp_inliers < 0:
        raise ValueError("--min-pnp-inliers must be non-negative")
    if not 0.0 <= args.min_inlier_ratio <= 1.0:
        raise ValueError("--min-inlier-ratio must be in [0, 1]")
    if args.max_step_translation_m <= 0.0:
        raise ValueError("--max-step-translation-m must be positive")
    if args.max_corrected_rmse_m <= 0.0:
        raise ValueError("--max-corrected-rmse-m must be positive")
    if args.max_gap_x <= 0.0:
        raise ValueError("--max-gap-x must be positive")
    if args.min_improvement_percent is not None and args.min_improvement_percent < 0.0:
        raise ValueError("--min-improvement-percent must be non-negative")
    if args.ros1_bag is not None and paths.ros2_bag.exists() and args.ros2_bag is None:
        # Reuse the existing default conversion rather than failing on converter's
        # destination-exists guard.
        pass
    missing = missing_inputs(args, paths)
    if missing and not args.dry_run:
        raise ValueError("missing required input(s): " + "; ".join(missing))


def run_command(command: list[str], dry_run: bool) -> None:
    print(shell_join(command))
    if not dry_run:
        subprocess.run(command, check=True)


def format_manifest(
    args,
    paths: PrepPaths,
    commands: list[tuple[str, list[str]]],
    missing: list[str] | None = None,
) -> str:
    missing = missing or []
    lines = [
        "# Tank DVL Held-Out Input Preparation",
        "",
        f"- Sequence: `{args.sequence}`",
        f"- Profile: `{args.profile}`",
        f"- Reference: `{args.reference}`",
        f"- ROS 1 bag: `{args.ros1_bag}`",
        f"- ROS 2 bag: `{paths.ros2_bag}`",
        f"- Visual TUM: `{paths.visual_tum}`",
        f"- Bundle output: `{paths.bundle_out_dir}`",
        f"- Dry run: `{args.dry_run}`",
        f"- Missing required inputs: `{len(missing)}`",
        "",
        "## Commands",
        "",
    ]
    if commands:
        for label, command in commands:
            lines.extend([f"### {label}", "", "```bash", shell_join(command), "```", ""])
    else:
        lines.append("- none")
        lines.append("")
    lines.extend(["## Missing Inputs", ""])
    if missing:
        for item in missing:
            lines.append(f"- {item}")
    else:
        lines.append("- none")
    lines.append("")
    lines.extend([
        "## Readout",
        "",
        "- Use this manifest to reproduce the held-out validation inputs.",
        "- In dry-run mode, this manifest is still written when inputs are missing so operators can see the exact blockers.",
        "- The validation bundle is paper-safe only when no same-sequence override is used.",
        "",
    ])
    return "\n".join(lines)


def parse_args(argv):
    default_markdown = default_benchmark_markdown()
    parser = argparse.ArgumentParser(
        description="Prepare Tank held-out ROS 2 bag, visual TUM, and DVL validation bundle."
    )
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--profile", required=True, type=Path)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--ros1-bag", type=Path)
    source.add_argument("--ros2-bag", type=Path)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--visual", type=Path, help="Existing visual TUM to reuse.")
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/aqua_tank_dvl_heldout_prepare"))
    parser.add_argument("--benchmark-markdown", type=Path, default=default_markdown)
    parser.add_argument("--skip-convert", action="store_true")
    parser.add_argument("--skip-visual", action="store_true")
    parser.add_argument("--skip-bundle", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--visual-system", default="aqua_visual_frontend_direct")
    parser.add_argument("--translation-scale", type=float, default=0.095)
    parser.add_argument("--min-pnp-inliers", type=int, default=12)
    parser.add_argument("--min-inlier-ratio", type=float, default=0.85)
    parser.add_argument("--ransac-iterations", type=int, default=100)
    parser.add_argument("--ransac-reprojection-error-px", type=float, default=4.0)
    parser.add_argument("--ransac-confidence", type=float, default=0.99)
    parser.add_argument("--max-step-translation-m", type=float, default=0.02)
    parser.add_argument("--camera-fx", type=float, default=DEFAULT_FX)
    parser.add_argument("--camera-fy", type=float, default=DEFAULT_FY)
    parser.add_argument("--camera-cx", type=float, default=DEFAULT_CX)
    parser.add_argument("--camera-cy", type=float, default=DEFAULT_CY)
    parser.add_argument("--camera-bf", type=float, default=DEFAULT_BF)
    parser.add_argument("--base-from-camera-x-m", type=float, default=-0.15)
    parser.add_argument("--base-from-camera-y-m", type=float, default=-0.55)
    parser.add_argument("--base-from-camera-z-m", type=float, default=0.0)
    parser.add_argument("--orb-n-features", type=int, default=1000)
    parser.add_argument("--orb-fast-threshold", type=int, default=12)
    parser.add_argument("--opencv-threads", type=int, default=2)
    parser.add_argument("--max-corrected-rmse-m", type=float, default=0.0194)
    parser.add_argument("--max-gap-x", type=float, default=1.0)
    parser.add_argument("--min-improvement-percent", type=float)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    paths = resolve_paths(args)
    try:
        validate_args(args, paths)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    args.out_dir.mkdir(parents=True, exist_ok=True)
    commands = planned_commands(args, paths)
    manifest = format_manifest(args, paths, commands, missing_inputs(args, paths))
    paths.manifest.write_text(manifest, encoding="utf-8")
    print(manifest)
    for _label, command in commands:
        run_command(command, args.dry_run)
    return 0


if __name__ == "__main__":
    sys.exit(main())
