#!/usr/bin/env python3
"""Calibrate visual-odometry scale on one TUM pair and validate on another.

This is the paper-safe companion to `calibrate_visual_scale.py`: one sequence is
used only to estimate `tracking.translation_scale`, and a separate held-out
sequence is scaled and evaluated with rigid SE(3) alignment.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

import calibrate_visual_scale
import compare_trajectories


def scaled_estimate_array(estimate: np.ndarray, factor: float) -> np.ndarray:
    if factor <= 0.0:
        raise ValueError("scale factor must be positive")
    out = estimate.copy()
    out[:, 1:4] *= factor
    return out


def compare_scaled_validation(
    reference_path: Path, estimate_path: Path, scale_factor: float, no_align: bool = False) -> dict:
    ref = compare_trajectories.load_tum(reference_path)
    est = scaled_estimate_array(compare_trajectories.load_tum(estimate_path), scale_factor)

    ref_at_est = compare_trajectories.interpolate_positions(ref, est[:, 0])
    valid = ~np.isnan(ref_at_est).any(axis=1)
    if not np.any(valid):
        raise ValueError("no overlapping timestamps between validation reference and estimate")

    est_xyz = est[valid, 1:4]
    ref_xyz = ref_at_est[valid]
    if no_align:
        rotation = np.eye(3)
        translation = np.zeros(3)
        alignment_scale = 1.0
    else:
        rotation, translation, alignment_scale = compare_trajectories.umeyama_alignment(
            est_xyz, ref_xyz, with_scale=False)

    aligned = compare_trajectories.apply_transform(
        est_xyz, rotation, translation, alignment_scale)
    errors = np.linalg.norm(aligned - ref_xyz, axis=1)
    stats = compare_trajectories.ape_statistics(errors)
    stats["matched_seconds"] = float(est[valid, 0].max() - est[valid, 0].min())
    stats["alignment"] = {
        "applied": not no_align,
        "with_scale": False,
        "scale": alignment_scale,
        "translation": translation.tolist(),
        "rotation_matrix": rotation.tolist(),
    }
    return stats


def markdown_row(dataset: str, sequence: str, system: str, stats: dict, note: str) -> str:
    return (
        f"| {dataset} | {sequence} | {system} | SE(3) | {stats['count']} | "
        f"{stats['matched_seconds']:.2f} | {stats['mean']:.4f} | {stats['median']:.4f} | "
        f"{stats['rmse']:.4f} | {stats['max']:.4f} | {note} |"
    )


def run_validation(args) -> dict:
    calibration = calibrate_visual_scale.estimate_scale(
        args.calibration_reference, args.calibration_estimate, args.calibration_current_scale)
    scale_factor = (
        calibration["recommended_translation_scale"] / args.validation_current_scale)
    validation = compare_scaled_validation(
        args.validation_reference, args.validation_estimate, scale_factor, args.no_align)
    return {
        "calibration": calibration,
        "validation": validation,
        "scale_factor_applied_to_validation": scale_factor,
    }


def format_report(args, result: dict) -> str:
    calibration = result["calibration"]
    validation = result["validation"]
    lines = [
        "Calibration sequence:",
        f"  samples: {calibration['matched_samples']}",
        f"  duration: {calibration['matched_seconds']:.2f} s",
        f"  recommended tracking.translation_scale: "
        f"{calibration['recommended_translation_scale']:.9f}",
        f"  calibration Sim(3) RMSE: {calibration['sim3_rmse']:.4f} m",
        "Validation sequence:",
        f"  scale factor applied to validation TUM: "
        f"{result['scale_factor_applied_to_validation']:.9f}",
        f"  samples: {validation['count']}",
        f"  duration: {validation['matched_seconds']:.2f} s",
        f"  SE(3) RMSE: {validation['rmse']:.4f} m",
        f"  SE(3) mean/median/max: "
        f"{validation['mean']:.4f} / {validation['median']:.4f} / {validation['max']:.4f} m",
    ]
    if args.markdown:
        note = (
            f"scale calibrated on {args.calibration_sequence}; "
            f"tracking.translation_scale={calibration['recommended_translation_scale']:.9f}")
        lines.append(markdown_row(
            args.dataset, args.validation_sequence, args.system, validation, note))
    if args.calibration_reference == args.validation_reference or \
       args.calibration_estimate == args.validation_estimate:
        lines.append(
            "WARNING: calibration and validation paths overlap; this is a diagnostic, not held-out validation.")
    return "\n".join(lines)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Calibrate visual scale on one TUM pair and validate on a held-out pair.")
    parser.add_argument("--calibration-reference", required=True, type=Path)
    parser.add_argument("--calibration-estimate", required=True, type=Path)
    parser.add_argument("--validation-reference", required=True, type=Path)
    parser.add_argument("--validation-estimate", required=True, type=Path)
    parser.add_argument("--calibration-current-scale", type=float, default=1.0)
    parser.add_argument("--validation-current-scale", type=float, default=1.0)
    parser.add_argument("--dataset", default="Tank Dataset")
    parser.add_argument("--calibration-sequence", default="calibration")
    parser.add_argument("--validation-sequence", default="validation")
    parser.add_argument("--system", default="aqua_visual_frontend")
    parser.add_argument("--markdown", action="store_true")
    parser.add_argument("--no-align", action="store_true")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    result = run_validation(args)
    print(format_report(args, result))
    return 0


if __name__ == "__main__":
    sys.exit(main())
