#!/usr/bin/env python3
"""Compare two TUM-format trajectories and print translation APE statistics.

The "estimate" trajectory is aligned to the "reference" trajectory using a closed-form
Umeyama solution (rigid SE(3) by default, or similarity Sim(3) with `--scale`).
Time alignment is done by linear interpolation of the estimate to the reference
timestamps after the first pose. Only timestamps that fall inside both trajectories
are used.

Pure numpy; no dependency on evo or rosbags. Suitable for NTNU `fjord_1_baseline.tum`
and the output of `record_odometry.py --format tum`.
"""

import argparse
import sys
from pathlib import Path

import numpy as np


def load_tum(path: Path) -> np.ndarray:
    """Load a TUM file as an (N, 8) array (t, tx, ty, tz, qx, qy, qz, qw)."""
    rows = []
    with path.open("r", encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                raise ValueError(f"{path}: expected 8 fields, got {len(parts)}: {line}")
            rows.append([float(p) for p in parts])
    if not rows:
        raise ValueError(f"{path}: empty trajectory")
    arr = np.asarray(rows, dtype=np.float64)
    order = np.argsort(arr[:, 0])
    return arr[order]


def interpolate_positions(traj: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    """Linearly interpolate translation columns at query timestamps.

    Returns an (M, 3) array. Out-of-range timestamps return NaN rows; callers must
    filter them out before metrics. No quaternion interpolation: APE in translation
    is the metric of interest here.
    """
    times = traj[:, 0]
    out = np.full((query_times.shape[0], 3), np.nan, dtype=np.float64)
    in_range = (query_times >= times[0]) & (query_times <= times[-1])
    if not np.any(in_range):
        return out
    qt = query_times[in_range]
    for axis in range(3):
        out[in_range, axis] = np.interp(qt, times, traj[:, 1 + axis])
    return out


def umeyama_alignment(src: np.ndarray, dst: np.ndarray, with_scale: bool):
    """Closed-form least-squares similarity transform that maps src to dst.

    Inputs are (N, 3). Returns (R, t, s) such that dst ~= s * R @ src.T + t.
    With `with_scale=False`, s is forced to 1.0 (rigid SE(3)).

    Reference: Umeyama 1991, "Least-squares estimation of transformation parameters
    between two point patterns".
    """
    if src.shape != dst.shape or src.shape[1] != 3 or src.shape[0] < 2:
        raise ValueError(f"umeyama: bad shapes src={src.shape} dst={dst.shape}")

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean
    cov = (dst_c.T @ src_c) / src.shape[0]
    U, sigma, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt
    if with_scale:
        var_src = (src_c ** 2).sum() / src.shape[0]
        scale = float((sigma * np.diag(S)).sum() / var_src) if var_src > 0 else 1.0
    else:
        scale = 1.0
    t = dst_mean - scale * R @ src_mean
    return R, t, scale


def apply_transform(points: np.ndarray, R: np.ndarray, t: np.ndarray, scale: float) -> np.ndarray:
    return scale * (points @ R.T) + t


def ape_statistics(errors_m: np.ndarray) -> dict:
    return {
        "count": int(errors_m.shape[0]),
        "mean": float(errors_m.mean()),
        "median": float(np.median(errors_m)),
        "rmse": float(np.sqrt(np.mean(errors_m ** 2))),
        "std": float(errors_m.std()),
        "min": float(errors_m.min()),
        "max": float(errors_m.max()),
    }


def compare(reference_path: Path, estimate_path: Path, with_scale: bool, no_align: bool):
    ref = load_tum(reference_path)
    est = load_tum(estimate_path)

    # Use estimate timestamps as the query, interpolated against the reference.
    ref_at_est = interpolate_positions(ref, est[:, 0])
    valid = ~np.isnan(ref_at_est).any(axis=1)
    if not np.any(valid):
        raise ValueError("no overlapping timestamps between reference and estimate")

    est_xyz = est[valid, 1:4]
    ref_xyz = ref_at_est[valid]

    if no_align:
        R = np.eye(3)
        t = np.zeros(3)
        scale = 1.0
    else:
        R, t, scale = umeyama_alignment(est_xyz, ref_xyz, with_scale)

    aligned = apply_transform(est_xyz, R, t, scale)
    errors = np.linalg.norm(aligned - ref_xyz, axis=1)
    stats = ape_statistics(errors)
    stats["alignment"] = {
        "with_scale": with_scale,
        "applied": not no_align,
        "scale": scale,
        "translation": t.tolist(),
        "rotation_matrix": R.tolist(),
    }
    stats["matched_seconds"] = float(est[valid, 0].max() - est[valid, 0].min())
    return stats, errors


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Compare two TUM trajectories and print translation APE statistics."
    )
    parser.add_argument("reference", type=Path, help="Reference TUM file (e.g., fjord_1_baseline.tum).")
    parser.add_argument("estimate", type=Path, help="Estimate TUM file (e.g., output of record_odometry.py).")
    parser.add_argument(
        "--scale",
        action="store_true",
        help="Solve for similarity (Sim(3)) with scale instead of rigid SE(3).",
    )
    parser.add_argument(
        "--no-align",
        action="store_true",
        help="Skip Umeyama alignment and compare raw positions.",
    )
    parser.add_argument(
        "--save-aligned",
        type=Path,
        default=None,
        help="Optional path to write the aligned estimate as a TUM file (positions only, identity rotations).",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    stats, errors = compare(args.reference, args.estimate, args.scale, args.no_align)

    print(f"reference: {args.reference}")
    print(f"estimate:  {args.estimate}")
    print(f"matched samples:    {stats['count']}")
    print(f"matched duration:   {stats['matched_seconds']:.2f} s")
    print(f"alignment applied:  {stats['alignment']['applied']} (scale={stats['alignment']['with_scale']})")
    print(f"alignment scale:    {stats['alignment']['scale']:.6f}")
    print("APE translation [m]:")
    print(f"  count : {stats['count']}")
    print(f"  mean  : {stats['mean']:.4f}")
    print(f"  median: {stats['median']:.4f}")
    print(f"  rmse  : {stats['rmse']:.4f}")
    print(f"  std   : {stats['std']:.4f}")
    print(f"  min   : {stats['min']:.4f}")
    print(f"  max   : {stats['max']:.4f}")

    if args.save_aligned is not None:
        ref = load_tum(args.reference)
        est = load_tum(args.estimate)
        ref_at_est = interpolate_positions(ref, est[:, 0])
        valid = ~np.isnan(ref_at_est).any(axis=1)
        if args.no_align:
            R = np.eye(3)
            t = np.zeros(3)
            scale = 1.0
        else:
            R, t, scale = umeyama_alignment(est[valid, 1:4], ref_at_est[valid], args.scale)
        aligned = apply_transform(est[valid, 1:4], R, t, scale)
        timestamps = est[valid, 0]
        args.save_aligned.parent.mkdir(parents=True, exist_ok=True)
        with args.save_aligned.open("w", encoding="utf-8") as fp:
            for ts, xyz in zip(timestamps, aligned):
                fp.write(
                    f"{ts:.9f} {xyz[0]:.9f} {xyz[1]:.9f} {xyz[2]:.9f} 0.0 0.0 0.0 1.0\n"
                )
        print(f"aligned trajectory written to: {args.save_aligned}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
