#!/usr/bin/env python3
"""Chi-square calibration of `aqua_sonar_loc` published pose covariance.

Background. `aqua_sonar_loc` derives a diagonal pose covariance from the
post-registration fitness score and inlier count. Translation variance per
axis is

  sigma2_xyz = clip(position_scale * fitness_score / inliers,
                    position_floor_m2,
                    position_cap_m2)

and rotation variance is

  sigma2_rpy = clip(rotation_scale * fitness_score
                      / (inliers * characteristic_range_m^2),
                    rotation_floor_rad2,
                    rotation_cap_rad2)

The model captures the *ordering* (high fitness → larger published variance)
but the absolute scales (`position_scale`, `rotation_scale`) need a per-
platform calibration loop against ground-truth pose error to produce
statistically consistent uncertainty estimates.

This script computes the calibration. Given a bag containing both the
sonar-estimate odometry (with the fitness-derived covariance) and a
reference odometry topic, it:

  1. Time-aligns the two streams.
  2. Computes per-step translation error:        e = p_est - p_ref
  3. Reads the diagonal translation variance from the published covariance.
  4. Computes the squared Mahalanobis distance per sample,
       d2 = e_x^2 / sigma2_x + e_y^2 / sigma2_y + e_z^2 / sigma2_z.
  5. Under correct calibration d2 follows chi-square with 3 dof; the 95th
     percentile target is ~7.815.
  6. Suggests a scaling factor that brings the observed 95th percentile to
     7.815, plus an alternative factor that makes the mean(d2) hit the
     expected value of 3.0.

Usage:

  ./aqua_localization/scripts/calibrate_sonar_covariance.py \\
    --bag aqua_localization/datasets/public/mbes_slam/demo_with_estimate \\
    --reference-topic /nav/processed/odometry \\
    --estimate-topic /aqua_sonar_loc/odometry

The script does NOT modify any YAML — it prints a recommended factor that
you multiply into your existing `scan_matching.covariance.position_scale`
and `rotation_scale`.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

try:
    import numpy as np
    from rosbags.highlevel import AnyReader
except ImportError as e:
    sys.stderr.write(f"missing dependency: {e}\n")
    raise


# Chi-square 95th percentile for k degrees of freedom (translation = 3).
CHI2_95_DOF3 = 7.81472790325118


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True,
                        help="rosbag2 directory or .mcap file")
    parser.add_argument("--reference-topic", default="/nav/processed/odometry",
                        help="Ground-truth (or reference) odometry topic")
    parser.add_argument("--estimate-topic", default="/aqua_sonar_loc/odometry",
                        help="aqua_sonar_loc odometry topic carrying the "
                             "fitness-derived covariance")
    parser.add_argument("--max-time-diff-s", type=float, default=0.5,
                        help="Reject estimate samples that have no reference "
                             "sample within this many seconds (default: 0.5)")
    parser.add_argument("--no-align", action="store_true",
                        help="Skip Umeyama SE(3) alignment of estimate onto "
                             "reference. The default applies alignment so the "
                             "residuals reflect calibration of *registration "
                             "step* uncertainty (rather than absolute frame "
                             "offset) when the sonar estimate is in its own "
                             "first-pose-origin frame.")
    return parser.parse_args()


def umeyama_se3(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid SE(3) alignment: returns (R, t) so dst ≈ R @ src.T + t."""
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    cov = ((dst - dst_mean).T @ (src - src_mean)) / src.shape[0]
    U, _, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    R = U @ S @ Vt
    t = dst_mean - R @ src_mean
    return R, t


def header_seconds(msg) -> float:
    s = msg.header.stamp
    return float(s.sec) + float(s.nanosec) * 1e-9


def odometry_position(msg) -> np.ndarray:
    p = msg.pose.pose.position
    return np.array([p.x, p.y, p.z], dtype=np.float64)


def covariance_diag_translation(msg) -> np.ndarray:
    """Return the (sigma2_x, sigma2_y, sigma2_z) diagonal from the 6x6 pose
    covariance flat array in row-major order."""
    cov = msg.pose.covariance
    return np.array([cov[0], cov[7], cov[14]], dtype=np.float64)


def main() -> int:
    args = parse_args()
    bag_dir = args.bag if args.bag.is_dir() else args.bag.parent
    if not bag_dir.is_dir():
        sys.stderr.write(f"not a rosbag2 directory: {bag_dir}\n")
        return 1

    ref_buf: list[tuple[float, np.ndarray]] = []
    est_buf: list[tuple[float, np.ndarray, np.ndarray]] = []

    targets = {args.reference_topic, args.estimate_topic}
    with AnyReader([bag_dir]) as reader:
        wanted = [c for c in reader.connections if c.topic in targets]
        if len(wanted) < 2:
            sys.stderr.write(
                f"missing topics. expected both {targets}, got "
                f"{[c.topic for c in wanted]}\n"
            )
            return 2
        for connection, _t_ns, raw in reader.messages(connections=wanted):
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception:
                continue
            t = header_seconds(msg)
            if connection.topic == args.reference_topic:
                ref_buf.append((t, odometry_position(msg)))
            elif connection.topic == args.estimate_topic:
                est_buf.append(
                    (t, odometry_position(msg), covariance_diag_translation(msg))
                )

    if not ref_buf or not est_buf:
        sys.stderr.write(
            f"empty buffers: ref={len(ref_buf)} est={len(est_buf)}\n"
        )
        return 3

    print(f"reference samples: {len(ref_buf)}")
    print(f"estimate samples : {len(est_buf)}")

    ref_t = np.array([t for t, _ in ref_buf])
    ref_xyz = np.array([p for _, p in ref_buf])

    # Linear interpolation of reference position at each estimate timestamp,
    # rejecting samples that fall outside the reference time range or that
    # have a nearest neighbour beyond max_time_diff_s on either side.
    errors = []
    sigmas = []
    skipped_out_of_range = 0
    skipped_far_neighbour = 0
    for t, p_est, sigma2 in est_buf:
        if t < ref_t[0] or t > ref_t[-1]:
            skipped_out_of_range += 1
            continue
        idx = int(np.searchsorted(ref_t, t))
        idx_prev = max(0, idx - 1)
        idx_next = min(len(ref_t) - 1, idx)
        if (t - ref_t[idx_prev] > args.max_time_diff_s
                and ref_t[idx_next] - t > args.max_time_diff_s):
            skipped_far_neighbour += 1
            continue
        if ref_t[idx_next] == ref_t[idx_prev]:
            p_ref = ref_xyz[idx_prev]
        else:
            a = (t - ref_t[idx_prev]) / (ref_t[idx_next] - ref_t[idx_prev])
            p_ref = (1 - a) * ref_xyz[idx_prev] + a * ref_xyz[idx_next]
        errors.append(p_est - p_ref)
        sigmas.append(sigma2)

    if skipped_out_of_range:
        print(f"  skipped (outside ref time range): {skipped_out_of_range}")
    if skipped_far_neighbour:
        print(f"  skipped (no neighbour within {args.max_time_diff_s} s): {skipped_far_neighbour}")

    if not errors:
        sys.stderr.write("no overlapping estimate samples after time filtering\n")
        return 4

    errors_arr = np.asarray(errors)               # (N, 3) - raw est minus ref
    sigma2_arr = np.asarray(sigmas)               # (N, 3)
    n = errors_arr.shape[0]
    raw_err_norm = np.linalg.norm(errors_arr, axis=1)

    if not args.no_align and n >= 3:
        # The sonar estimate is in its own first-pose origin frame; the
        # reference is in (potentially) a UTM-style world frame. Without
        # Umeyama alignment the residual is dominated by frame offset, not
        # by registration uncertainty. We solve `aligned_est = R @ est + t`
        # and recompute residuals; the published covariance is rotation-
        # invariant under R because we keep only its diagonal.
        # We need both raw estimate positions and the reference samples we
        # paired them with — recover them from the time loop above.
        est_xyz_paired = np.array(
            [p_est for t, p_est, _ in est_buf
             if (t >= ref_t[0] and t <= ref_t[-1])
             and (t - ref_t[max(0, int(np.searchsorted(ref_t, t)) - 1)]
                  <= args.max_time_diff_s
                  or ref_t[min(len(ref_t) - 1, int(np.searchsorted(ref_t, t)))]
                  - t <= args.max_time_diff_s)]
        )
        # Recompute paired reference xyz exactly the same way as before, so
        # the alignment is consistent with the residuals we logged.
        ref_xyz_paired = est_xyz_paired - errors_arr
        R_align, t_align = umeyama_se3(est_xyz_paired, ref_xyz_paired)
        aligned = est_xyz_paired @ R_align.T + t_align
        errors_arr = aligned - ref_xyz_paired
        # Per-axis variance is unchanged by rotation only when the rotation
        # is small or when we keep a diagonal covariance approximation.
        # Apply R to the error vector instead (equivalent: rotate residuals
        # back into the estimate frame for chi-square).
        errors_in_est_frame = errors_arr @ R_align  # R^T applied
        errors_arr = errors_in_est_frame
        print(
            f"applied Umeyama SE(3) alignment of estimate onto reference: "
            f"|t|={np.linalg.norm(t_align):.3f} m"
        )

    abs_err_norm = np.linalg.norm(errors_arr, axis=1)
    if not args.no_align:
        print(
            f"raw |error| (m) before alignment: "
            f"mean={raw_err_norm.mean():.3f} max={raw_err_norm.max():.3f}"
        )

    # Per-axis components (chi-square 3 dof on diagonal).
    safe_sigma2 = np.maximum(sigma2_arr, 1e-12)
    d2 = (errors_arr ** 2 / safe_sigma2).sum(axis=1)

    print(f"matched samples : {n}")
    print(
        f"|error| (m): mean={abs_err_norm.mean():.3f} "
        f"median={np.median(abs_err_norm):.3f} "
        f"max={abs_err_norm.max():.3f}"
    )
    print(
        f"published sigma_x (m): "
        f"mean={np.sqrt(sigma2_arr[:, 0]).mean():.3f} "
        f"median={np.sqrt(np.median(sigma2_arr[:, 0])):.3f}"
    )
    print(
        f"Mahalanobis^2 (chi-square 3 dof): "
        f"mean={d2.mean():.3f} (target 3.0)  "
        f"95th={np.quantile(d2, 0.95):.3f} (target {CHI2_95_DOF3:.3f})"
    )
    in_2sigma = float(np.mean(d2 <= CHI2_95_DOF3))
    print(
        f"  fraction with d^2 <= chi2_0.95: {in_2sigma * 100:.1f}% "
        f"(target ~95%)"
    )

    # Recommended scaling factor: covariance scales linearly with
    # `position_scale`. d2 = e^2 / sigma2 = e^2 / (k * sigma2_old) when scale
    # multiplies. So if observed 95th percentile is q, the corrective factor
    # is q / CHI2_95_DOF3 — multiply your `position_scale` by that.
    if d2.size >= 5:
        observed_95 = float(np.quantile(d2, 0.95))
        suggested_factor_95 = observed_95 / CHI2_95_DOF3
        observed_mean = float(d2.mean())
        suggested_factor_mean = observed_mean / 3.0
        print()
        print("Suggested adjustments to scan_matching.covariance.position_scale:")
        print(
            f"  match 95th percentile to chi2_0.95: "
            f"multiply current scale by {suggested_factor_95:.3f}"
        )
        print(
            f"  match mean to chi-square 3 expected value: "
            f"multiply by {suggested_factor_mean:.3f}"
        )
        print(
            "  (rotation_scale receives the same adjustment if rotation "
            "covariance is also under-/over-confident — extend this script "
            "with orientation residuals when sonar yaw error is non-trivial.)"
        )

    if any((sigma2_arr == sigma2_arr[0, 0]).all() for _ in range(1)):
        # Quick sanity print: are the sigmas at floor or cap most of the time?
        position_floor_count = int(
            (sigma2_arr == sigma2_arr.min()).all(axis=1).sum())
        position_cap_count = int(
            (sigma2_arr == sigma2_arr.max()).all(axis=1).sum())
        print()
        print(
            f"sigma2 distribution: at floor {position_floor_count} samples, "
            f"at cap {position_cap_count} samples — if either dominates the "
            f"calibration is near-degenerate, loosen the floor/cap before "
            f"re-running."
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
