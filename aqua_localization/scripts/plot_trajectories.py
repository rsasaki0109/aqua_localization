#!/usr/bin/env python3
"""Plot a reference and estimate TUM trajectory side by side and save to PNG.

Reuses the loading and Umeyama alignment helpers from compare_trajectories.py so the
same alignment that produces the numerical APE also drives the visualization.

The output figure has two panels:

- left:  XY top-down view (reference vs estimate after rigid SE(3) alignment by default)
- right: Z (depth) over time

Use this script to generate `docs/media/public_demo_thumbnail.png` or any other
trajectory comparison thumbnail without an evo dependency.
"""

import argparse
import importlib.util
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


SCRIPTS_DIR = Path(__file__).resolve().parent


def load_compare_module():
    """Import compare_trajectories.py as a sibling module."""
    spec = importlib.util.spec_from_file_location(
        "compare_trajectories", SCRIPTS_DIR / "compare_trajectories.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def align_estimate_to_reference(
    reference: np.ndarray, estimate: np.ndarray, with_scale: bool, no_align: bool
):
    compare = load_compare_module()
    ref_at_est = compare.interpolate_positions(reference, estimate[:, 0])
    valid = ~np.isnan(ref_at_est).any(axis=1)
    if not np.any(valid):
        raise ValueError("no overlapping timestamps between reference and estimate")

    est_xyz = estimate[valid, 1:4]
    ref_xyz = ref_at_est[valid]
    if no_align:
        R = np.eye(3)
        t = np.zeros(3)
        scale = 1.0
    else:
        R, t, scale = compare.umeyama_alignment(est_xyz, ref_xyz, with_scale)

    aligned = compare.apply_transform(est_xyz, R, t, scale)
    timestamps = estimate[valid, 0]
    return timestamps, aligned, ref_xyz


def _xy_limits(reference_xyz: np.ndarray, margin_factor: float = 0.4):
    xs = reference_xyz[:, 0]
    ys = reference_xyz[:, 1]
    span_x = float(xs.max() - xs.min())
    span_y = float(ys.max() - ys.min())
    margin = max(span_x, span_y, 1.0) * margin_factor
    return (
        (float(xs.min()) - margin, float(xs.max()) + margin),
        (float(ys.min()) - margin, float(ys.max()) + margin),
    )


def render_figure(
    timestamps: np.ndarray,
    estimate_aligned: np.ndarray,
    reference_xyz: np.ndarray,
    title: str,
    out_path: Path,
    dpi: int,
    width_in: float,
    height_in: float,
    estimate_label: str = "aqua_imu_loc",
):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(width_in, height_in), dpi=dpi)
    fig.suptitle(title, fontsize=14)

    ax_xy_ref = axes[0]
    ax_xy_ref.plot(reference_xyz[:, 0], reference_xyz[:, 1], color="#1a7", linewidth=2.0,
                   label="reference (baseline.tum)")
    ax_xy_ref.plot(estimate_aligned[:, 0], estimate_aligned[:, 1], color="#d33", linewidth=1.0,
                   linestyle="--", label=f"{estimate_label} (aligned)")
    ax_xy_ref.set_aspect("equal", adjustable="box")
    xlim, ylim = _xy_limits(reference_xyz)
    ax_xy_ref.set_xlim(*xlim)
    ax_xy_ref.set_ylim(*ylim)
    ax_xy_ref.set_xlabel("x [m]")
    ax_xy_ref.set_ylabel("y [m]")
    ax_xy_ref.set_title("XY (reference scale)")
    ax_xy_ref.grid(True, alpha=0.3)
    ax_xy_ref.legend(loc="best", fontsize=9)

    ax_xy_full = axes[1]
    ax_xy_full.plot(reference_xyz[:, 0], reference_xyz[:, 1], color="#1a7", linewidth=2.0,
                    label="reference")
    ax_xy_full.plot(estimate_aligned[:, 0], estimate_aligned[:, 1], color="#d33", linewidth=1.0,
                    linestyle="--", label=f"{estimate_label} (aligned)")
    ax_xy_full.set_aspect("equal", adjustable="box")
    ax_xy_full.set_xlabel("x [m]")
    ax_xy_full.set_ylabel("y [m]")
    ax_xy_full.set_title("XY (full estimate scale)")
    ax_xy_full.grid(True, alpha=0.3)
    ax_xy_full.legend(loc="best", fontsize=9)

    ax_z = axes[2]
    t0 = float(timestamps[0])
    ax_z.plot(reference_xyz[:, 2], color="#1a7", linewidth=1.6, label="reference z")
    ax_z.plot(estimate_aligned[:, 2], color="#d33", linewidth=1.0, linestyle="--",
              label=f"{estimate_label} z (aligned)")
    ax_z.set_xlabel(f"sample index (t0 = {t0:.0f} s)")
    ax_z.set_ylabel("z [m]")
    ax_z.set_title("Depth (z) over samples")
    ax_z.grid(True, alpha=0.3)
    ax_z.legend(loc="best", fontsize=9)

    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))
    fig.savefig(out_path)
    plt.close(fig)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Plot a reference and estimate TUM trajectory and save to PNG."
    )
    parser.add_argument("reference", type=Path, help="Reference TUM file.")
    parser.add_argument("estimate", type=Path, help="Estimate TUM file.")
    parser.add_argument("--out", type=Path, required=True,
                        help="Output PNG path. Parent dirs are created.")
    parser.add_argument("--title", default="aqua_localization vs baseline")
    parser.add_argument("--estimate-label", default="aqua_imu_loc",
                        help="Legend label for the estimate trajectory.")
    parser.add_argument("--scale", action="store_true",
                        help="Use Sim(3) (with-scale) alignment.")
    parser.add_argument("--no-align", action="store_true",
                        help="Skip Umeyama alignment.")
    parser.add_argument("--dpi", type=int, default=130)
    parser.add_argument("--width-in", type=float, default=14.5)
    parser.add_argument("--height-in", type=float, default=4.5)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    compare = load_compare_module()
    reference = compare.load_tum(args.reference)
    estimate = compare.load_tum(args.estimate)
    timestamps, aligned, ref_xyz = align_estimate_to_reference(
        reference, estimate, args.scale, args.no_align
    )
    render_figure(
        timestamps, aligned, ref_xyz, args.title, args.out,
        args.dpi, args.width_in, args.height_in,
        estimate_label=args.estimate_label,
    )
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
