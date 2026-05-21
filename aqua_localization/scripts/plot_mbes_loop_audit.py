#!/usr/bin/env python3
"""Render an MBES loop audit plan-view plot from a bag and loop-status CSV."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import audit_mbes_loop_candidates as audit
import publish_mbes_loop_audit_markers as marker_helpers


PLOT_COLORS = {
    "high": "#ef3b2c",
    "medium": "#f59f00",
    "low": "#2fb344",
}


def render_audit_plot(
    keyframes: dict[int, marker_helpers.KeyframePose],
    specs: list[marker_helpers.MarkerSpec],
    out: Path,
    *,
    title: str,
    max_labels: int,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: E402

    out.parent.mkdir(parents=True, exist_ok=True)
    ordered_keyframes = [keyframes[key] for key in sorted(keyframes)]
    xs = [kf.x for kf in ordered_keyframes]
    ys = [kf.y for kf in ordered_keyframes]

    fig, ax = plt.subplots(figsize=(10.5, 8.0))
    ax.plot(xs, ys, color="#64748b", linewidth=1.2, alpha=0.75, label="pose graph")
    ax.scatter(xs[0], ys[0], s=45, color="#22c55e", label="start", zorder=4)
    ax.scatter(xs[-1], ys[-1], s=45, color="#0ea5e9", label="end", zorder=4)

    priority_seen: set[str] = set()
    for spec in specs:
        color = PLOT_COLORS.get(spec.priority, PLOT_COLORS["medium"])
        width = {"high": 2.8, "medium": 1.8, "low": 1.2}.get(spec.priority, 1.8)
        alpha = {"high": 0.95, "medium": 0.72, "low": 0.55}.get(spec.priority, 0.72)
        label = f"{spec.priority} accepted loop" if spec.priority not in priority_seen else None
        priority_seen.add(spec.priority)
        ax.plot(
            [spec.candidate_xyz[0], spec.current_xyz[0]],
            [spec.candidate_xyz[1], spec.current_xyz[1]],
            color=color,
            linewidth=width,
            alpha=alpha,
            label=label,
            zorder=3,
        )

    for spec in specs[:max_labels]:
        color = PLOT_COLORS.get(spec.priority, PLOT_COLORS["medium"])
        ax.text(
            spec.label_xyz[0],
            spec.label_xyz[1],
            f"#{spec.rank} {spec.candidate_id}->{spec.current_id}",
            color=color,
            fontsize=8.5,
            weight="bold" if spec.priority == "high" else "normal",
            bbox={
                "boxstyle": "round,pad=0.18",
                "facecolor": "white",
                "edgecolor": color,
                "alpha": 0.82,
                "linewidth": 0.8,
            },
            zorder=5,
        )

    ax.set_title(title)
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#cbd5e1", linewidth=0.6, alpha=0.75)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=180)
    plt.close(fig)


def build_specs(args: argparse.Namespace):
    rows = audit.read_loop_status_csv(args.csv)
    keyframes = marker_helpers.read_keyframe_poses(args.bag, args.keyframe_topic)
    audit_rows = audit.accepted_audit_rows(rows, args)
    specs = marker_helpers.build_marker_specs(
        audit_rows,
        keyframes,
        max_markers=args.max_markers,
        label_z_offset=args.label_z_offset,
    )
    if not specs:
        raise RuntimeError("no accepted loop markers could be built")
    return keyframes, specs


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path,
                        help="Results-included rosbag2 directory")
    parser.add_argument("--csv", required=True, type=Path,
                        help="Loop-status CSV")
    parser.add_argument("--out", required=True, type=Path,
                        help="Output plot path, usually .png or .svg")
    parser.add_argument("--title", default="MBES beach_pond accepted loop audit")
    parser.add_argument("--keyframe-topic", default="/aqua_pose_graph/keyframe")
    parser.add_argument("--max-markers", type=int, default=35)
    parser.add_argument("--max-labels", type=int, default=12)
    parser.add_argument("--label-z-offset", type=float, default=1.0)

    parser.add_argument("--max-fitness", type=float, default=2.0)
    parser.add_argument("--max-translation-m", type=float, default=5.0)
    parser.add_argument("--max-rotation-rad", type=float, default=0.5)
    parser.add_argument("--min-keyframe-separation", type=int, default=20)
    parser.add_argument("--descriptor-extent-warn", type=float, default=5.0)
    parser.add_argument("--descriptor-point-ratio-warn", type=float, default=0.5)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    try:
        keyframes, specs = build_specs(args)
        render_audit_plot(
            keyframes,
            specs,
            args.out,
            title=args.title,
            max_labels=args.max_labels,
        )
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(f"wrote MBES loop audit plot to {args.out} with {len(specs)} accepted loops")
    return 0


if __name__ == "__main__":
    sys.exit(main())
