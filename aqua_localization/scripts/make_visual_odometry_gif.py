#!/usr/bin/env python3
"""Render a camera + monocular-visual-odometry GIF straight from a replay bag.

This is the generator behind the README hero. It reads a monocular underwater
camera stream out of a bag offline (no DDS), runs a small sparse optical-flow
visual odometry over the frames, and writes a two-panel animated GIF:

- left  : the (CLAHE-enhanced) underwater camera frame
- right : the visual-odometry trajectory recovered from that same camera,
          growing in step with the footage

The point is to tie the video to an *estimated trajectory* that comes from the
camera itself. On bags without DVL/visual fusion, IMU-only dead reckoning drifts
badly, whereas frame-to-frame optical flow over a textured seabed still yields a
coherent survey traverse. The odometry is monocular, so the trajectory is
scale-relative (pixel-integrated), not metric.

Typical usage (reproduces the README hero):

    ros2 run aqua_localization make_visual_odometry_gif.py \
        --bag aqua_localization/datasets/public/aqualoc/demo_with_estimate \
        --out aqua_localization/docs/media/aqualoc_harbor_hero.gif

It deliberately does not run a node or need a live pipeline: it reads the bag
directly so the artifact is reproducible anywhere the bag is available.
"""

from __future__ import annotations

import argparse
import io
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from rclpy.serialization import deserialize_message  # noqa: E402
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions  # noqa: E402
from sensor_msgs.msg import Image as RosImage  # noqa: E402

# Dark "underwater" theme for the trajectory panel.
BG = "#070d16"
PANEL = "#0c1622"
MUTE = "#6f93a3"
TRACK = "#37f0d8"
SPINE = "#26415a"


@dataclass(frozen=True)
class FlowParams:
    """Sparse optical-flow visual-odometry tuning."""

    max_corners: int = 400
    quality: float = 0.01
    min_distance: int = 8
    lk_window: int = 21
    lk_levels: int = 3
    min_tracked: int = 8

    @property
    def lk_criteria(self):
        return (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)


def frame_translation(prev: np.ndarray, cur: np.ndarray,
                      params: FlowParams) -> Optional[np.ndarray]:
    """Median image-space translation (dx, dy) of features from prev -> cur.

    Returns None when too few features survive the forward track, leaving the
    caller free to hold position rather than integrate a garbage step.
    """
    corners = cv2.goodFeaturesToTrack(
        prev, maxCorners=params.max_corners, qualityLevel=params.quality,
        minDistance=params.min_distance)
    if corners is None:
        return None
    nxt, status, _ = cv2.calcOpticalFlowPyrLK(
        prev, cur, corners, None,
        winSize=(params.lk_window, params.lk_window),
        maxLevel=params.lk_levels, criteria=params.lk_criteria)
    if nxt is None:
        return None
    good_prev = corners[status == 1]
    good_next = nxt[status == 1]
    if len(good_prev) < params.min_tracked:
        return None
    return np.median(good_next - good_prev, axis=0)


def integrate_camera_path(frames: list, params: FlowParams) -> np.ndarray:
    """Integrate per-frame flow into a camera trajectory (N, 2), scale-relative.

    The camera moves opposite to the image-space flow in x; image y points down,
    so a downward flow (content moving down) means the camera moved up-frame. A
    dropped step (None) holds the previous position.
    """
    path = [np.array([0.0, 0.0])]
    prev = frames[0]
    for cur in frames[1:]:
        flow = frame_translation(prev, cur, params)
        if flow is None:
            step = np.array([0.0, 0.0])
        else:
            step = np.array([-flow[0], flow[1]])
        path.append(path[-1] + step)
        prev = cur
    return np.asarray(path)


def read_camera_frames(bag: str, topic: str, start: int, end: int, step: int,
                       clahe) -> tuple[list, list]:
    """Read CLAHE-enhanced mono frames and their elapsed times from a bag."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    frames, times = [], []
    seen = 0
    t0 = None
    while reader.has_next():
        name, data, ts = reader.read_next()
        if name != topic:
            continue
        seen += 1
        if t0 is None:
            t0 = ts
        if seen < start or (end and seen >= end) or seen % step != 0:
            continue
        msg = deserialize_message(data, RosImage)
        buf = np.frombuffer(bytes(msg.data), np.uint8)
        if msg.encoding == "mono8":
            gray = buf.reshape(msg.height, msg.width)
        else:  # rgb8 / bgr8 -> gray
            gray = cv2.cvtColor(buf.reshape(msg.height, msg.width, 3),
                                cv2.COLOR_RGB2GRAY)
        frames.append(clahe.apply(gray))
        times.append((ts - t0) * 1e-9)
    return frames, times


def render_gif(frames: list, times: list, path: np.ndarray, args) -> int:
    pad = 0.08 * max(np.ptp(path[:, 0]), np.ptp(path[:, 1]), 1.0)
    xlim = (path[:, 0].min() - pad, path[:, 0].max() + pad)
    ylim = (path[:, 1].min() - pad, path[:, 1].max() + pad)
    disp = list(range(0, len(frames), args.display_stride))

    h, w = frames[0].shape
    r0, r1 = int(0.12 * h), int(0.92 * h)
    c0, c1 = int(0.05 * w), int(0.95 * w)

    fig, (axL, axR) = plt.subplots(
        1, 2, figsize=(args.width / args.dpi, args.height / args.dpi), dpi=args.dpi,
        gridspec_kw={"width_ratios": [1.42, 1.0]})
    fig.patch.set_facecolor(BG)
    fig.subplots_adjust(left=0.015, right=0.93, bottom=0.02, top=0.86, wspace=0.12)
    fig.suptitle(args.title, color="#e8f4fb", fontsize=12.5, y=0.955, weight="bold")

    rendered = []
    for i in disp:
        axL.clear(); axR.clear()
        axL.imshow(frames[i][r0:r1, c0:c1], cmap="gray", vmin=0, vmax=255, aspect="auto")
        axL.set_xticks([]); axL.set_yticks([])
        axL.set_title(args.left_title, color=MUTE, fontsize=10, pad=4)
        for s in axL.spines.values():
            s.set_color(SPINE)

        axR.set_facecolor(PANEL)
        axR.plot(path[:i + 1, 0], path[:i + 1, 1], "-", color=TRACK, lw=2.0)
        axR.plot(path[i, 0], path[i, 1], "o", color="#eafdff", markersize=6,
                 markeredgecolor=TRACK, markeredgewidth=1.2)
        axR.scatter(path[0, 0], path[0, 1], c="#2ca02c", s=22, zorder=3)
        axR.set_xlim(*xlim); axR.set_ylim(*ylim)
        axR.set_aspect("equal")
        axR.set_xticks([]); axR.set_yticks([])
        axR.grid(True, color="#16384a", alpha=0.6)
        axR.set_title(args.right_title, color=MUTE, fontsize=10, pad=4)
        for s in axR.spines.values():
            s.set_color(SPINE)
        axR.text(0.5, -0.04, f"t = {times[i]:4.1f} s   ·   scale-relative   ·   camera-only",
                 transform=axR.transAxes, ha="center", va="top",
                 color=MUTE, fontsize=8.5, family="monospace")

        buf = io.BytesIO()
        fig.savefig(buf, format="png", facecolor=fig.get_facecolor())
        buf.seek(0)
        rendered.append(Image.open(buf).convert("RGB"))

    rendered = [f.resize((args.final_width, args.final_height), Image.LANCZOS)
                for f in rendered]
    pal = rendered[len(rendered) // 2].quantize(colors=args.colors, method=Image.MEDIANCUT)
    out_frames = []
    for f in rendered:
        g = f.quantize(palette=pal, dither=Image.NONE)
        g.info.pop("transparency", None)
        out_frames.append(g)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    out_frames[0].save(args.out, save_all=True, append_images=out_frames[1:],
                       duration=int(round(1000 / max(args.fps, 1.0))),
                       loop=0, optimize=True, disposal=1)
    print(f"VO frames {len(frames)}, GIF frames {len(out_frames)} -> {args.out}")
    return 0


def parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Render a camera + monocular visual-odometry GIF from a bag.")
    p.add_argument("--bag", required=True)
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--image-topic", default="/camera/image_raw")
    p.add_argument("--window-start", type=int, default=0,
                   help="First camera frame index of the clip (1-based).")
    p.add_argument("--window-end", type=int, default=0,
                   help="Last camera frame index (exclusive); 0 uses the whole bag.")
    p.add_argument("--vo-step", type=int, default=2,
                   help="Run VO on every Nth camera frame.")
    p.add_argument("--display-stride", type=int, default=13,
                   help="Render every Nth VO frame as a GIF frame.")
    p.add_argument("--fps", type=float, default=12.0)
    p.add_argument("--width", type=int, default=980)
    p.add_argument("--height", type=int, default=430)
    p.add_argument("--dpi", type=int, default=110)
    p.add_argument("--final-width", type=int, default=760)
    p.add_argument("--final-height", type=int, default=334)
    p.add_argument("--colors", type=int, default=64)
    p.add_argument("--clahe-clip", type=float, default=2.5)
    p.add_argument("--clahe-tiles", type=int, default=8)
    p.add_argument("--max-corners", type=int, default=400)
    p.add_argument("--quality", type=float, default=0.01)
    p.add_argument("--min-distance", type=int, default=8)
    p.add_argument("--lk-window", type=int, default=21)
    p.add_argument("--lk-levels", type=int, default=3)
    p.add_argument("--title",
                   default="aqua_localization  ·  monocular visual odometry")
    p.add_argument("--left-title", default="underwater camera")
    p.add_argument("--right-title", default="monocular visual odometry (camera-only)")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    clahe = cv2.createCLAHE(clipLimit=args.clahe_clip,
                            tileGridSize=(args.clahe_tiles, args.clahe_tiles))
    frames, times = read_camera_frames(
        args.bag, args.image_topic, args.window_start, args.window_end,
        max(1, args.vo_step), clahe)
    if len(frames) < 2:
        print("need at least 2 camera frames", file=sys.stderr)
        return 1
    params = FlowParams(
        max_corners=args.max_corners, quality=args.quality,
        min_distance=args.min_distance, lk_window=args.lk_window,
        lk_levels=args.lk_levels)
    path = integrate_camera_path(frames, params)
    return render_gif(frames, times, path, args)


if __name__ == "__main__":
    sys.exit(main())
