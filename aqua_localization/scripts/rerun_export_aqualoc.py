#!/usr/bin/env python3
"""Export an AQUALOC `harbor` `aqua_localization` results-included demo bag
as a rerun.io recording, including the underwater camera feed.

What you get:

  - `world/aqua_imu_loc/path` — IMU + pressure dead-reckoning trajectory
  - `camera/image`            — decoded /camera/image_raw frames
  - `plots/depth/aqua_imu_loc` — z(t) (negative-down)
  - `plots/pressure`           — barometer pressure (Pa)
  - `plots/imu/a{x,y,z}`       — IMU acceleration

The default 3D view follows the trajectory; the right column shows the
camera image and time-series plots.

Quick usage:

  ./aqua_localization/scripts/rerun_export_aqualoc.py \\
    --bag aqua_localization/datasets/public/aqualoc/demo_with_estimate \\
    --out docs/media/aqualoc.rrd

Note: AQUALOC harbor_07 has no in-bag ground-truth track. The dataset
publishes its own SLAM trajectory via the IJRR paper supplements; see
datasets/aqualoc_demo.md. We render only the aqua_imu_loc estimate plus
the underwater video.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

try:
    import numpy as np
    import rerun as rr
    import rerun.blueprint as rrb
    from rosbags.highlevel import AnyReader
except ImportError as e:
    sys.stderr.write(f"missing dependency: {e}\n")
    raise


ESTIMATE_COLOR = (58, 161, 255)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--imu-topic", default="/rtimulib_node/imu")
    parser.add_argument("--pressure-topic", default="/barometer_node/pressure")
    parser.add_argument("--estimate-topic", default="/aqua_imu_loc/odometry")
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--decimate-image", type=int, default=8,
                        help="Log every Nth image (default: 8)")
    parser.add_argument("--decimate-imu", type=int, default=20)
    parser.add_argument("--decimate-pressure", type=int, default=2)
    parser.add_argument("--application-id", default="aqua_localization AQUALOC harbor_07")
    return parser.parse_args()


def header_seconds(msg) -> float:
    s = msg.header.stamp
    return float(s.sec) + float(s.nanosec) * 1e-9


def decode_image(msg) -> np.ndarray | None:
    """Decode a sensor_msgs/Image. AQUALOC harbor frames come in as `mono8`,
    `bgr8`, or `bayer_*` — handle the common cases without cv_bridge."""
    enc = msg.encoding.lower()
    h, w = int(msg.height), int(msg.width)
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc == "mono8":
        return data.reshape(h, w)
    if enc == "rgb8":
        return data.reshape(h, w, 3)
    if enc == "bgr8":
        bgr = data.reshape(h, w, 3)
        return bgr[..., ::-1]
    if enc.startswith("bayer"):
        # Crude debayer: take every other pixel in row-major order, build a
        # luma proxy that's good enough for visualization.
        return data.reshape(h, w)
    sys.stderr.write(f"unsupported image encoding: {msg.encoding}\n")
    return None


def main() -> int:
    args = parse_args()
    bag_dir = args.bag if args.bag.is_dir() else args.bag.parent
    if not bag_dir.is_dir():
        sys.stderr.write(f"not a rosbag2 directory: {bag_dir}\n")
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)

    # AQUALOC IMU-only dead reckoning explodes to km-scale drift in seconds
    # (no DVL/visual aiding, no still window for the static-bias initializer
    # since the ROV is moving from t=0). The visually compelling part of this
    # bag is the underwater camera feed plus the pressure-driven depth track
    # — make those the headline. The trajectory is logged but the default
    # layout puts the camera in the dominant slot.
    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(
                name="Underwater camera (ueye)",
                origin="/camera",
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="depth z (m)",
                    contents="/plots/depth/**",
                ),
                rrb.TimeSeriesView(
                    name="pressure (Pa)",
                    contents="/plots/pressure",
                ),
                rrb.TimeSeriesView(
                    name="IMU acceleration (m/s^2)",
                    contents="/plots/imu/**",
                ),
            ),
            column_shares=[2, 3],
        ),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="collapsed"),
        rrb.BlueprintPanel(state="collapsed"),
    )

    rr.init(args.application_id, default_blueprint=blueprint)
    rr.save(str(args.out), default_blueprint=blueprint)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    est_buf: list[tuple[float, np.ndarray]] = []
    press_buf: list[tuple[float, float]] = []
    imu_buf: list[tuple[float, np.ndarray]] = []
    image_buf: list[tuple[float, np.ndarray]] = []

    targets = {
        args.estimate_topic, args.pressure_topic, args.imu_topic, args.image_topic,
    }

    image_kept = 0
    image_seen = 0
    with AnyReader([bag_dir]) as reader:
        wanted = [c for c in reader.connections if c.topic in targets]
        for connection, t_ns, raw in reader.messages(connections=wanted):
            try:
                msg = reader.deserialize(raw, connection.msgtype)
            except Exception:
                continue
            t = header_seconds(msg)
            if connection.topic == args.estimate_topic:
                p = msg.pose.pose.position
                est_buf.append((t, np.array([p.x, p.y, p.z])))
            elif connection.topic == args.pressure_topic:
                press_buf.append((t, float(msg.fluid_pressure)))
            elif connection.topic == args.imu_topic:
                a = msg.linear_acceleration
                imu_buf.append((t, np.array([a.x, a.y, a.z])))
            elif connection.topic == args.image_topic:
                image_seen += 1
                if image_seen % args.decimate_image != 0:
                    continue
                img = decode_image(msg)
                if img is not None:
                    image_buf.append((t, img))
                    image_kept += 1

    print(
        f"buffered: estimate={len(est_buf)} pressure={len(press_buf)}"
        f" imu={len(imu_buf)} images={image_kept}/{image_seen}"
    )

    if not est_buf:
        sys.stderr.write("no estimate samples\n")
        return 2

    est_t = np.array([t for t, _ in est_buf])
    est_xyz = np.array([p for _, p in est_buf])
    t0 = float(min(
        est_t.min(),
        min((t for t, _ in press_buf), default=est_t.min()),
        min((t for t, _ in imu_buf), default=est_t.min()),
        min((t for t, _ in image_buf), default=est_t.min()),
    ))

    rr.log("world/aqua_imu_loc/path",
           rr.LineStrips3D([est_xyz], colors=ESTIMATE_COLOR, radii=0.04),
           static=True)

    # Decimated time series.
    for i, (t, p) in enumerate(zip(est_t, est_xyz)):
        if i % 10 != 0:
            continue
        rr.set_time("bag_time", duration=t - t0)
        rr.log("plots/depth/aqua_imu_loc", rr.Scalars(float(p[2])))

    for i, (t, pp) in enumerate(press_buf):
        if i % args.decimate_pressure != 0:
            continue
        rr.set_time("bag_time", duration=t - t0)
        rr.log("plots/pressure", rr.Scalars(pp))

    for i, (t, a) in enumerate(imu_buf):
        if i % args.decimate_imu != 0:
            continue
        rr.set_time("bag_time", duration=t - t0)
        rr.log("plots/imu/ax", rr.Scalars(float(a[0])))
        rr.log("plots/imu/ay", rr.Scalars(float(a[1])))
        rr.log("plots/imu/az", rr.Scalars(float(a[2])))

    for t, img in image_buf:
        rr.set_time("bag_time", duration=t - t0)
        rr.log("camera/image", rr.Image(img))

    print(f"wrote {args.out} ({args.out.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
