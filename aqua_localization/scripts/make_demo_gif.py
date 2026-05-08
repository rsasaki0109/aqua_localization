#!/usr/bin/env python3
"""Capture a side-by-side sonar fan + estimated trajectory animation as a GIF.

Subscribes to a sonar `PointCloud2` topic and a `nav_msgs/Odometry` topic, plus an
optional `aqua_msgs/ScanMatchingStatus` topic for the fitness overlay. On each new
point cloud the script records a matplotlib frame:

- left panel: top-down (X/Y) scatter of the latest fan in the sonar frame
- right panel: cumulative top-down trajectory built from the odometry stream

A bottom annotation reports the wall-clock seconds since the first frame and the
most recent registration fitness score. On shutdown (Ctrl-C, or after
`--max-frames`/`--duration-s` is reached) the buffered frames are written out as
an animated GIF.

Typical usage with the MBES-SLAM `beach_pond` bag, while `aqua_sonar_loc` is
already running and consuming `/norbit/detections`:

    ros2 run aqua_localization make_demo_gif.py \
        --points /aqua_sonar_loc/points_filtered \
        --odometry /aqua_sonar_loc/odometry \
        --status /aqua_sonar_loc/status \
        --duration-s 8.0 \
        --out docs/media/mbes_slam_beach_pond.gif

The script intentionally does not run a bag itself — pair it with
`ros2 launch aqua_localization replay.launch.py` (or a manual `ros2 bag play`)
so the inputs and the recorder share the same sim_time clock.
"""

import argparse
import io
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

import rclpy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402

try:
    from aqua_msgs.msg import ScanMatchingStatus
except ImportError:
    ScanMatchingStatus = None


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class DemoGifRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("aqua_demo_gif_recorder")
        self.args = args
        self.frames: list[bytes] = []
        self.trajectory: list[tuple[float, float, float]] = []  # (t, x, y)
        # Latest odometry as (t, x, y, z, qx, qy, qz, qw); used to transform
        # incoming sonar fans into the odom frame for the accumulated map view.
        self.latest_odom: tuple[float, ...] | None = None
        self.latest_reference_pose: tuple[float, ...] | None = None
        # Reference origin used to ground the accumulated map at (0, 0) so the map
        # axes match the trajectory axes after subtracting ref[0].
        self.reference_origin: tuple[float, float, float] | None = None
        # Accumulated map: list of (x, y, z) tuples in odom frame, capped at
        # --max-map-points by reservoir-style downsampling.
        self.accumulated_xyz = np.empty((0, 3), dtype=np.float32)
        self.last_fitness: float | None = None
        self.first_frame_time: float | None = None
        self.start_wall = time.monotonic()
        self.skipped_until_first_odom = True

        sensor_qos = QoSProfile(depth=20)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.points_sub = self.create_subscription(
            PointCloud2, args.points, self.on_points, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, args.odometry, self.on_odom, QoSProfile(depth=200)
        )
        if ScanMatchingStatus is not None and args.status:
            self.status_sub = self.create_subscription(
                ScanMatchingStatus, args.status, self.on_status, QoSProfile(depth=200)
            )
        else:
            self.status_sub = None

        self.reference_traj: list[tuple[float, float, float, float]] = []
        if args.reference_odometry:
            self.reference_sub = self.create_subscription(
                Odometry,
                args.reference_odometry,
                self.on_reference_odom,
                QoSProfile(depth=400),
            )
        else:
            self.reference_sub = None

        # Single matplotlib figure reused across frames; renders to an in-memory
        # PNG and decoded into a PIL frame so we never hit the filesystem.
        self.fig, (self.ax_points, self.ax_traj) = plt.subplots(
            1, 2, figsize=(args.width / args.dpi, args.height / args.dpi), dpi=args.dpi
        )
        self.fig.subplots_adjust(left=0.08, right=0.97, bottom=0.20, top=0.84, wspace=0.30)
        self.fig.suptitle("aqua_sonar_loc · MBES-SLAM beach_pond", fontsize=12, y=0.96)
        # Single Text artist for the bottom overlay; we update its content each
        # frame instead of churning new fig.text artists (which previously also
        # cleared the suptitle when we wiped them out).
        self.bottom_overlay = self.fig.text(
            0.5, 0.06, "", ha="center", va="center", fontsize=10, family="monospace"
        )

        self.get_logger().info(
            f"recording GIF: points={args.points} odom={args.odometry} "
            f"status={args.status if self.status_sub else '(disabled)'} "
            f"target={args.out} max_frames={args.max_frames} duration={args.duration_s}s"
        )

    # ----- subscriptions -------------------------------------------------------------

    def on_odom(self, msg: Odometry) -> None:
        t = stamp_to_seconds(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.trajectory.append((t, p.x, p.y, p.z))
        self.latest_odom = (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        self.skipped_until_first_odom = False

    def on_status(self, msg) -> None:
        if msg.success and np.isfinite(msg.fitness_score):
            self.last_fitness = float(msg.fitness_score)

    def on_reference_odom(self, msg: Odometry) -> None:
        t = stamp_to_seconds(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.reference_traj.append((t, p.x, p.y, p.z))
        self.latest_reference_pose = (t, p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def on_points(self, msg: PointCloud2) -> None:
        # Wait until we have a pose to ground the accumulated map and the trajectory
        # before recording the first frame. Reference takes priority when subscribed
        # because it drives the map transform.
        if self.reference_sub is not None:
            if self.latest_reference_pose is None:
                return
        elif self.skipped_until_first_odom:
            return
        if self.first_frame_time is None:
            self.first_frame_time = stamp_to_seconds(msg.header.stamp)

        # sensor_msgs_py.point_cloud2.read_points returns a structured numpy array
        # with named fields, not a plain (N, 3) float array. Stack the named columns
        # so we can scatter-plot them directly.
        structured = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if len(structured) == 0:
            return
        fan_local = np.stack(
            [structured["x"], structured["y"], structured["z"]], axis=1
        ).astype(np.float32, copy=False)

        # Transform the fan (sonar/base frame) into the odom frame using the latest
        # odometry pose. Approximation: sonar frame ~ base_link; if your platform has
        # a non-trivial sonar→base TF this should incorporate that lookup. The MBES-
        # SLAM bag publishes points in `norbit` and we do not yet have a static TF
        # for the lever arm, so we treat them as identity.
        fan_odom = self._transform_to_odom(fan_local)
        self._accumulate_points(fan_odom)

        elapsed = stamp_to_seconds(msg.header.stamp) - self.first_frame_time
        self.render_frame(elapsed)

        if self.args.max_frames and len(self.frames) >= self.args.max_frames:
            self.get_logger().info("hit --max-frames; flushing GIF and shutting down")
            self.shutdown_and_save()
        elif self.args.duration_s and elapsed >= self.args.duration_s:
            self.get_logger().info(f"reached duration {self.args.duration_s}s; flushing GIF")
            self.shutdown_and_save()

    # ----- transform / accumulate helpers --------------------------------------------

    @staticmethod
    def _quat_to_matrix(q: tuple[float, float, float, float]) -> np.ndarray:
        # q = (qx, qy, qz, qw)
        x, y, z, w = q
        n = x * x + y * y + z * z + w * w
        if n < 1.0e-12:
            return np.eye(3, dtype=np.float32)
        s = 2.0 / n
        return np.array(
            [
                [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
                [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
            ],
            dtype=np.float32,
        )

    def _transform_to_odom(self, fan_local: np.ndarray) -> np.ndarray:
        # When a reference odometry is available, use its pose to transform fans
        # into world coordinates; this is what reveals the seabed shape when the
        # estimator itself is stuck near origin (no IMU prior). With both the GT
        # and the estimate normalized to start at (0, 0), the accumulated map and
        # the trajectory share a single common origin.
        if self.latest_reference_pose is not None:
            _, tx, ty, tz, qx, qy, qz, qw = self.latest_reference_pose
            if self.reference_origin is None:
                self.reference_origin = (tx, ty, tz)
            ox, oy, oz = self.reference_origin
            tx, ty, tz = tx - ox, ty - oy, tz - oz
        elif self.latest_odom is not None:
            _, tx, ty, tz, qx, qy, qz, qw = self.latest_odom
        else:
            return fan_local
        rot = self._quat_to_matrix((qx, qy, qz, qw))
        return fan_local @ rot.T + np.array([tx, ty, tz], dtype=np.float32)

    def _accumulate_points(self, fan_odom: np.ndarray) -> None:
        # Decimate aggressively so the map stays light: keep ~1 in N points per fan.
        decim = max(1, int(self.args.fan_decimation))
        keep = fan_odom[::decim]
        self.accumulated_xyz = np.concatenate([self.accumulated_xyz, keep], axis=0)
        # Cap the running map at --max-map-points by uniform random thinning. This
        # keeps memory and matplotlib scatter draw time bounded for long captures.
        cap = self.args.max_map_points
        if cap > 0 and self.accumulated_xyz.shape[0] > cap:
            idx = np.random.default_rng(seed=42).choice(
                self.accumulated_xyz.shape[0], size=cap, replace=False
            )
            self.accumulated_xyz = self.accumulated_xyz[np.sort(idx)]

    # ----- rendering -----------------------------------------------------------------

    def render_frame(self, elapsed_s: float) -> None:
        ax_p = self.ax_points
        ax_t = self.ax_traj
        ax_p.clear()
        ax_t.clear()

        # Left: accumulated point cloud in odom frame, top-down, colored by depth.
        if self.accumulated_xyz.size > 0:
            ax_p.scatter(
                self.accumulated_xyz[:, 0],
                self.accumulated_xyz[:, 1],
                c=self.accumulated_xyz[:, 2],
                cmap="viridis_r",
                s=2,
                linewidths=0,
            )
        ax_p.set_aspect("equal")
        ax_p.set_xlabel("x (m)")
        ax_p.set_ylabel("y (m)")
        ax_p.set_title("Accumulated multibeam map (top-down, color = depth)")
        ax_p.grid(True, alpha=0.3)

        # Right: cumulative trajectories, top-down. Both trajectories are shifted so
        # they start at (0, 0); this lets us overlay the reference (UTM-scale) and
        # the aqua_sonar_loc estimate (already odom-frame from origin) on a single
        # axis without one collapsing the other to a point.
        if self.reference_traj:
            ref = np.asarray([(x, y) for _, x, y, _ in self.reference_traj])
            ref = ref - ref[0]
            ax_t.plot(
                ref[:, 0], ref[:, 1], "-", color="#2ca02c", linewidth=1.5,
                label="reference",
            )
            ax_t.plot(ref[-1, 0], ref[-1, 1], "o", color="#2ca02c", markersize=5)
        if self.trajectory:
            traj = np.asarray([(x, y) for _, x, y, _ in self.trajectory])
            traj = traj - traj[0]
            ax_t.plot(
                traj[:, 0], traj[:, 1], "-", color="#1f77b4", linewidth=1.5,
                label="aqua_sonar_loc",
            )
            ax_t.plot(traj[-1, 0], traj[-1, 1], "o", color="#d62728", markersize=6)
        if self.reference_traj or self.trajectory:
            ax_t.legend(loc="best", fontsize=8)
        ax_t.set_aspect("equal")
        ax_t.set_xlabel("x (m)")
        ax_t.set_ylabel("y (m)")
        ax_t.set_title("Estimated trajectory (X/Y)")
        ax_t.grid(True, alpha=0.3)

        fitness_text = (
            f"fitness {self.last_fitness:.3f}" if self.last_fitness is not None else "fitness n/a"
        )
        self.bottom_overlay.set_text(
            f"t = {elapsed_s:5.2f}s   |   {fitness_text}   |   frame {len(self.frames) + 1}"
            f"   |   map pts {self.accumulated_xyz.shape[0]}"
        )

        buf = io.BytesIO()
        self.fig.savefig(buf, format="png")
        buf.seek(0)
        self.frames.append(buf.getvalue())

    # ----- shutdown ------------------------------------------------------------------

    def shutdown_and_save(self) -> None:
        if not self.frames:
            self.get_logger().warning("no frames captured; nothing to write")
        else:
            output = self.args.out
            output.parent.mkdir(parents=True, exist_ok=True)
            images = [Image.open(io.BytesIO(buf)) for buf in self.frames]
            duration_ms = max(1, int(round(1000.0 / max(self.args.fps, 1.0))))
            images[0].save(
                output,
                save_all=True,
                append_images=images[1:],
                duration=duration_ms,
                loop=0,
                optimize=True,
                disposal=2,
            )
            self.get_logger().info(
                f"wrote {len(self.frames)} frames to {output} ({duration_ms} ms/frame)"
            )
        rclpy.shutdown()


def parse_args(argv) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record a side-by-side sonar-fan + trajectory GIF from a running aqua_sonar_loc."
    )
    parser.add_argument("--points", default="/aqua_sonar_loc/points_filtered")
    parser.add_argument("--odometry", default="/aqua_sonar_loc/odometry")
    parser.add_argument("--status", default="/aqua_sonar_loc/status")
    parser.add_argument(
        "--reference-odometry",
        default="",
        help="Optional reference odometry topic (e.g. /nav/processed/odometry from "
        "the MBES-SLAM bag) drawn alongside the aqua_sonar_loc estimate.",
    )
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument(
        "--duration-s",
        type=float,
        default=8.0,
        help="Stop capture after this many seconds of bag time. 0 disables.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=120,
        help="Hard cap on number of frames recorded.",
    )
    parser.add_argument("--fps", type=float, default=10.0, help="Output GIF frames per second.")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--dpi", type=int, default=120)
    parser.add_argument(
        "--fan-decimation",
        type=int,
        default=4,
        help="Keep 1-in-N points from each multibeam fan when accumulating the map.",
    )
    parser.add_argument(
        "--max-map-points",
        type=int,
        default=20000,
        help="Cap on accumulated map points; older points are randomly thinned.",
    )
    parser.add_argument("--use-sim-time", action="store_true")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = DemoGifRecorder(args)
    if args.use_sim_time:
        node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Save whatever we have so a Ctrl-C still gets a partial GIF.
        if rclpy.ok():
            node.shutdown_and_save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
