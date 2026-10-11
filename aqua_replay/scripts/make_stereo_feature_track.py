#!/usr/bin/env python3
"""Track stereo features offline and write them as aqua_msgs/StereoFeatures.

A VINS-style frontend for rectified stereo bags: corners are detected in the
left image, tracked frame to frame with pyramidal Lucas-Kanade (with a
forward-backward consistency check and fundamental-matrix RANSAC), and matched
into the right image with LK along the (rectified) epipolar line. Every
surviving feature keeps a persistent id, so a backend such as aqua_graph can
build landmark tracks and add reprojection factors.

The output is a small rosbag2 holding only the feature topic, stamped with the
left image's header stamp. Feed it to offline_replay next to the raw bag with
``--extra-bag``. Everything here is deterministic (single-threaded OpenCV, no
random sampling outside OpenCV's fixed-seed RANSAC), so the same input bag
always yields the same features.

    make_stereo_feature_track.py --bag short_test_ros2 --dst short_test_features
"""

import argparse
import inspect
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

STEREO_FEATURES_MSG = """
std_msgs/Header header
uint64[] ids
float32[] u_left
float32[] u_right
float32[] v
"""
STEREO_FEATURES_TYPE = "aqua_msgs/msg/StereoFeatures"


def ros2_typestore():
    for name in ("ROS2_JAZZY", "ROS2_HUMBLE", "LATEST"):
        store = getattr(Stores, name, None)
        if store is not None:
            typestore = get_typestore(store)
            typestore.register(get_types_from_msg(STEREO_FEATURES_MSG, STEREO_FEATURES_TYPE))
            return typestore
    raise RuntimeError("rosbags does not expose a ROS 2 typestore")


def decode(msg) -> np.ndarray:
    if hasattr(msg, "format"):  # CompressedImage
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError("failed to decode compressed image")
        return image
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    channels = msg.step // msg.width
    image = data.reshape(msg.height, msg.width, channels) if channels > 1 else \
        data.reshape(msg.height, msg.width)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if channels == 3 else image


class StereoTracker:
    def __init__(self, args):
        self.args = args
        self.lk = dict(winSize=(args.lk_window, args.lk_window), maxLevel=args.lk_levels,
                       criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        self.prev_left = None
        self.prev_pts = np.empty((0, 2), np.float32)
        self.ids = np.empty((0,), np.int64)
        self.next_id = 0
        self.disparity_guess = args.initial_disparity_px

    def _flow(self, img0, img1, pts, guess=None):
        """LK with a forward-backward check; returns tracked points and a mask."""
        if len(pts) == 0:
            return pts, np.zeros((0,), bool)
        p0 = pts.reshape(-1, 1, 2)
        flags = 0
        p1_init = None
        if guess is not None:
            p1_init = guess.reshape(-1, 1, 2).copy()
            flags = cv2.OPTFLOW_USE_INITIAL_FLOW
        p1, st1, _ = cv2.calcOpticalFlowPyrLK(img0, img1, p0, p1_init, flags=flags, **self.lk)
        p0r, st2, _ = cv2.calcOpticalFlowPyrLK(img1, img0, p1, p0.copy(),
                                               flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **self.lk)
        p1 = p1.reshape(-1, 2)
        back = np.linalg.norm(p0r.reshape(-1, 2) - pts, axis=1)
        h, w = img1.shape[:2]
        inside = (p1[:, 0] >= 0) & (p1[:, 0] < w - 1) & (p1[:, 1] >= 0) & (p1[:, 1] < h - 1)
        ok = (st1.ravel() == 1) & (st2.ravel() == 1) & (back < self.args.max_backtrack_px) & inside
        return p1, ok

    def process(self, left: np.ndarray, right: np.ndarray):
        a = self.args
        # 1) Temporal tracking in the left image.
        if self.prev_left is not None and len(self.prev_pts):
            cur, ok = self._flow(self.prev_left, left, self.prev_pts)
            prev, cur, ids = self.prev_pts[ok], cur[ok], self.ids[ok]
            if len(cur) >= 8:
                _, inliers = cv2.findFundamentalMat(prev, cur, cv2.FM_RANSAC,
                                                     a.fundamental_threshold_px, 0.99)
                if inliers is not None:
                    keep = inliers.ravel() == 1
                    cur, ids = cur[keep], ids[keep]
            pts, ids = cur, ids
        else:
            pts, ids = np.empty((0, 2), np.float32), np.empty((0,), np.int64)

        # 2) Top up with new corners away from existing tracks.
        missing = a.max_features - len(pts)
        if missing > 0:
            mask = np.full(left.shape[:2], 255, np.uint8)
            for x, y in pts:
                cv2.circle(mask, (int(round(x)), int(round(y))), a.min_distance_px, 0, -1)
            corners = cv2.goodFeaturesToTrack(left, missing, a.quality_level, a.min_distance_px,
                                              mask=mask)
            if corners is not None:
                corners = corners.reshape(-1, 2).astype(np.float32)
                new_ids = np.arange(self.next_id, self.next_id + len(corners), dtype=np.int64)
                self.next_id += len(corners)
                pts = np.vstack([pts, corners]) if len(pts) else corners
                ids = np.concatenate([ids, new_ids])

        self.prev_left, self.prev_pts, self.ids = left, pts.astype(np.float32), ids

        # 3) Stereo matching into the right image along the epipolar line.
        if not len(pts):
            return ids, pts, np.empty((0,), np.float32)
        guess = pts.copy()
        guess[:, 0] -= self.disparity_guess
        right_pts, ok = self._flow(left, right, pts.astype(np.float32), guess)
        disparity = pts[:, 0] - right_pts[:, 0]
        ok &= np.abs(pts[:, 1] - right_pts[:, 1]) < a.max_epipolar_px
        ok &= (disparity > a.min_disparity_px) & (disparity < a.max_disparity_px)
        if ok.sum() >= 10:
            self.disparity_guess = float(np.median(disparity[ok]))
        return ids[ok], pts[ok], right_pts[ok, 0]


def run(args) -> int:
    cv2.setNumThreads(1)
    dst = Path(args.dst)
    if dst.exists():
        if not args.overwrite:
            print(f"refusing to overwrite {dst}; pass --overwrite", file=sys.stderr)
            return 2
        shutil.rmtree(dst)

    lefts, rights = [], []
    with AnyReader([Path(args.bag)]) as reader:
        conns = [c for c in reader.connections if c.topic in (args.left_topic, args.right_topic)]
        if {c.topic for c in conns} != {args.left_topic, args.right_topic}:
            raise SystemExit(f"bag lacks {args.left_topic} and/or {args.right_topic}")
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            (lefts if conn.topic == args.left_topic else rights).append((stamp_ns, msg))
    lefts.sort(key=lambda r: r[0])
    rights.sort(key=lambda r: r[0])
    right_stamps = np.array([s for s, _ in rights], dtype=np.int64)

    ts = ros2_typestore()
    Msg = ts.types[STEREO_FEATURES_TYPE]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    tracker = StereoTracker(args)
    frames = observations = 0
    processed = 0
    track_lengths = {}
    writer_kwargs = {"version": 9} if "version" in inspect.signature(Writer).parameters else {}
    with Writer(dst, **writer_kwargs) as writer:
        conn = writer.add_connection(args.topic, STEREO_FEATURES_TYPE, typestore=ts)
        for stamp_ns, left_msg in lefts:
            j = int(np.argmin(np.abs(right_stamps - stamp_ns)))
            if abs(int(right_stamps[j]) - stamp_ns) > args.sync_slop_s * 1e9:
                continue
            ids, left_pts, u_right = tracker.process(decode(left_msg), decode(rights[j][1]))
            processed += 1
            # Track on every frame (short baselines track best) but only emit
            # every Nth frame, which bounds the backend's state rate.
            if (processed - 1) % args.publish_every:
                continue
            msg = Msg(
                header=Header(stamp=Time(sec=stamp_ns // 1_000_000_000,
                                         nanosec=stamp_ns % 1_000_000_000),
                              frame_id=args.frame_id),
                ids=ids.astype(np.uint64), u_left=left_pts[:, 0].astype(np.float32),
                u_right=u_right.astype(np.float32), v=left_pts[:, 1].astype(np.float32))
            writer.write(conn, stamp_ns, ts.serialize_cdr(msg, STEREO_FEATURES_TYPE))
            frames += 1
            observations += len(ids)
            for i in ids.tolist():
                track_lengths[i] = track_lengths.get(i, 0) + 1
    lengths = np.array(list(track_lengths.values()) or [0])
    print(f"wrote {dst}: {frames} frames, {observations} stereo observations, "
          f"{len(track_lengths)} tracks (median length {np.median(lengths):.0f}, "
          f"p90 {np.percentile(lengths, 90):.0f})")
    return 0


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--bag", required=True, help="Input rosbag2 with stereo images.")
    p.add_argument("--dst", required=True, help="Output rosbag2 directory.")
    p.add_argument("--left-topic", default="/camera/left/image_dehazed/compressed")
    p.add_argument("--right-topic", default="/camera/right/image_dehazed/compressed")
    p.add_argument("--topic", default="/aqua_visual_frontend/stereo_features")
    p.add_argument("--frame-id", default="camera_left")
    p.add_argument("--sync-slop-s", type=float, default=0.005)
    p.add_argument("--max-features", type=int, default=200)
    p.add_argument("--publish-every", type=int, default=1,
                   help="Emit every Nth tracked frame (tracking still uses all frames).")
    p.add_argument("--min-distance-px", type=int, default=20)
    p.add_argument("--quality-level", type=float, default=0.01)
    p.add_argument("--lk-window", type=int, default=21)
    p.add_argument("--lk-levels", type=int, default=3)
    p.add_argument("--max-backtrack-px", type=float, default=0.5)
    p.add_argument("--fundamental-threshold-px", type=float, default=1.0)
    p.add_argument("--max-epipolar-px", type=float, default=1.5)
    p.add_argument("--min-disparity-px", type=float, default=1.0)
    p.add_argument("--max-disparity-px", type=float, default=200.0)
    p.add_argument("--initial-disparity-px", type=float, default=0.0)
    p.add_argument("--overwrite", action="store_true")
    return p.parse_args(argv)


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
