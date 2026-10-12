#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Absolute trajectory error of an aqua_graph replay against AQUALOC ground truth.

AQUALOC's COLMAP ground truth gives camera poses for a subset of images,
indexed by the image's position in the sequence (`img_number tx ty tz qx qy
qz qw`). The image stamps come from the source bag; each is shifted by
Kalibr's timeshift_cam_imu into IMU time, the estimated body pose is
interpolated there, and mapped to the camera with T_imu_cam. Reports
SE(3)- and Sim(3)-aligned ATE RMSE (positions) over the matched images.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluate_replay_trajectories import umeyama  # noqa: E402


def quat_to_rot(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def stamp(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def image_stamps(bag: Path, topic: str) -> np.ndarray:
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        return np.array([stamp(reader.deserialize(raw, c.msgtype))
                         for c, _, raw in reader.messages(connections=conns)])


def estimate(bag: Path, topic: str):
    """Odometry stream, or the poses of the last nav_msgs/Path message."""
    t, p, q = [], [], []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        last_path = None
        for c, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, c.msgtype)
            if c.msgtype == 'nav_msgs/msg/Path':
                last_path = m
                continue
            t.append(stamp(m))
            pp, qq = m.pose.pose.position, m.pose.pose.orientation
            p.append((pp.x, pp.y, pp.z))
            q.append((qq.x, qq.y, qq.z, qq.w))
        if last_path is not None:
            for ps in last_path.poses:
                t.append(stamp(ps))
                pp, qq = ps.pose.position, ps.pose.orientation
                p.append((pp.x, pp.y, pp.z))
                q.append((qq.x, qq.y, qq.z, qq.w))
    return np.array(t), np.array(p), np.array(q)


def interpolate_pose(t, p, q, s):
    """Body pose at stamp s (linear position, nlerp orientation), or None."""
    i = int(np.searchsorted(t, s))
    if i <= 0 or i >= len(t):
        return None
    f = (s - t[i - 1]) / max(t[i] - t[i - 1], 1e-9)
    qa, qb = q[i - 1], q[i]
    if np.dot(qa, qb) < 0:
        qb = -qb
    qi = (1 - f) * qa + f * qb
    qi /= np.linalg.norm(qi)
    return (1 - f) * p[i - 1] + f * p[i], quat_to_rot(*qi)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--replay', required=True, type=Path, help='offline_replay output bag')
    ap.add_argument('--source', required=True, type=Path, help='source bag with the images')
    ap.add_argument('--gt', required=True, type=Path, help='AQUALOC *_colmap_traj_sequence_*.txt')
    ap.add_argument('--image-topic', default='/camera/image_raw')
    ap.add_argument('--estimate-topic', default='/aqua_graph/odometry')
    ap.add_argument('--timeshift-cam-imu', type=float, default=-0.0403806549886)
    ap.add_argument('--t-imu-cam', type=float, nargs=16, default=[
        -0.99978035, 0.0169654, 0.01230552, -0.01719238,
        0.01210101, -0.01210461, 0.99985351, 0.14944769,
        0.01711187, 0.9997828, 0.01189665, -0.01915984,
        0.0, 0.0, 0.0, 1.0])
    ap.add_argument('--json', type=Path, help='write the metrics here')
    args = ap.parse_args()

    T_ic = np.array(args.t_imu_cam).reshape(4, 4)
    gt = np.loadtxt(args.gt, comments='#')
    stamps = image_stamps(args.source, args.image_topic) + args.timeshift_cam_imu
    t, p, q = estimate(args.replay, args.estimate_topic)
    est, ref = [], []
    for row in gt:
        idx = int(round(row[0]))
        if idx >= len(stamps):
            continue
        pose = interpolate_pose(t, p, q, stamps[idx])
        if pose is None:
            continue
        pos, rot = pose
        est.append(rot @ T_ic[:3, 3] + pos)
        ref.append(row[1:4])
    est, ref = np.array(est), np.array(ref)
    if len(est) < 3:
        sys.exit(f'only {len(est)} ground-truth images matched the estimate')
    out = {'matched': int(len(est)), 'gt_rows': int(len(gt)),
           'gt_length_m': float(np.linalg.norm(np.diff(gt[:, 1:4], axis=0), axis=1).sum())}
    for name, with_scale in (('se3', False), ('sim3', True)):
        s, R, tt = umeyama(est, ref, with_scale)
        err = np.linalg.norm((s * (R @ est.T)).T + tt - ref, axis=1)
        out[f'ate_rmse_{name}_m'] = float(np.sqrt(np.mean(err ** 2)))
        out[f'ate_max_{name}_m'] = float(err.max())
        if with_scale:
            out['sim3_scale'] = float(s)
    out['ate_se3_percent_of_length'] = 100.0 * out['ate_rmse_se3_m'] / out['gt_length_m']
    print(json.dumps(out, indent=2))
    if args.json:
        args.json.write_text(json.dumps(out, indent=2) + '\n')


if __name__ == '__main__':
    main()
