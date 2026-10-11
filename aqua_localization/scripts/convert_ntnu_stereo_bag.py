#!/usr/bin/env python3
"""Convert an NTNU underwater-datasets ROS 1 bag to a rectified-stereo ROS 2 bag.

The NTNU multi-camera dataset (huggingface.co/datasets/ntnu-arl/underwater-datasets)
records five Alphasense cameras with equidistant (fisheye) distortion. The
aqua_visual_frontend tracker expects rectified stereo, so this script:

1. Reads the ROS 1 bag with `rosbags` (no ROS 1 install needed).
2. Rectifies the cam0/cam1 pair with OpenCV's fisheye model, using the
   in-water intrinsics + stereo extrinsic (Kalibr camchain), and orders the
   pair so the published "left" camera is the one with the other camera to
   its right (Kalibr's cam0 is not necessarily the left one).
3. Writes the rectified pair as JPEG `sensor_msgs/CompressedImage` on
   /camera/left/image_rect/compressed and /camera/right/image_rect/compressed,
   plus the Alphasense IMU, the MAVROS barometer (/pressure; note it is the
   hull's internal barometer, not a water-depth sensor), and the dataset's reference
   trajectory (`<sequence>_baseline.tum`, ReAqROVIO with four cameras + IMU)
   as nav_msgs/Odometry on /reference/odometry.
4. Writes an aqua_graph parameter overlay (--params-out) with the rectified
   intrinsics, the baseline, and body_T_camera = T_imu_leftrect from the
   Kalibr IMU-camera calibration, so the backend uses exactly the geometry
   the images were rectified with.

    convert_ntnu_stereo_bag.py --src fjord_1.bag --reference fjord_1_baseline.tum \\
        --intrinsics camchain-stereo-intrinsics-underwater.yaml \\
        --imu-extrinsics camchain-imucam-stereo-extrinsics-air.yaml \\
        --dst fjord_1_stereo_ros2 --params-out fjord_1_aqua_graph.yaml
"""

import argparse
import bz2
import inspect
import io
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

LEFT_TOPIC = "/camera/left/image_rect/compressed"
RIGHT_TOPIC = "/camera/right/image_rect/compressed"
IMU_TOPIC = "/imu/data"
PRESSURE_TOPIC = "/pressure"
REFERENCE_TOPIC = "/reference/odometry"


def camchain_camera(chain: dict, name: str):
    cam = chain[name]
    if cam.get("distortion_model") != "equidistant" or cam.get("camera_model") != "pinhole":
        raise SystemExit(f"{name}: only pinhole-equidistant (Kalibr) cameras are supported")
    fx, fy, cx, cy = cam["intrinsics"]
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    D = np.array(cam["distortion_coeffs"], dtype=np.float64).reshape(4, 1)
    size = tuple(int(v) for v in cam["resolution"])  # (width, height)
    return K, D, size, cam["rostopic"]


def rectification(intrinsics: dict, balance: float):
    """Return per-camera maps, rectified P, and the ordering (left, right) of cam0/cam1."""
    K0, D0, size, topic0 = camchain_camera(intrinsics, "cam0")
    K1, D1, _, topic1 = camchain_camera(intrinsics, "cam1")
    T_10 = np.array(intrinsics["cam1"]["T_cn_cnm1"], dtype=np.float64)  # p_c1 = T_10 p_c0
    R, t = T_10[:3, :3], T_10[:3, 3]
    # OpenCV wants the second camera at +x of the first, i.e. t_x < 0 in
    # p_2 = R p_1 + t. Swap the pair if cam1 is the left camera.
    if t[0] < 0:
        order = (("cam0", K0, D0, topic0), ("cam1", K1, D1, topic1))
    else:
        order = (("cam1", K1, D1, topic1), ("cam0", K0, D0, topic0))
        R, t = R.T, -R.T @ t
    (ln, Kl, Dl, lt), (rn, Kr, Dr, rt) = order
    R_l, R_r, P_l, P_r, _ = cv2.fisheye.stereoRectify(
        Kl, Dl, Kr, Dr, size, R, t.reshape(3, 1), flags=cv2.CALIB_ZERO_DISPARITY,
        newImageSize=size, balance=balance, fov_scale=1.0)
    maps = {
        lt: cv2.fisheye.initUndistortRectifyMap(Kl, Dl, R_l, P_l, size, cv2.CV_16SC2),
        rt: cv2.fisheye.initUndistortRectifyMap(Kr, Dr, R_r, P_r, size, cv2.CV_16SC2),
    }
    baseline = -P_r[0, 3] / P_r[0, 0]
    return {"left_name": ln, "right_name": rn, "left_topic": lt, "right_topic": rt,
            "R_left": R_l, "P_left": P_l, "baseline": float(baseline), "maps": maps,
            "size": size}


def body_T_left_rect(imu_chain: dict, left_name: str, R_left: np.ndarray) -> np.ndarray:
    T_cam_imu = np.array(imu_chain[left_name]["T_cam_imu"], dtype=np.float64)
    T_imu_cam = np.linalg.inv(T_cam_imu)
    T_cam_rect = np.eye(4)
    T_cam_rect[:3, :3] = R_left.T  # p_rect = R_left p_cam
    return T_imu_cam @ T_cam_rect


def decode(msg) -> np.ndarray:
    if hasattr(msg, "format"):
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_GRAYSCALE)
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    channels = msg.step // msg.width
    img = data.reshape(msg.height, msg.width, channels) if channels > 1 else \
        data.reshape(msg.height, msg.width)
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if channels == 3 else img


def _read_exact(stream, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            raise EOFError
        buf += chunk
    return bytes(buf)


def _fields(header: bytes) -> dict:
    out, pos = {}, 0
    while pos < len(header):
        (n,) = struct.unpack_from("<I", header, pos)
        key, _, value = header[pos + 4:pos + 4 + n].partition(b"=")
        out[key.decode()] = value
        pos += 4 + n
    return out


def _records(stream):
    """Yield (fields, data) for each ROS 1 bag v2.0 record in `stream`."""
    while True:
        try:
            (hlen,) = struct.unpack("<I", _read_exact(stream, 4))
        except EOFError:
            return
        fields = _fields(_read_exact(stream, hlen))
        (dlen,) = struct.unpack("<I", _read_exact(stream, 4))
        yield fields, _read_exact(stream, dlen)


def stream_ros1_messages(stream, topics):
    """Sequentially parse a ROS 1 bag (no index needed: connection records
    precede the messages that use them) and yield (topic, msgtype, t_ns, raw)
    for the wanted topics. Lets huge remote bags be converted without a local
    copy."""
    magic = _read_exact(stream, 13)
    if magic != b"#ROSBAG V2.0\n":
        raise SystemExit(f"not a ROS 1 bag v2.0 stream: {magic!r}")
    conns = {}

    def handle(fields, data):
        op = fields["op"][0]
        if op == 0x07:  # connection
            conn = struct.unpack("<I", fields["conn"])[0]
            info = _fields(data)
            msgtype = info["type"].decode()
            pkg, _, name = msgtype.partition("/")
            conns[conn] = (fields["topic"].decode(), f"{pkg}/msg/{name}")
        elif op == 0x02:  # message data
            conn = struct.unpack("<I", fields["conn"])[0]
            topic, msgtype = conns[conn]
            if topic in topics:
                sec, nsec = struct.unpack("<II", fields["time"])
                return topic, msgtype, sec * 1_000_000_000 + nsec, data
        return None

    for fields, data in _records(stream):
        op = fields["op"][0]
        if op == 0x05:  # chunk
            compression = fields["compression"].decode()
            if compression == "bz2":
                data = bz2.decompress(data)
            elif compression == "lz4":
                import lz4.frame
                data = lz4.frame.decompress(data)
            elif compression != "none":
                raise SystemExit(f"unsupported chunk compression {compression}")
            for inner_fields, inner_data in _records(io.BytesIO(data)):
                out = handle(inner_fields, inner_data)
                if out is not None:
                    yield out
        else:
            out = handle(fields, data)
            if out is not None:
                yield out


def iter_source(args, wanted):
    """Yield (topic, t_ns, deserialized msg) from --src (file) or --src-url (stream)."""
    if args.src_url:
        ts1 = get_typestore(Stores.ROS1_NOETIC)
        curl = subprocess.Popen(["curl", "-sSL", "--fail", args.src_url], stdout=subprocess.PIPE)
        try:
            stream = io.BufferedReader(curl.stdout, buffer_size=1 << 22)
            for topic, msgtype, t_ns, raw in stream_ros1_messages(stream, wanted):
                yield topic, t_ns, ts1.deserialize_ros1(raw, msgtype)
        finally:
            curl.kill()
            curl.wait()
        return
    with AnyReader([Path(args.src)]) as reader:
        src_conns = [c for c in reader.connections if c.topic in wanted]
        missing = wanted - {c.topic for c in src_conns}
        if missing:
            raise SystemExit(f"source bag lacks {sorted(missing)}")
        for conn, t_ns, raw in reader.messages(connections=src_conns):
            yield conn.topic, t_ns, reader.deserialize(raw, conn.msgtype)


def ros2_typestore():
    for name in ("ROS2_JAZZY", "ROS2_HUMBLE", "LATEST"):
        store = getattr(Stores, name, None)
        if store is not None:
            return get_typestore(store)
    raise RuntimeError("rosbags lacks a ROS 2 typestore")


def run(args) -> int:
    dst = Path(args.dst)
    if dst.exists():
        if not args.overwrite:
            print(f"refusing to overwrite {dst}; pass --overwrite", file=sys.stderr)
            return 2
        shutil.rmtree(dst)
    intrinsics = yaml.safe_load(Path(args.intrinsics).read_text())
    imu_chain = yaml.safe_load(Path(args.imu_extrinsics).read_text())
    rect = rectification(intrinsics, args.balance)
    T_body_cam = body_T_left_rect(imu_chain, rect["left_name"], rect["R_left"])
    P = rect["P_left"]
    print(f"left={rect['left_name']} right={rect['right_name']} fx={P[0, 0]:.2f} "
          f"cx={P[0, 2]:.2f} cy={P[1, 2]:.2f} baseline={rect['baseline']:.5f} m")

    ts = ros2_typestore()
    T = ts.types
    Header, Time = T["std_msgs/msg/Header"], T["builtin_interfaces/msg/Time"]

    def header(ns, frame):
        return Header(stamp=Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000),
                      frame_id=frame)

    def stamp_ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    writer_kwargs = {"version": 9} if "version" in inspect.signature(Writer).parameters else {}
    start = end = None
    counts = {}
    with Writer(dst, **writer_kwargs) as writer:
        conns = {
            LEFT_TOPIC: writer.add_connection(LEFT_TOPIC, "sensor_msgs/msg/CompressedImage",
                                              typestore=ts),
            RIGHT_TOPIC: writer.add_connection(RIGHT_TOPIC, "sensor_msgs/msg/CompressedImage",
                                               typestore=ts),
            IMU_TOPIC: writer.add_connection(IMU_TOPIC, "sensor_msgs/msg/Imu", typestore=ts),
            PRESSURE_TOPIC: writer.add_connection(PRESSURE_TOPIC, "sensor_msgs/msg/FluidPressure",
                                                  typestore=ts),
            REFERENCE_TOPIC: writer.add_connection(REFERENCE_TOPIC, "nav_msgs/msg/Odometry",
                                                   typestore=ts),
        }
        out_topic = {rect["left_topic"]: LEFT_TOPIC, rect["right_topic"]: RIGHT_TOPIC}
        wanted = set(out_topic) | {args.imu_topic, args.pressure_topic}
        for topic_in, t_ns, msg in iter_source(args, wanted):
            if args.duration_s > 0 and start is not None and \
                    t_ns - start > args.duration_s * 1e9:
                break
            start = t_ns if start is None else start
            end = t_ns
            ns = stamp_ns(msg)
            if topic_in in out_topic:
                m1, m2 = rect["maps"][topic_in]
                img = cv2.remap(decode(msg), m1, m2, cv2.INTER_LINEAR)
                ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                if not ok:
                    continue
                topic = out_topic[topic_in]
                out = T["sensor_msgs/msg/CompressedImage"](
                    header=header(ns, "camera_left" if topic == LEFT_TOPIC else "camera_right"),
                    format="mono8; jpeg compressed mono8", data=jpg.reshape(-1))
                writer.write(conns[topic], t_ns,
                             ts.serialize_cdr(out, "sensor_msgs/msg/CompressedImage"))
            elif topic_in == args.imu_topic:
                topic = IMU_TOPIC
                out = T["sensor_msgs/msg/Imu"](
                    header=header(ns, "imu"), orientation=msg.orientation,
                    orientation_covariance=np.asarray(msg.orientation_covariance),
                    angular_velocity=msg.angular_velocity,
                    angular_velocity_covariance=np.asarray(msg.angular_velocity_covariance),
                    linear_acceleration=msg.linear_acceleration,
                    linear_acceleration_covariance=np.asarray(
                        msg.linear_acceleration_covariance))
                writer.write(conns[topic], t_ns, ts.serialize_cdr(out, "sensor_msgs/msg/Imu"))
            else:
                topic = PRESSURE_TOPIC
                out = T["sensor_msgs/msg/FluidPressure"](
                    header=header(ns, "pressure"), fluid_pressure=msg.fluid_pressure,
                    variance=msg.variance)
                writer.write(conns[topic], t_ns,
                             ts.serialize_cdr(out, "sensor_msgs/msg/FluidPressure"))
            counts[topic] = counts.get(topic, 0) + 1

        # Reference trajectory (TUM: t x y z qx qy qz qw) as Odometry.
        ref = np.loadtxt(args.reference, comments="#")
        P_, Q_ = T["geometry_msgs/msg/Point"], T["geometry_msgs/msg/Quaternion"]
        for row in ref:
            ns = int(round(row[0] * 1e9))
            if start is not None and not (start - 1e9 <= ns <= end + 1e9):
                continue
            odom = T["nav_msgs/msg/Odometry"](
                header=header(ns, "world"), child_frame_id="imu",
                pose=T["geometry_msgs/msg/PoseWithCovariance"](
                    pose=T["geometry_msgs/msg/Pose"](
                        position=P_(x=row[1], y=row[2], z=row[3]),
                        orientation=Q_(x=row[4], y=row[5], z=row[6], w=row[7])),
                    covariance=np.zeros(36)),
                twist=T["geometry_msgs/msg/TwistWithCovariance"](
                    twist=T["geometry_msgs/msg/Twist"](
                        linear=T["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0),
                        angular=T["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)),
                    covariance=np.zeros(36)))
            writer.write(conns[REFERENCE_TOPIC], ns, ts.serialize_cdr(odom, "nav_msgs/msg/Odometry"))
            counts[REFERENCE_TOPIC] = counts.get(REFERENCE_TOPIC, 0) + 1
    print(f"wrote {dst}: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))

    if args.params_out:
        params = {"aqua_graph": {"ros__parameters": {"visual": {
            "camera": {"fx": float(P[0, 0]), "fy": float(P[1, 1]), "cx": float(P[0, 2]),
                       "cy": float(P[1, 2]), "baseline_m": rect["baseline"]},
            "body_T_camera": [float(v) for v in T_body_cam.reshape(-1)],
        }}}}
        Path(args.params_out).write_text(
            "# Generated by convert_ntnu_stereo_bag.py: rectified left-camera geometry\n"
            + yaml.safe_dump(params, sort_keys=False))
        print(f"wrote {args.params_out}")
    return 0


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--src", help="Local ROS 1 bag.")
    src.add_argument("--src-url",
                     help="Stream the ROS 1 bag over HTTP(S) (via curl) instead of reading a "
                          "local copy; for bags larger than the free disk.")
    p.add_argument("--dst", required=True)
    p.add_argument("--reference", required=True, help="<sequence>_baseline.tum")
    p.add_argument("--intrinsics", required=True,
                   help="Kalibr camchain with cam0/cam1 intrinsics (in-water) and T_cn_cnm1.")
    p.add_argument("--imu-extrinsics", required=True,
                   help="Kalibr imu-cam camchain with T_cam_imu for cam0/cam1.")
    p.add_argument("--imu-topic", default="/alphasense_driver_ros/imu")
    p.add_argument("--pressure-topic", default="/mavros/imu/static_pressure")
    p.add_argument("--params-out", help="Write an aqua_graph visual-geometry params overlay.")
    p.add_argument("--balance", type=float, default=0.0,
                   help="cv2.fisheye.stereoRectify balance (0 = crop to valid pixels).")
    p.add_argument("--jpeg-quality", type=int, default=95)
    p.add_argument("--duration-s", type=float, default=0.0)
    p.add_argument("--overwrite", action="store_true")
    return p.parse_args(argv)


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())
