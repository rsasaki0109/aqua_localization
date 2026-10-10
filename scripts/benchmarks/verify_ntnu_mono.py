"""Independently compare every replay scalar/image with native selected ROS packets."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import struct
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from ntnu_sensor_only import ALLOWED, sha, new_json


def take(f, fmt):
    size = struct.calcsize(fmt); data = f.read(size)
    if len(data) != size: raise ValueError('truncated wire')
    return struct.unpack(fmt,data)


def verify(source, directory):
    source, directory = Path(source), Path(directory)
    plan = json.loads((directory/'plan.json').read_text())
    result = json.loads((directory/'result.json').read_text())
    if sha(source) != plan['source_sha256'] or sha(directory/'input.bin') != result['wire_sha256']:
        raise ValueError('source/wire identity mismatch')
    if sha(directory/'native-packet-ledger.jsonl') != result['ledger_sha256']:
        raise ValueError('native ledger changed')
    expected = {}; counts = collections.Counter(); ts = get_typestore(Stores.ROS1_NOETIC)
    cutoff = result['fixed_prefix_end_ns']; first,last = result['native_imu_support_ns']
    with Reader(source) as reader:
        if any(c.topic not in ALLOWED for c in reader.connections): raise ValueError('forbidden topic')
        for c,_,raw in reader.messages():
            m = ts.deserialize_ros1(raw,c.msgtype); ordinal = counts[c.topic]; counts[c.topic] += 1
            stamp = int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            if c.topic.endswith('/imu'):
                stamp += plan['imu_time_shift_ns']
                if stamp > cutoff+plan['imu_lookahead_ns']: continue
                a,g = m.linear_acceleration,m.angular_velocity
                value = struct.pack('<6d',a.x,a.y,a.z,g.x,g.y,g.z); kind = 'I'
            elif c.topic.endswith('/cam0'):
                if ordinal >= plan['native_cam0_prefix'] or not first <= stamp <= last: continue
                value = struct.pack('<III',m.width,m.height,len(m.data))+m.data.tobytes(); kind = 'C'
            elif c.topic.endswith('/static_pressure'):
                if stamp > cutoff: continue
                value = struct.pack('<d',float(m.fluid_pressure)); kind = 'P'
            else: continue
            expected[(kind,ordinal)] = (stamp,hashlib.sha256(value).hexdigest())
    if dict(counts) != result['native_sensor_counts']: raise ValueError('native accounting mismatch')
    verified = collections.Counter(); previous_observation = None; previous_imu = None
    with (directory/'input.bin').open('rb') as f:
        if f.read(8) != b'AQMONO01' or take(f,'<qq') != (first,last): raise ValueError('wire support changed')
        while f.peek(1):
            kind,stamp,ordinal = take(f,'<c q Q'); kind = kind.decode()
            if kind == 'I':
                value = f.read(48)
                if previous_imu is not None and stamp <= previous_imu: raise ValueError('IMU order changed')
                previous_imu = stamp
            elif kind == 'P': value = f.read(8)
            elif kind == 'C':
                w,h,n = take(f,'<III')
                if (w,h,n) != (720,540,388800): raise ValueError('image contract')
                value = struct.pack('<III',w,h,n)+f.read(n)
            else: raise ValueError('unknown wire tag')
            if kind != 'I':
                if previous_observation is not None and stamp < previous_observation:
                    raise ValueError('observation order changed')
                previous_observation = stamp
                if previous_imu is None or not stamp <= previous_imu <= stamp+plan['imu_lookahead_ns']:
                    raise ValueError('common bounded IMU lookahead missing')
            if expected.pop((kind,ordinal),None) != (stamp,hashlib.sha256(value).hexdigest()):
                raise ValueError('wire does not match native bytes/clock/ordinal')
            verified[kind] += 1
    if expected or dict(verified) != result['wire_counts']: raise ValueError('missing native wire packet')
    return dict(schema='ntnu-mono-wire-verification-v1', verified_counts=dict(verified),
        input_sha256=sha(directory/'input.bin'), verifier_sha256=sha(Path(__file__)),
        full_native_selected_sensor_counts=dict(counts), reference_values_read=False,
        status='every_wire_sensor_value_image_ordinal_and_corrected_clock_verified')

if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--source',required=True); p.add_argument('--directory',required=True)
    a = p.parse_args(); value = verify(a.source,a.directory)
    new_json(Path(a.directory)/'wire-verification.json',value)
    print(json.dumps(value,indent=2))
