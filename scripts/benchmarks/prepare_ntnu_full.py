"""Full sensor accounting and a preregistered native cam0/pressure development prefix."""
import argparse
import bisect
import collections
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from ntnu_sensor_only import ALLOWED, sha, new_json, select

IMU = '/alphasense_driver_ros/imu'
CAM = '/alphasense_driver_ros/cam0'
PRESSURE = '/mavros/imu/static_pressure'
IMU_SHIFT_NS = -1770000  # Fixed publisher launch value, no reference fitting.
NATIVE_CAM0_PREFIX = 6368


def prepare(source, output):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    isolation = json.loads((source.parent/'result.json').read_text())
    if isolation['derived_sha256'] != sha(source):
        raise ValueError('sensor isolation hash mismatch')
    if isolation['verification']['status'] != 'every_selected_packet_byte_verified':
        raise ValueError('native byte verification missing')
    plan = dict(schema='ntnu-full-mono-input-v1', source=str(source), source_sha256=sha(source),
        isolation_result_sha256=sha(source.parent/'result.json'),
        converter_sha256=sha(Path(__file__)), camera='native cam0 only; cam1 retained but excluded from this mono profile',
        camera_time_shift_ns=0, imu_time_shift_ns=IMU_SHIFT_NS,
        prefix='all 6368 native cam0 packets; all IMU/pressure included, cam1 explicitly outside mono profile; fixed before full trials and reference access',
        native_cam0_prefix=NATIVE_CAM0_PREFIX, imu_lookahead_ns=105000000, end_of_stream='complete native acquisition, SDK wait disabled only after all events; no extrapolation',
        pressure_profile=dict(depth='-(raw - 2660)/241', provenance='pinned publisher ReAqROVIO pressure callback; no SI conversion inferred'),
        timestamp_contract='integer native header timestamps plus fixed IMU correction; no camera rounding, interpolation or retiming',
        reference_values_read=False, evaluation_ready=False)
    new_json(output/'plan.json', plan)
    shutil.copyfile(__file__, output/'prepare_ntnu_full.py')
    ts = get_typestore(Stores.ROS1_NOETIC)
    counts = collections.Counter(); previous = {}; imus = []; observations = []; records = []
    blobpath = output/'native-cam0-prefix.bin'
    with Reader(source) as reader, blobpath.open('xb') as blob:
        chosen = select(reader)
        if len(chosen) != len(reader.connections): raise ValueError('withheld connection in sensor input')
        for c, bag_ns, raw in reader.messages(connections=chosen):
            m = ts.deserialize_ros1(raw, c.msgtype)
            stamp = int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)
            ordinal = counts[c.topic]; counts[c.topic] += 1
            if c.topic in previous and stamp <= previous[c.topic]:
                raise ValueError('non-increasing native sensor header; artifacts retained')
            previous[c.topic] = stamp
            effective = stamp + (IMU_SHIFT_NS if c.topic == IMU else 0)
            rec = dict(topic=c.topic, ordinal=ordinal, bag_ns=bag_ns, header_ns=stamp,
                effective_ns=effective, raw_sha256=hashlib.sha256(raw).hexdigest(), frame=m.header.frame_id)
            if c.topic == IMU:
                acc = m.linear_acceleration; gyr = m.angular_velocity
                values = [acc.x,acc.y,acc.z,gyr.x,gyr.y,gyr.z]
                if not all(math.isfinite(v) for v in values): raise ValueError('nonfinite native IMU')
                imus.append((effective,ordinal,values))
            elif c.topic == PRESSURE:
                rec.update(raw_pressure=float(m.fluid_pressure), variance=float(m.variance))
                observations.append((effective,'P',ordinal,float(m.fluid_pressure)))
            else:
                if (m.width,m.height,m.encoding,m.step,len(m.data)) != (720,540,'mono8',720,388800):
                    raise ValueError('native image shape/encoding differs from registered profile')
                rec.update(width=int(m.width),height=int(m.height),encoding=m.encoding)
                if c.topic == CAM and ordinal < NATIVE_CAM0_PREFIX:
                    offset = blob.tell(); blob.write(m.data.tobytes())
                    observations.append((effective,'C',ordinal,offset))
            records.append(rec)
    if counts[CAM] != NATIVE_CAM0_PREFIX: raise ValueError('insufficient prefix; no smaller window substitution')
    camera = [o for o in observations if o[1] == 'C']
    cutoff = max(o[0] for o in observations)  # Include pressure through native end, including support exclusions.
    first, last = imus[0][0], imus[-1][0]
    reasons = collections.Counter()
    with (output/'native-packet-ledger.jsonl').open('x') as ledger:
        for r in records:
            t = r['effective_ns']; topic = r['topic']
            if topic == '/alphasense_driver_ros/cam1': reason = 'outside_registered_mono_camera_profile'
            elif t > cutoff + (105000000 if topic == IMU else 0): reason = 'outside_registered_full_sensor_extent'
            elif topic == CAM and r['ordinal'] >= NATIVE_CAM0_PREFIX: reason = 'outside_registered_full_sensor_extent'
            elif topic == CAM and not first <= t <= last: reason = 'outside_native_corrected_imu_support'
            elif topic == PRESSURE and not first <= t <= last: reason = 'retained_pressure_outside_native_imu_support'
            else: reason = 'retained_for_replay'
            r['disposition'] = reason; reasons[(topic,reason)] += 1
            ledger.write(json.dumps(r,allow_nan=False)+'\n')
    observations = sorted(o for o in observations if o[0] <= cutoff and
                          (o[1] != 'C' or first <= o[0] <= last))
    times = [i[0] for i in imus]
    cursor = 0; wire_counts = collections.Counter()
    with (output/'input.bin').open('xb') as wire, blobpath.open('rb') as blob:
        wire.write(b'AQMONO01'+struct.pack('<qq',first,last))
        for stamp,kind,ordinal,payload in observations:
            stop = bisect.bisect_right(times,stamp+105000000)
            for t,idx,values in imus[cursor:stop]:
                wire.write(struct.pack('<c q Q 6d',b'I',t,idx,*values)); wire_counts['I'] += 1
            cursor = max(cursor,stop)
            wire.write(struct.pack('<c q Q',kind.encode(),stamp,ordinal)); wire_counts[kind] += 1
            if kind == 'P': wire.write(struct.pack('<d',payload))
            else:
                blob.seek(payload); data = blob.read(388800)
                if len(data) != 388800: raise ValueError('truncated native image spool')
                wire.write(struct.pack('<III',720,540,388800)+data)
    if sha(source) != plan['source_sha256'] or sha(Path(__file__)) != plan['converter_sha256']:
        raise ValueError('source changed while preparing; no readiness')
    # The spool is redundant; its bytes are preserved in the verified wire and source bag.
    blobpath.unlink()
    result = dict(schema='ntnu-mono-input-result-v1', plan_sha256=sha(output/'plan.json'),
        native_sensor_counts=dict(counts), native_imu_support_ns=[first,last],
        fixed_prefix_end_ns=cutoff, wire_counts=dict(wire_counts),
        reasons=[dict(topic=k[0],reason=k[1],count=v) for k,v in reasons.items()],
        wire_sha256=sha(output/'input.bin'), ledger_sha256=sha(output/'native-packet-ledger.jsonl'),
        reference_values_read=False, evaluation_ready=False)
    new_json(output/'result.json',result)
    return result

if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--source',required=True); p.add_argument('--output',required=True)
    a = p.parse_args(); print(json.dumps(prepare(a.source,a.output),indent=2))
