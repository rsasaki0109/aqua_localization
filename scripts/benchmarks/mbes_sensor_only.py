"""Registered MBES input isolation; never deserialize withheld navigation values."""
import argparse
import collections
import hashlib
import itertools
import json
from pathlib import Path
from rosbags.rosbag1 import Reader, Writer

ALLOWED = {'/norbit/detections': 'sensor_msgs/msg/PointCloud2',
           '/nav/sensors/microstrain/imu/raw': 'sensor_msgs/msg/Imu'}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''): h.update(b)
    return h.hexdigest()


def new_json(path, value):
    with path.open('x') as f: json.dump(value, f, indent=2); f.write('\n')


def select(reader):
    chosen = [c for c in reader.connections if c.topic in ALLOWED]
    if {c.topic for c in chosen} != set(ALLOWED): raise ValueError('required raw sensors missing')
    if any(c.msgtype != ALLOWED[c.topic] for c in chosen): raise ValueError('sensor type mismatch')
    return chosen


def convert(native, target):
    native, target = Path(native), Path(target)
    target.mkdir(exist_ok=False, parents=True)
    source = Path(__file__).resolve()
    registration = dict(schema='mbes-sensor-isolation-plan-v1', native=str(native),
        native_sha256=sha(native), source_sha256=sha(source), allowed_topics=ALLOWED,
        timestamps='unchanged native bag timestamps and raw message header bytes',
        excluded='all topics outside allowlist; includes GNSS, fused/projected navigation, processed AHRS and all TF',
        reference_values_read=False, estimator_runs='not_executed',
        evaluation_ready=False, sequence_role='beach_pond previously used development, not new held-out evidence')
    new_json(target/'plan.json', registration)
    with (target/source.name).open('xb') as f: f.write(source.read_bytes())
    counts = collections.Counter()
    with Reader(native) as reader, Writer(target/'sensor_only.bag') as writer:
        chosen = select(reader)
        metadata = [{'topic':c.topic,'type':c.msgtype,'count':c.msgcount,
                     'included':c in chosen,'reason':'registered raw sensor' if c in chosen else 'outside registered sensor allowlist'}
                    for c in reader.connections]
        connections = {c.id: writer.add_connection(c.topic,c.msgtype,msgdef=c.msgdef.data,md5sum=c.digest)
                       for c in chosen}
        for connection, stamp, raw in reader.messages(connections=chosen):
            writer.write(connections[connection.id],stamp,raw); counts[connection.topic]+=1
    verification = verify(native, target/'sensor_only.bag')
    if sha(native) != registration['native_sha256'] or sha(source) != registration['source_sha256']:
        raise ValueError('native/source changed during conversion; artifacts preserved')
    result = dict(schema='mbes-sensor-isolation-result-v1', plan_sha256=sha(target/'plan.json'),
        native_topic_metadata=metadata, selected_counts=dict(counts), verification=verification,
        derived_sha256=sha(target/'sensor_only.bag'), reference_values_read=False,
        evaluation_ready=False, remaining_gates=['extrinsic/frame/clock provenance',
        'hardware RTK quality and antenna lever arm', 'competitive method registration', 'new held-out sequence'])
    new_json(target/'result.json',result)
    return result


def verify(native, derived):
    with Reader(native) as a, Reader(derived) as b:
        selected = select(a); select(b)
        if any(c.topic not in ALLOWED for c in b.connections): raise ValueError('withheld topic leaked into estimator input')
        count = 0
        for left,right in itertools.zip_longest(a.messages(connections=selected),b.messages()):
            if left is None or right is None: raise ValueError('sensor packet count mismatch')
            ca,ta,ra=left; cb,tb,rb=right
            if ca.topic!=cb.topic or ca.msgtype!=cb.msgtype or ta!=tb or ra!=rb:
                raise ValueError('sensor bytes/time/order changed')
            count+=1
        return {'status':'every_selected_packet_byte_verified','count':count,'withheld_values_read':False}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--native',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();print(json.dumps(convert(args.native,args.output),indent=2))
