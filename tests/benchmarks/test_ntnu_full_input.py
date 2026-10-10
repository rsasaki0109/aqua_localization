"""Full native accounting includes unsupported pressure at both boundaries."""
import json
from pathlib import Path
import sys
from rosbags.rosbag1 import Reader,Writer
from rosbags.typesys import Stores,get_typestore
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/benchmarks'))
import ntnu_sensor_only as isolation
import prepare_ntnu_full as prepare
import verify_ntnu_full as verify
from test_ntnu_mono_input import fixture


def test_all_native_pressure_retained_including_end_support_exclusion(tmp_path,monkeypatch):
    source,_=fixture(tmp_path,monkeypatch)
    store=get_typestore(Stores.ROS1_NOETIC); T=store.types
    native=tmp_path/'extended.bag'
    with Reader(source) as r, Writer(native) as w:
        connections={c.id:w.add_connection(c.topic,c.msgtype,msgdef=c.msgdef.data,md5sum=c.digest) for c in r.connections}
        for c,t,raw in r.messages():w.write(connections[c.id],t,raw)
        c=next(c for c in r.connections if c.topic==prepare.PRESSURE)
        header=T['std_msgs/msg/Header'](0,T['builtin_interfaces/msg/Time'](1,350000000),'fixture')
        pressure=T['sensor_msgs/msg/FluidPressure'](header,2670.,0.)
        w.write(connections[c.id],1350000000,store.serialize_ros1(pressure,c.msgtype))
    isolated=tmp_path/'extended-isolated';isolation.convert(native,isolated)
    monkeypatch.setattr(prepare,'NATIVE_CAM0_PREFIX',3)
    out=tmp_path/'full-v1';result=prepare.prepare(isolated/'sensor_only.bag',out)
    assert result['wire_counts']=={'I':61,'P':5,'C':2}
    assert result['native_sensor_counts'][prepare.CAM]==3
    assert any(r['reason']=='retained_pressure_outside_native_imu_support' and r['count']==2 for r in result['reasons'])
    assert any(r['reason']=='outside_native_corrected_imu_support' and r['count']==1 for r in result['reasons'])
    assert verify.verify(isolated/'sensor_only.bag',out)['verified_counts']==result['wire_counts']
    ledger=[json.loads(x) for x in (out/'native-packet-ledger.jsonl').read_text().splitlines()]
    last=next(r for r in ledger if r['topic']==prepare.PRESSURE and r['ordinal']==4)
    assert last['header_ns']==1350000000 and last['disposition']=='retained_pressure_outside_native_imu_support'
