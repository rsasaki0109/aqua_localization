"""Boundary handling, native byte verification and immutable input registration."""
import json
from pathlib import Path
import sys
import numpy as np
import pytest
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores,get_typestore

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/benchmarks'))
import ntnu_sensor_only as isolation
import prepare_ntnu_mono as prepare
import verify_ntnu_mono as verify


def fixture(tmp_path,monkeypatch,duplicate=False):
    monkeypatch.setattr(prepare,'NATIVE_CAM0_PREFIX',2)
    store=get_typestore(Stores.ROS1_NOETIC); T=store.types
    def header(ns):
        return T['std_msgs/msg/Header'](0,T['builtin_interfaces/msg/Time'](ns//10**9,ns%10**9),'fixture')
    records=[]
    for ns in range(1000000000,1300000001,5000000):
        m=T['sensor_msgs/msg/Imu'](header(ns),T['geometry_msgs/msg/Quaternion'](0.,0.,0.,1.),np.zeros(9),
            T['geometry_msgs/msg/Vector3'](0.1,0.2,0.3),np.zeros(9),T['geometry_msgs/msg/Vector3'](0.,0.,9.81),np.zeros(9))
        records.append((prepare.IMU,ns,m))
    for topic,times in [(prepare.CAM,[970000000,1010000000,1060000000]),('/alphasense_driver_ros/cam1',[1020000000])]:
        for ordinal,ns in enumerate(times):
            actual=970000000 if duplicate and ordinal==1 else ns
            m=T['sensor_msgs/msg/Image'](header(actual),540,720,'mono8',0,720,np.full(388800,ordinal,np.uint8))
            records.append((topic,ns,m))
    for ns in [980000000,1000000000,1020000000,1100000000]:
        records.append((prepare.PRESSURE,ns,T['sensor_msgs/msg/FluidPressure'](header(ns),2660.5,0.)))
    native=tmp_path/'native.bag'
    with Writer(native) as w:
        connections={topic:w.add_connection(topic,typ,typestore=store) for topic,typ in isolation.ALLOWED.items()}
        for topic,ns,m in sorted(records,key=lambda x:x[1]):
            w.write(connections[topic],ns,store.serialize_ros1(m,isolation.ALLOWED[topic]))
    root=tmp_path/'isolated';isolation.convert(native,root)
    return root/'sensor_only.bag',tmp_path/'mono-v1'


def test_explicit_boundary_and_full_native_counts(tmp_path,monkeypatch):
    source,out=fixture(tmp_path,monkeypatch)
    result=prepare.prepare(source,out)
    assert result['native_sensor_counts'][prepare.CAM]==3
    assert result['wire_counts']['C']==1
    assert result['wire_counts']['P']==2
    assert any(r['reason']=='outside_native_corrected_imu_support' and r['count']==1 for r in result['reasons'])
    assert any(r['reason']=='retained_pressure_outside_native_imu_support' and r['count']==1 for r in result['reasons'])
    assert verify.verify(source,out)['verified_counts']==result['wire_counts']
    assert result['reference_values_read'] is False
    with pytest.raises(FileExistsError):prepare.prepare(source,out)


def test_independent_native_check_rejects_rehashed_image_corruption(tmp_path,monkeypatch):
    source,out=fixture(tmp_path,monkeypatch);prepare.prepare(source,out)
    path=out/'input.bin';data=bytearray(path.read_bytes());data[-1]^=1;path.write_bytes(data)
    result=json.loads((out/'result.json').read_text());result['wire_sha256']=isolation.sha(path)
    (out/'result.json').write_text(json.dumps(result))
    with pytest.raises(ValueError,match='does not match native'):verify.verify(source,out)


def test_duplicate_native_time_is_not_silently_repaired(tmp_path,monkeypatch):
    source,out=fixture(tmp_path,monkeypatch,duplicate=True)
    with pytest.raises(ValueError,match='non-increasing native'):prepare.prepare(source,out)
    assert (out/'plan.json').exists()
    assert not (out/'result.json').exists()
