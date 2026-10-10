"""Isolation tests: withheld packet bodies must not enter the estimator bag."""
import importlib.util
from pathlib import Path
import pytest
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

spec=importlib.util.spec_from_file_location('mbes_isolation',Path(__file__).resolve().parents[2]/'scripts/benchmarks/mbes_sensor_only.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def make_bag(path, missing=False):
    store=get_typestore(Stores.ROS1_NOETIC)
    with Writer(path) as w:
        topics=dict(m.ALLOWED)
        if missing: topics.pop('/norbit/detections')
        topics['/nav/sensors/navsat/ubx_pos/fix']='sensor_msgs/msg/NavSatFix'
        topics['/nav/processed/odometry']='nav_msgs/msg/Odometry'
        for i,(topic,typ) in enumerate(topics.items()):
            c=w.add_connection(topic,typ,typestore=store)
            # Deliberately not decodable: converter/verifier must copy selected
            # bytes verbatim and never deserialize withheld position payloads.
            w.write(c,100+i,topic.encode()+b' sentinel')


def test_withholds_position_and_navigation_bytes(tmp_path):
    source=tmp_path/'native.bag';make_bag(source)
    result=m.convert(source,tmp_path/'input-v1')
    assert result['verification']['count']==2
    assert result['reference_values_read'] is False
    assert result['evaluation_ready'] is False
    with Reader(tmp_path/'input-v1/sensor_only.bag') as r:
        assert {c.topic for c in r.connections}==set(m.ALLOWED)


def test_refuses_existing_registration(tmp_path):
    source=tmp_path/'native.bag';make_bag(source)
    m.convert(source,tmp_path/'input-v1')
    with pytest.raises(FileExistsError):m.convert(source,tmp_path/'input-v1')


def test_missing_sensor_does_not_establish_readiness(tmp_path):
    source=tmp_path/'native.bag';make_bag(source,True)
    with pytest.raises(ValueError,match='required raw sensors missing'):m.convert(source,tmp_path/'input-v1')
    assert not (tmp_path/'input-v1/result.json').exists()


def test_verifier_rejects_navigation_leakage(tmp_path):
    source=tmp_path/'native.bag';make_bag(source)
    with pytest.raises(ValueError,match='withheld topic leaked'):m.verify(source,source)
