"""Native-pressure defects must remain visible; no localization accuracy claim."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).parents[2]/'scripts/benchmarks'))
from audit_pressure_quality import classify_pressure, audit_pressure_quality
from test_harbor05_acquisition import tiny_native_bag
import audit_native_imu_support as audit


@pytest.mark.parametrize('value,reason',[
    (float('nan'),'nonfinite_pressure'),(float('inf'),'nonfinite_pressure'),
    (-2585590.,'negative_absolute_pressure'),
    (79999.,'below_registered_underwater_absolute_pressure_range'),
    (80000.,None),(134920.,None),(1e8,None),
    (1e8+1,'above_registered_absolute_pressure_range')])
def test_registered_physical_pressure_bounds(value,reason):
    assert classify_pressure(value)==reason


def test_negative_native_pressure_packet_keeps_time_bytes_hash_and_reason(tmp_path):
    from rosbags.rosbag1 import Reader,Writer
    from rosbags.typesys import Stores,get_typestore
    source,bad=tmp_path/'source.bag',tmp_path/'bad.bag'
    tiny_native_bag(source)
    store=get_typestore(Stores.ROS1_NOETIC)
    expected=None
    with Reader(source) as reader,Writer(bad) as writer:
        connections={c.id:writer.add_connection(c.topic,c.msgtype,typestore=store) for c in reader.connections}
        for c,ns,raw in reader.messages():
            if c.topic==audit.PRESSURE_TOPIC:
                msg=store.deserialize_ros1(raw,c.msgtype);msg.fluid_pressure=-2585590.
                raw=store.serialize_ros1(msg,c.msgtype)
                import hashlib
                expected=(ns,hashlib.sha256(raw).hexdigest())
            writer.write(connections[c.id],ns,raw)
    before=bad.read_bytes()
    result=audit_pressure_quality(bad)
    assert bad.read_bytes()==before
    assert result['native_pressure_count']==result['measurement_excluded_count']==1
    row=result['records'][0]
    assert (row['timestamp_ns'],row['source_message_sha256'])==expected
    assert row['native_pressure_ordinal']==0 and row['pressure_pa']==-2585590.
    assert row['exclusion_reason']=='negative_absolute_pressure'
    assert result['raw_pressure_messages_and_timestamps_retained'] is True
