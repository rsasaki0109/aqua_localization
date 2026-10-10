"""Protocol and freeze gates only, not synthetic accuracy claims."""
import io
import json
from pathlib import Path
import struct
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[2] / 'scripts/benchmarks'))
import run_reconstructed_openvins as runner
import audit_native_imu_support as audit
import openvins_supported_bag as supported
from test_harbor05_acquisition import tiny_native_bag


def full_geometry_bag(source, destination):
    from rosbags.rosbag1 import Reader, Writer
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(source) as reader, Writer(destination) as writer:
        connections = {c.id: writer.add_connection(c.topic, c.msgtype, typestore=store) for c in reader.connections}
        for c, ns, raw in reader.messages():
            if c.topic == audit.CAMERA_TOPIC:
                m = store.deserialize_ros1(raw, c.msgtype)
                m.width, m.height, m.step = 640, 512, 640
                m.data = np.full(640 * 512, 42, dtype=np.uint8)
                raw = store.serialize_ros1(m, c.msgtype)
            writer.write(connections[c.id], ns, raw)


def test_stream_preserves_exact_sensor_times_ordinals_pixels_and_pressure(tmp_path):
    small, native = tmp_path / 'small.bag', tmp_path / 'native.bag'
    tiny_native_bag(small)
    full_geometry_bag(small, native)
    output = tmp_path / 'v2'
    supported.convert_supported_bag(native, output)
    ledger = json.loads((output / 'input_audit.json').read_text())
    sink = io.BytesIO()
    runner.stream_input(output / 'supported.bag', ledger, sink)
    content = sink.getvalue()
    assert content[:8] == b'AQUAVIO3'
    origin, count = struct.unpack_from('<qI', content, 8)
    assert origin == 100000000000 and count == 3
    imus = [struct.unpack_from('<q6d', content, 20 + i * 56) for i in range(count)]
    assert [r[0] for r in imus] == [origin, origin + 50000000, origin + 100000000]
    assert all(r[-1] == 9.80665 for r in imus)
    at = 20 + count * 56
    assert content[at:at+1] == b'P'
    assert struct.unpack_from('<qId', content, at+1) == (origin+1, 0, 101325.)
    at += 21
    assert content[at:at+1] == b'C'
    assert struct.unpack_from('<qIIII', content, at+1) == (origin+80000000-40380655, 1, 640, 512, 640*512)
    assert content[at+25:] == bytes([42]) * (640*512)
    assert len(content) == at+25+640*512


def test_stream_does_not_resize_unregistered_image_geometry(tmp_path):
    native = tmp_path / 'native.bag'
    tiny_native_bag(native)
    supported.convert_supported_bag(native, tmp_path / 'v2')
    ledger = json.loads((tmp_path / 'v2/input_audit.json').read_text())
    with pytest.raises(ValueError, match='encoding/geometry'):
        runner.stream_input(tmp_path / 'v2/supported.bag', ledger, io.BytesIO())


def test_incomplete_method_cannot_bypass_four_trial_freeze_registration(tmp_path):
    plan = tmp_path / 'plan.json'
    plan.write_text(json.dumps({'comparison_executable': True, 'source_sha256': {}, 'runtime_artifact_sha256': {}}))
    scoring = tmp_path / 'scoring-plan.json'
    scoring.write_text('{}')
    (tmp_path / 'registration.json').write_text(json.dumps({'plan_sha256': audit.sha256(plan),
                                                          'scoring_plan_sha256': audit.sha256(scoring)}))
    with pytest.raises(ValueError, match='complete new method/source/four-trial'):
        runner.check_registration(plan)
