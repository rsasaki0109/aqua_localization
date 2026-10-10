"""Record invalid absolute-pressure measurements while retaining every raw packet."""
import hashlib
import math
from pathlib import Path

from audit_native_imu_support import PRESSURE_TOPIC, sha256


def classify_pressure(pa):
    if not math.isfinite(pa):
        return 'nonfinite_pressure'
    if pa < 0:
        return 'negative_absolute_pressure'
    if pa < 80000:
        return 'below_registered_underwater_absolute_pressure_range'
    if pa > 1e8:
        return 'above_registered_absolute_pressure_range'
    return None


def audit_pressure_quality(bag):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS1_NOETIC)
    records = []
    with Reader(Path(bag)) as reader:
        for ordinal, (c, ns, raw) in enumerate(reader.messages(connections=[c for c in reader.connections if c.topic == PRESSURE_TOPIC])):
            m = store.deserialize_ros1(raw, c.msgtype)
            value = float(m.fluid_pressure)
            reason = classify_pressure(value)
            records.append({'native_pressure_ordinal': ordinal, 'timestamp_ns': ns,
                'pressure_pa': value if math.isfinite(value) else None,
                'nonfinite_value_repr': str(value) if not math.isfinite(value) else None,
                'source_message_sha256': hashlib.sha256(raw).hexdigest(),
                'measurement_eligible': reason is None, 'exclusion_reason': reason})
    return {'schema': 'pressure-value-quality-v1', 'input_sha256': sha256(bag),
            'native_pressure_count': len(records),
            'measurement_eligible_count': sum(r['measurement_eligible'] for r in records),
            'measurement_excluded_count': sum(not r['measurement_eligible'] for r in records),
            'policy': 'finite absolute pressure in Pa within [80000,100000000]; no unit guessing',
            'raw_pressure_messages_and_timestamps_retained': True,
            'imu_support_exclusions_are_separate': True, 'records': records,
            'reference_access': 'not_performed'}
