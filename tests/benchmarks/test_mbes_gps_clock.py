import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts/benchmarks"))
from types import SimpleNamespace as N
from mbes_hardware_score_v2 import gps_timestamp_ns, ns


def test_quality_uses_native_gps_epoch_not_receive_header():
    m=N(header=N(stamp=N(sec=1580658965,nanosec=483163585)),
        timestamp=N(sec=1580658965,nanosec=400218390))
    fix=N(stamp=N(sec=1580658965,nanosec=400218390))
    assert ns(m.header)-ns(fix)==82945195
    assert gps_timestamp_ns(m)==ns(fix)
