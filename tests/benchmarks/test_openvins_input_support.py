import importlib.util
from pathlib import Path
import sys

import pytest

MODULE = Path(__file__).parents[2] / "scripts/benchmarks/openvins_input_support.py"
spec = importlib.util.spec_from_file_location("support", MODULE)
support = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = support
spec.loader.exec_module(support)
Observation = support.Observation


def test_reported_first_camera_gap_preserves_ordinal_and_offset():
    # Relative timestamps reconstruct the reported boundary only, not real data.
    source = [Observation("camera", 0, 33842804, "source-hash"),
              Observation("pressure", 0, 0), Observation("camera", 1, 80000000)]
    result = support.audit_support([0, 100000000], source)
    excluded = result["records"][0]
    assert excluded["corrected_stamp_ns"] == -6537851
    assert excluded["distance_outside_support_ns"] == 6537851
    assert excluded["exclusion_reason"] == "before_native_imu_support"
    assert excluded["native_ordinal"] == 0
    assert excluded["source_message_sha256"] == "source-hash"
    assert result["native_observation_counts"] == {"camera": 2, "pressure": 1}
    assert result["eligible_observation_counts"] == {"camera": 1, "pressure": 1}
    assert result["camera_offset_ns"] == -40380655
    assert len(source) == len(result["records"]) == 3
    assert source[0].native_stamp_ns == 33842804
    assert excluded["native_imu_bracket_ordinals"] is None
    assert result["records"][1]["native_imu_bracket_ordinals"] == [0, 0]
    assert result["records"][2]["native_imu_bracket_ordinals"] == [0, 1]


def test_closed_boundaries_and_end_exclusion():
    result = support.audit_support([10, 20], [Observation("camera", 0, 10),
        Observation("camera", 1, 20), Observation("camera", 2, 21)], camera_offset_ns=0)
    assert [r["included"] for r in result["records"]] == [True, True, False]
    assert result["records"][2]["exclusion_reason"] == "after_native_imu_support"
    assert result["records"][2]["distance_outside_support_ns"] == 1
    assert [r["native_imu_bracket_ordinals"] for r in result["records"]] == [[0, 0], [1, 1], None]


def test_nanosecond_bracketing_preserves_precision_at_large_epoch():
    epoch = 1_700_000_000_000_000_000
    result = support.audit_support([epoch, epoch + 10, epoch + 20],
        [Observation("pressure", 0, epoch + 1),
         Observation("camera", 0, epoch + 20 - support.CAMERA_OFFSET_NS)])
    first, second = result["records"]
    assert first["native_imu_bracket_ordinals"] == [0, 1]
    assert first["native_imu_bracket_stamps_ns"] == [epoch, epoch + 10]
    assert second["corrected_stamp_ns"] == epoch + 20
    assert second["native_imu_bracket_ordinals"] == [2, 2]


def test_pressure_uses_native_time_and_is_audited_independently():
    result = support.audit_support([100, 200], [Observation("pressure", 0, 99),
        Observation("pressure", 1, 150)], camera_offset_ns=-999999)
    assert [r["corrected_stamp_ns"] for r in result["records"]] == [99, 150]
    assert result["excluded_observation_counts"]["pressure"] == 1
    assert result["camera_mask_shared_by_both_arms"] is True


@pytest.mark.parametrize("imu", [[], [1], [2, 1], [1, 1], [1.0, 2]])
def test_invalid_native_imu_fails_without_fabricating_support(imu):
    with pytest.raises(ValueError):
        support.audit_support(imu, [])


@pytest.mark.parametrize("observations", [
    [Observation("camera", 1, 10)],
    [Observation("camera", 0, 10), Observation("camera", 0, 20)],
    [Observation("camera", 0, 20), Observation("camera", 1, 10)],
    [Observation("camera", 0, 10), Observation("camera", 1, 10)],
    [Observation("other", 0, 10)], [Observation("camera", 0, 10.0)],
])
def test_missing_ordinals_or_ambiguous_timestamps_fail(observations):
    with pytest.raises(ValueError):
        support.audit_support([0, 100], observations)
