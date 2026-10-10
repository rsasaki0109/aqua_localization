"""Known geometry and timing checks, not localization accuracy evidence."""
from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).parents[2] / 'scripts/benchmarks'))
from vio_metrics import correspond, rigid_score
from check_vio_scores import independent_correspond, horn_score


def test_independent_horn_and_kabsch_recover_known_rigid_transform():
    source = np.array([[0,0,0], [1,0,0], [0,2,0], [0,0,3], [1,2,3]], dtype=float)
    rotation = np.array([[0,-1,0], [1,0,0], [0,0,1]])
    translation = np.array([2., -3., 4.])
    target = source @ rotation.T + translation
    for result in [rigid_score(source, target), horn_score(source, target)]:
        np.testing.assert_allclose(result['rotation'], rotation, atol=1e-12)
        np.testing.assert_allclose(result['translation_m'], translation, atol=1e-12)
        assert result['rmse_m'] < 1e-12 and result['scale'] == 1


def test_reflection_and_scale_cannot_be_optimized_away():
    source = np.array([[0,0,0], [1,0,0], [0,2,0], [0,0,3], [1,2,3]], dtype=float)
    for target in [source * np.array([-1,1,1]), source * 2]:
        result = rigid_score(source, target)
        independent = horn_score(source, target)
        assert np.linalg.det(np.array(result['rotation'])) > .999999
        assert result['rmse_m'] > .1
        np.testing.assert_allclose(result['rmse_m'], independent['rmse_m'], atol=1e-12)


def test_integer_epoch_exact_gap_boundary_and_no_extrapolation():
    origin = 1_500_000_000_000_000_001
    refs = [origin, origin+100000000, origin+200000001]
    positions = np.array([[0,0,0], [1,0,0], [2,0,0]], dtype=float)
    times = [origin-1, origin, origin+1, origin+50000000, origin+100000000,
             origin+150000000, origin+200000001, origin+200000002]
    source = np.zeros((len(times),3))
    primary = correspond(times, source, refs, positions)
    independent = independent_correspond(times, source, refs, positions)
    assert primary[0] == independent[0] == [times[i] for i in [1,2,3,4,6]]
    np.testing.assert_array_equal(primary[2], independent[2])
    assert primary[2][1,0] == 1e-8
    assert [r['reason'] for r in primary[3]] == ['outside_reference_support',
        'reference_interpolation_interval_exceeds_0.1_s', 'outside_reference_support']


def test_degenerate_alignment_fails_explicitly():
    source = np.array([[0,0,0], [1,0,0], [2,0,0]], dtype=float)
    with pytest.raises(ValueError, match='degenerate'):
        rigid_score(source, source)
