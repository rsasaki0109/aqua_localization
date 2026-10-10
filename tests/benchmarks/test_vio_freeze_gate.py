"""Reference access must follow immutable four-trial outcomes."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[2] / 'scripts/benchmarks'))
import audit_native_imu_support as audit
import score_vio_comparison as scoring
import prepare_vio_scoring as registration


@pytest.fixture
def frozen_trials(tmp_path, monkeypatch):
    plan = tmp_path / 'method/plan.json'
    plan.parent.mkdir()
    plan.write_text('{}')
    plan.with_name('registration.json').write_text('{}')
    trials = []
    for arm in ('vio', 'pressure'):
        for i in (1, 2):
            directory = plan.parent / 'trials' / f'{arm}-{i}'
            directory.mkdir(parents=True)
            output = directory / 'trajectory.csv'
            output.write_text('fixed output\n')
            result = {'id': f'{arm}-{i}', 'status': 'success', 'output_sha256': {'trajectory.csv': audit.sha256(output)}}
            report = directory / 'result.json'
            report.write_text(json.dumps(result))
            trials.append(dict(result, result_sha256=audit.sha256(report)))
    method = {'trials': [{'id': r['id']} for r in trials], 'source_sha256': {}, 'runtime_artifact_sha256': {}}
    monkeypatch.setattr(scoring, 'check_registration', lambda _: method)
    frozen = {'all_planned_outcomes_frozen': True, 'plan_sha256': audit.sha256(plan),
        'registration_sha256': audit.sha256(plan.with_name('registration.json')),
        'source_sha256': {}, 'runtime_artifact_sha256': {}, 'trials': trials}
    plan.with_name('freeze.json').write_text(json.dumps(frozen))
    return plan


def test_frozen_output_mutation_blocks_before_reference_client_creation(frozen_trials, tmp_path, monkeypatch):
    method_plan = frozen_trials
    (method_plan.parent / 'trials/vio-1/trajectory.csv').write_text('modified output\n')
    folder = tmp_path / 'scoring'
    folder.mkdir()
    repo = Path(__file__).parents[2]
    hashes = {n: audit.sha256(repo / n) for n in registration.SOURCE_FILES}
    for n in hashes:
        copy = folder / 'source' / n
        copy.parent.mkdir(parents=True, exist_ok=True)
        copy.write_bytes((repo / n).read_bytes())
    plan = folder / 'plan.json'
    plan.write_text(json.dumps({'schema': 'aqualoc-freeze-gated-scoring-v3',
                               'source_sha256': hashes, 'method_plan': str(method_plan)}))
    (folder / 'registration.json').write_text(json.dumps({'scoring_plan_sha256': audit.sha256(plan)}))
    def forbidden(*args, **kwargs):
        pytest.fail('reference client created before freeze validation')
    monkeypatch.setattr(scoring.urllib.request, 'build_opener', forbidden)
    with pytest.raises(ValueError, match='frozen trial output changed'):
        scoring.run(plan)
    assert not (folder / 'scored').exists()


def test_missing_trial_cannot_be_hidden_in_freeze(frozen_trials):
    path = frozen_trials.with_name('freeze.json')
    value = json.loads(path.read_text()); value['trials'].pop(); path.write_text(json.dumps(value))
    with pytest.raises(ValueError, match='all four outcomes'):
        scoring.verify_freeze(frozen_trials)


def test_reference_image_positions_use_registered_native_clock_without_retiming(tmp_path):
    origin = 1500000000000000001
    ledger = {'records': [{'sensor': 'camera', 'native_ordinal': i,
                          'corrected_stamp_ns': origin + i*50000000} for i in range(3)]}
    reference = tmp_path / 'reference.txt'
    reference.write_text('# img_number tx ty tz qx qy qz qw\n2.0e0 2 0 0 0 0 0 1\n0 0 0 0 0 0 0 1\n')
    times, positions = scoring.load_reference(reference, ledger, 0)
    assert times == [origin, origin+100000000]
    np.testing.assert_array_equal(positions, [[0,0,0], [2,0,0]])
    with pytest.raises(ValueError, match='zero-based'):
        scoring.load_reference(reference, ledger, 1)
    reference.write_text('3 0 0 0 0 0 0 1\n')
    with pytest.raises(ValueError, match='outside native sequence'):
        scoring.load_reference(reference, ledger, 0)
