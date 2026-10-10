"""New freeze-gated scorer; never modifies a registered estimator or its output."""
import argparse
import csv
from decimal import Decimal
import json
import math
from pathlib import Path
import ssl
import urllib.parse
import urllib.request

import numpy as np
from audit_native_imu_support import sha256, write_new_json
from ensure_harbor05_data import HTTPSOnlyRedirect
from run_reconstructed_openvins import check_registration
from vio_metrics import correspond, rigid_score
from check_vio_scores import verify as independent_verify, horn_score


def verify_freeze(method_plan_path):
    method_plan_path = Path(method_plan_path)
    method = check_registration(method_plan_path)
    frozen = json.loads(method_plan_path.with_name('freeze.json').read_text())
    if (frozen.get('all_planned_outcomes_frozen') is not True or
        frozen['plan_sha256'] != sha256(method_plan_path) or
        frozen['registration_sha256'] != sha256(method_plan_path.with_name('registration.json')) or
        frozen['source_sha256'] != method['source_sha256'] or
        frozen['runtime_artifact_sha256'] != method['runtime_artifact_sha256'] or
        [r['id'] for r in frozen['trials']] != [r['id'] for r in method['trials']]):
        raise ValueError('all four outcomes and registered hashes must be frozen before reference access')
    for result in frozen['trials']:
        if result['status'] not in ('success', 'failed'):
            raise ValueError('unexecuted outcome cannot pass the freeze gate')
        directory = method_plan_path.parent / 'trials' / result['id']
        if sha256(directory / 'result.json') != result['result_sha256']:
            raise ValueError('frozen result changed')
        original = json.loads((directory / 'result.json').read_text())
        if dict(original, result_sha256=result['result_sha256']) != result:
            raise ValueError('freeze disagrees with trial result')
        for name, digest in result['output_sha256'].items():
            if Path(name).name != name or sha256(directory / name) != digest:
                raise ValueError('frozen trial output changed')
    return method, frozen


def load_reference(path, input_audit, index_base):
    if index_base != 0:
        raise ValueError('only separately registered zero-based image-position mapping is implemented')
    clocks = {r['native_ordinal']: r['corrected_stamp_ns'] for r in input_audit['records'] if r['sensor'] == 'camera'}
    rows = {}
    for number, line in enumerate(Path(path).read_text().splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        tokens = line.split()
        if len(tokens) != 8:
            raise ValueError(f'unregistered reference format at line {number}')
        value = Decimal(tokens[0])
        if not value.is_finite() or value != value.to_integral_value():
            raise ValueError('reference image position must be an exact integer')
        index = int(value)
        if index not in clocks or index in rows:
            raise ValueError('reference image position outside native sequence or duplicated')
        values = list(map(float, tokens[1:]))
        if not all(math.isfinite(x) for x in values):
            raise ValueError('nonfinite reference data')
        rows[index] = (clocks[index], values[:3])
    ordered = [rows[k] for k in sorted(rows)]
    return [r[0] for r in ordered], np.array([r[1] for r in ordered])


def run(scoring_plan_path):
    scoring_plan_path = Path(scoring_plan_path)
    plan = json.loads(scoring_plan_path.read_text())
    registration = json.loads(scoring_plan_path.with_name('registration.json').read_text())
    if sha256(scoring_plan_path) != registration['scoring_plan_sha256']:
        raise ValueError('scoring registration changed')
    from prepare_vio_scoring import SOURCE_FILES
    if plan.get('schema') != 'aqualoc-freeze-gated-scoring-v3' or set(plan.get('source_sha256', {})) != set(SOURCE_FILES):
        raise ValueError('complete scoring source registration required before reference access')
    repo = Path(__file__).resolve().parents[2]
    for name, digest in plan['source_sha256'].items():
        if sha256(repo / name) != digest or sha256(scoring_plan_path.parent / 'source' / name) != digest:
            raise ValueError('scoring source changed after registration')
    method, frozen = verify_freeze(plan['method_plan'])
    if sha256(plan['method_plan']) != plan['method_plan_sha256']:
        raise ValueError('registered method plan changed')
    if sha256(Path(plan['method_plan']).with_name('freeze.json')) != plan['freeze_sha256']:
        raise ValueError('frozen outcomes changed')
    if plan['reference_index_base'] != 0 or plan['scale'] != 1 or plan['max_interpolation_interval_ns'] != 100000000:
        raise ValueError('unregistered scoring parameters')
    from prepare_vio_scoring import REFERENCE_URL
    if plan['reference_url'] != REFERENCE_URL or urllib.parse.urlsplit(plan['reference_url']).scheme != 'https':
        raise ValueError('registered official HTTPS reference source required')
    output = scoring_plan_path.parent / 'scored'
    output.mkdir(exist_ok=False)
    reference = output / 'reference.txt'
    write_new_json(output / 'reference_access_started.json', {'source': plan['reference_url'],
                   'freeze_sha256': plan['freeze_sha256'], 'scoring_plan_sha256': sha256(scoring_plan_path)})
    opener = urllib.request.build_opener(HTTPSOnlyRedirect(), urllib.request.HTTPSHandler(context=ssl.create_default_context()))
    with opener.open(plan['reference_url'], timeout=60) as response:
        final = urllib.parse.urlsplit(response.geturl())
        if (response.status != 200 or final.scheme != 'https' or final.hostname != urllib.parse.urlsplit(plan['reference_url']).hostname or
            int(response.headers.get('Content-Length', -1)) != plan['reference_expected_bytes']):
            raise ValueError('reference origin/status/size differs from registration')
        payload = response.read(plan['reference_expected_bytes'] + 1)
        etag = response.headers.get('ETag')
    if len(payload) != plan['reference_expected_bytes']:
        with reference.with_suffix('.txt.partial').open('xb') as f:
            f.write(payload)
        raise ValueError('reference length mismatch')
    with reference.open('xb') as f:
        f.write(payload)
    write_new_json(output / 'reference_acquisition.json', {
        'source': plan['reference_url'], 'bytes': len(payload), 'etag': etag,
        'sha256': sha256(reference), 'hash_provenance': 'this verified TLS download; not a publisher checksum',
        'freeze_sha256': plan['freeze_sha256'], 'scoring_registration_sha256': sha256(scoring_plan_path)})
    audit = json.loads(Path(method['input_audit']).read_text())
    rt, rp = load_reference(reference, audit, plan['reference_index_base'])
    evaluations, datasets = [], {}
    method_root = Path(plan['method_plan']).parent
    for trial in frozen['trials']:
        result = {'id': trial['id'], 'arm': trial['arm'], 'estimator_status': trial['status'],
                  'runtime_seconds': trial['runtime_seconds'], 'trial_support_ns': trial['support_ns'],
                  'initialization_and_missing_state_exclusions': trial['initialization_and_missing_state_exclusions']}
        if trial['status'] != 'success':
            result['scoring_status'] = 'unavailable_failed_trial'; evaluations.append(result); continue
        with (method_root / 'trials' / trial['id'] / 'trajectory.csv').open() as f:
            rows = list(csv.DictReader(f))
        times = [int(r['timestamp_ns']) for r in rows]
        positions = np.array([[float(r[k]) for k in ('x', 'y', 'z')] for r in rows])
        matched, source, target, exclusions = correspond(times, positions, rt, rp)
        result.update(matched_timestamps_ns=matched, interpolation_exclusions=exclusions,
                      estimated_camera_count=len(rows), reference_matched_count=len(matched),
                      matched_support_ns=[matched[0], matched[-1]] if matched else None)
        try:
            result['score'] = rigid_score(source, target)
            result['independent_score'] = independent_verify(result, times, positions, rt, rp)
            result['scoring_status'] = 'scored_and_independently_verified'
            datasets[trial['id']] = (matched, source, target)
        except ValueError as error:
            result['scoring_status'] = 'failed'; result['scoring_error'] = str(error)
        evaluations.append(result)
    common = {'status': 'unavailable_unless_all_four_estimates_and_scores_succeed'}
    if len(datasets) == 4:
        shared = sorted(set.intersection(*(set(v[0]) for v in datasets.values())))
        scores = {}
        for name, (stamps, source, target) in datasets.items():
            index = {ns: i for i, ns in enumerate(stamps)}
            selected = [index[ns] for ns in shared]
            primary, independent = rigid_score(source[selected], target[selected]), horn_score(source[selected], target[selected])
            for key in ('rmse_m', 'rotation', 'translation_m'):
                np.testing.assert_allclose(primary[key], independent[key], rtol=1e-8, atol=1e-8)
            scores[name] = dict(primary, independent_horn=independent)
        baseline = np.mean([scores[f'vio-{i}']['rmse_m'] for i in (1, 2)])
        pressure = np.mean([scores[f'pressure-{i}']['rmse_m'] for i in (1, 2)])
        common = dict(status='scored_and_independently_verified', count=len(shared),
                      timestamps_ns=shared, support_ns=[shared[0], shared[-1]], scores=scores,
                      baseline_mean_rmse_m=float(baseline), pressure_mean_rmse_m=float(pressure),
                      improvement_percent=float(100*(baseline-pressure)/baseline) if baseline > 0 else None)
    verify_freeze(plan['method_plan'])
    write_new_json(output / 'scores.json', {
        'schema': 'aqualoc-reconstructed-position-scores-v1', 'scale': 1., 'alignment': 'rigid SE(3)',
        'max_interpolation_interval_ns': 100000000, 'trials': evaluations, 'common_all_four': common,
        'reference_sha256': sha256(reference), 'freeze_sha256': plan['freeze_sha256'],
        'reference_limitation': 'same-image COLMAP and pressure/depth scale; not independent hardware truth',
        'original_method_reproduced': False, 'sota_achieved': False})
    summary = {k: v for k, v in common.items() if k not in ('timestamps_ns', 'scores')}
    print(json.dumps({'common_all_four': summary, 'scores_sha256': sha256(output / 'scores.json')}, sort_keys=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    args = parser.parse_args()
    try:
        run(args.plan)
    except BaseException as error:
        if args.plan.parent.exists() and not (args.plan.parent / 'scoring_failure.json').exists():
            write_new_json(args.plan.parent / 'scoring_failure.json', {'status': 'failed',
                'type': type(error).__name__, 'error': str(error),
                'reference_access_begun': (args.plan.parent / 'scored/reference_access_started.json').exists(),
                'reference_file_retained': (args.plan.parent / 'scored/reference.txt').exists()})
        raise
