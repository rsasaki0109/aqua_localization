"""Register a scoring-only version against immutable existing four-trial outcomes."""
import argparse
from decimal import Decimal
from datetime import datetime, timezone
import json
from pathlib import Path

from audit_native_imu_support import sha256, write_new_json
from score_vio_comparison import verify_freeze

SOURCE_FILES = ['scripts/benchmarks/' + n for n in (
    'prepare_vio_scoring.py', 'score_vio_comparison.py', 'vio_metrics.py', 'check_vio_scores.py')]
REFERENCE_URL = 'https://seafile.lirmm.fr/d/79b03788f29148ca84e5/files/?p=%2FHarbor_sites_sequences%2Fharbor_groundtruth_files%2Fnew_harbor_colmap_traj_sequence_05.txt&dl=1'


def register(method_plan_path, proof_path, version='vio-scoring-v3'):
    method_plan_path, proof_path = Path(method_plan_path), Path(proof_path)
    method, frozen = verify_freeze(method_plan_path)
    # Diagnose invalid input/initialization while harbor_05 is still unreferenced.
    if any(r['status'] != 'success' for r in frozen['trials']):
        raise ValueError('one or more trials failed; outcomes retained; diagnose before harbor_05 reference access')
    proof = json.loads(proof_path.read_text())
    if (proof.get('schema') != 'aqualoc-index-origin-proof-v1' or proof.get('index_base') != 0 or
        proof.get('estimator_freeze_sha256') != sha256(method_plan_path.with_name('freeze.json')) or
        proof.get('source_dataset') != 'harbor_06_development' or
        proof.get('source_file_sha256') != sha256(proof['source_file'])):
        raise ValueError('zero-based image-position evidence must be fixed independently of harbor_05 reference')
    indices = [Decimal(line.split()[0]) for line in Path(proof['source_file']).read_text().splitlines()
               if line.strip() and not line.lstrip().startswith('#')]
    if not indices or min(indices) != 0 or any(n != n.to_integral_value() or n < 0 for n in indices):
        raise ValueError('development format does not establish zero-based image positions; no guess or reference-error selection')
    root = method_plan_path.parent.parent
    target = root / version
    if target.parent != root or not version.startswith('vio-scoring-v'):
        raise ValueError('use an exclusive new scoring version below the data root')
    target.mkdir(exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    hashes = {n: sha256(repo / n) for n in SOURCE_FILES}
    plan = dict(schema='aqualoc-freeze-gated-scoring-v3', registered_utc=datetime.now(timezone.utc).isoformat(),
        method_plan=str(method_plan_path), method_plan_sha256=sha256(method_plan_path),
        freeze_sha256=sha256(method_plan_path.with_name('freeze.json')),
        previous_scoring_plan_sha256=sha256(method_plan_path.with_name('scoring-plan.json')),
        reference_url=REFERENCE_URL, reference_expected_bytes=100668,
        expected_size_provenance='official public directory metadata; content not yet acquired/read',
        reference_format='img_number tx ty tz qx qy qz qw; metric camera positions',
        reference_index_base=0, reference_clock='native camera ordinal -> audited corrected header nanoseconds',
        reference_order='sort unique image positions; retain raw source text unchanged',
        index_origin_proof_sha256=sha256(proof_path), index_origin_proof=proof,
        reference_access='not_performed', source_sha256=hashes,
        scale=1, alignment='rigid SE(3)', max_interpolation_interval_ns=100000000,
        matching='exact or linearly interpolated bracketing reference, no extrapolation',
        optimization={'clock': False, 'scale': False, 'calibration': False},
        supports='individual and timestamp intersection of all four; never exclude failed trials from common support',
        independent_check='separate forward-scan time matching and Horn quaternion alignment',
        original_method_reproduced=False, independent_hardware_truth=False, sota_achieved=False)
    for name in SOURCE_FILES:
        copy = target / 'source' / name
        copy.parent.mkdir(parents=True, exist_ok=True)
        with copy.open('xb') as f:
            f.write((repo / name).read_bytes())
        if sha256(copy) != hashes[name]:
            raise ValueError('source changed during registration; incomplete version retained')
    write_new_json(target / 'plan.json', plan)
    write_new_json(target / 'registration.json', {'scoring_plan_sha256': sha256(target / 'plan.json'),
                                                'reference_access': 'not_performed'})
    return target


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--method-plan', type=Path, required=True)
    parser.add_argument('--index-proof', type=Path, required=True)
    parser.add_argument('--version', default='vio-scoring-v3')
    args = parser.parse_args()
    print(register(args.method_plan, args.index_proof, args.version))
