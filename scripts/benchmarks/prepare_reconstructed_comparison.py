"""Register a fresh executable method separately from the unavailable old method."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess

from audit_native_imu_support import sha256, write_new_json
from prepare_vio_comparison import SOURCE_FILES, OPENVINS_COMMIT

METHOD_SOURCES = SOURCE_FILES + ['scripts/benchmarks/' + name for name in (
    'openvins_reconstructed.cpp', 'openvins_pressure.hpp', 'run_reconstructed_openvins.py',
    'prepare_reconstructed_comparison.py')] + ['aqua_imu_loc/config/aqualoc.yaml']
METHOD_SOURCES += ['scripts/benchmarks/build_reconstructed_openvins.bash']
METHOD_SOURCES += ['scripts/benchmarks/check_reconstructed_calibration.py']
METHOD_SOURCES += ['scripts/benchmarks/audit_pressure_quality.py']
METHOD_SOURCES += ['docs/benchmarks/calibration/' + n for n in (
    'harbor_camera_calib.yaml', 'harbor_imu_camera_calib.yaml', 'harbor_imu_noises.yaml')]


def register(root, executable, version='vio-comparison-v4'):
    root, executable = Path(root), Path(executable)
    repo = Path(__file__).resolve().parents[2]
    previous = root / 'vio-comparison-v2'
    native_plan = json.loads((previous / 'plan.json').read_text())
    conversion = json.loads((previous / 'input/conversion.json').read_text())
    verification = json.loads((previous / 'input/independent-verification.json').read_text())
    if conversion['status'] != 'converted' or verification['status'] != 'verified':
        raise ValueError('complete independent common-input verification required')
    supported = previous / 'input/supported.bag'
    if sha256(supported) != conversion['supported_bag_sha256'] or conversion['supported_bag_sha256'] != verification['supported_bag_sha256']:
        raise ValueError('verified common input changed')
    for name, digest in native_plan['source_sha256'].items():
        if sha256(repo / name) != digest:
            raise ValueError('input-contract source changed; register a new input version first')
    dependency = Path('/workspace/.aqua-cloud/third-party/open_vins')
    if subprocess.check_output(['git', '-C', str(dependency), 'rev-parse', 'HEAD'], text=True).strip() != OPENVINS_COMMIT:
        raise ValueError('upstream pin changed')
    subprocess.run(['git', '-C', str(dependency), 'diff', '--quiet', 'HEAD'], check=True)
    subprocess.run([str(executable), '--self-test'], check=True)
    from check_reconstructed_calibration import check
    check(executable, repo / 'docs/benchmarks/calibration')
    build_manifest = executable.with_name('reconstructed-build-manifest.json')
    build = json.loads(build_manifest.read_text())
    if (build['upstream_commit'] != OPENVINS_COMMIT or build['executable_sha256'] != sha256(executable) or
        build['upstream_library_sha256'] != sha256('/workspace/.aqua-cloud/openvins-build/libov_msckf_lib.so') or
        any(sha256(name) != digest for name, digest in build['source_sha256'].items())):
        raise ValueError('build does not bind current registered sources and dependency')
    target = root / version
    if target.parent != root or not version.startswith('vio-comparison-v'):
        raise ValueError('use a new version below the data root')
    target.mkdir(exist_ok=False)
    from audit_pressure_quality import audit_pressure_quality
    quality = audit_pressure_quality(supported)
    if (quality['native_pressure_count'] != 1650 or
        [r['native_pressure_ordinal'] for r in quality['records'] if not r['measurement_eligible']] != [1401]):
        raise ValueError('pressure quality differs from pre-reference inspection; new registration required')
    write_new_json(target / 'pressure_quality.json', quality)
    source_hashes = {name: sha256(repo / name) for name in METHOD_SOURCES}
    artifacts = [executable, build_manifest, supported, previous / 'input/input_audit.json',
                 previous / 'input/independent-verification.json',
                 Path('/workspace/.aqua-cloud/openvins-build/libov_msckf_lib.so')]
    plan = dict(schema='aqualoc-reconstructed-method-v1', registered_utc=datetime.now(timezone.utc).isoformat(),
        sequence='harbor_05', reference_access='not_performed', comparison_executable=True,
        original_frozen_method_reproduced=False, unavailable_original_head='51cea95',
        upstream_commit=OPENVINS_COMMIT, source_sha256=source_hashes,
        runtime_artifact_sha256={str(p): sha256(p) for p in artifacts},
        executable=str(executable), supported_bag=str(supported),
        input_audit=str(previous / 'input/input_audit.json'),
        pressure_quality_sha256=sha256(target / 'pressure_quality.json'),
        pressure_quality_summary={k: quality[k] for k in ('native_pressure_count', 'measurement_eligible_count', 'measurement_excluded_count', 'policy')},
        previous_input_registration_sha256=sha256(previous / 'registration.json'),
        input_contract=native_plan['input_contract'],
        calibration_provenance={'source': 'official LIRMM Seafile harbor_calibration_files',
            'url': 'https://seafile.lirmm.fr/d/79b03788f29148ca84e5/',
            'file_sha256': {n: source_hashes['docs/benchmarks/calibration/' + n] for n in (
                'harbor_camera_calib.yaml', 'harbor_imu_camera_calib.yaml', 'harbor_imu_noises.yaml')},
            'hash_provenance': 'this environment verified TLS downloads, not published checksums',
            'runtime_values': 'official T_cam_imu used directly as SDK IMU-to-camera extrinsic; equidistant 640x512, native raw IMU frame'},
        method={'camera_to_output': 'camera center in OpenVINS world, metric scale',
            'imu_density_gyro': .001, 'imu_density_accel': .02,
            'noise_provenance': 'official harbor_imu_noises.yaml densities directly; no data/reference fit',
            'gyro_random_walk': 5e-5, 'accel_random_walk': .001,
            'pressure_topic': '/barometer_node/pressure', 'pressure_units': 'Pa; no magnitude-driven conversion',
            'density_kg_m3': 1025, 'gravity_m_s2': 9.80665, 'depth_sigma_m': .2,
            'pressure_model': 'z plus constant offset = -P/(rho*g); augment uncertain offset with full cross-covariance at first post-init sample',
            'invalid_pressure': 'retain every native packet and pressure timestamp; propagate both arms; skip only the invalid pressure measurement update, with ordinal/hash/value/reason ledger and event status',
            'pressure_lever_arm': 'assumed co-located with IMU; not recovered calibration',
            'pressure_propagation': 'both arms propagate at pressure timestamps and remove temporary clones; baseline ignores pressure values',
            'camera_offset_additional_ns': 0, 'online_time_intrinsic_extrinsic_calibration': False,
            'sdk_clock_origin': 'first native IMU header; exact native camera ns restored in output',
            'dynamic_initializer': True, 'minimum_rotation_degrees': 10,
            'initialization_choice': 'SDK public Euroc example min_deg; chosen before reference access',
            'threads': 1, 'image_encoding': 'native mono8 only; no resizing',
            'feature_preprocessing': 'pinned SDK default HISTOGRAM equalization; no additional enhancement',
            'all_other_defaults': 'bound by pinned SDK source and registered C++ source'},
        trials=[{'id': f'{arm}-{i}', 'arm': arm, 'repeat': i} for arm in ('vio', 'pressure') for i in (1, 2)],
        freeze_gate='all four successes/failures, runtimes, initialization exclusions and every output hash before reference access',
        scoring_status='blocked_until_reference_adapter_registered_before_reference_access',
        scoring={'alignment': 'rigid SE(3)', 'scale': 1., 'max_interpolation_interval_s': .1,
            'time_scale_calibration_optimization': False, 'individual_and_all_four_common_support': True,
            'independent_correspondence_and_Horn_check': True,
            'reference_format_adapter': 'not yet registered; do not acquire/read reference'},
        claim='new reconstruction; neither original improvement replication nor independent hardware/SoTA evidence')
    for name in METHOD_SOURCES:
        copy = target / 'source' / name
        copy.parent.mkdir(parents=True, exist_ok=True)
        with copy.open('xb') as f:
            f.write((repo / name).read_bytes())
        if sha256(copy) != source_hashes[name]:
            raise ValueError('source changed during registration; incomplete version retained')
    write_new_json(target / 'scoring-plan.json', dict(plan, schema='aqualoc-reconstructed-scoring-v1'))
    write_new_json(target / 'plan.json', plan)
    write_new_json(target / 'registration.json', dict(plan_sha256=sha256(target / 'plan.json'),
        scoring_plan_sha256=sha256(target / 'scoring-plan.json'), reference_access='not_performed'))
    return target


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--version', default='vio-comparison-v4')
    args = parser.parse_args()
    print(register(args.root, args.executable, args.version))
