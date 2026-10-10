"""Register and freeze two baseline trials before trajectory reference access."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time

import yaml
from ntnu_sensor_only import sha, new_json

REPO = Path('/workspace/aqua_localization')
ROOT = Path('/workspace/aqua-data/development/ntnu_fjord_01')
SDK = Path('/workspace/aqua-next/ntnu')
BINARY = SDK / 'openvins-native-build-v7/openvins-ntnu'
CONFIG = REPO / 'cfg/benchmarks/ntnu_openvins_fixed.json'
UPSTREAM = Path('/workspace/.aqua-cloud/third-party/open_vins')
PIN = '69488123ed9362dd44b6f28e7f4680abbff1442b'


def check_calibration():
    """Check executable values independently against publisher YAML metadata."""
    log = (BINARY.parent / 'resolved-profile.log').read_text()
    lines = [line.split('CALIBRATION_JSON=', 1)[1] for line in log.splitlines() if line.startswith('CALIBRATION_JSON=')]
    if len(lines) != 1:
        raise ValueError('executable calibration description missing')
    actual = json.loads(lines[0])
    publisher = SDK / 'hf-calibrations/calibrations/cam0_cam1_stereo'
    camera = yaml.safe_load((REPO / 'cfg/benchmarks/ntnu_cam0_water.yaml').read_text())
    transform = yaml.safe_load((publisher / 'extrinsics_air/camchain-imucam-stereo-extrinsics-air.yaml').read_text())['cam0']['T_cam_imu']
    noise = yaml.safe_load((publisher / 'extrinsics_air/imu0_alphasense_noise.yaml').read_text())
    matrix = camera['camera_matrix']['data']
    expected = dict(intrinsics=[matrix[0], matrix[4], matrix[2], matrix[5], *camera['distortion_coefficients']['data']],
                    R_imu_to_camera=[transform[i][j] for i in range(3) for j in range(3)],
                    p_imu_in_camera=[transform[i][3] for i in range(3)],
                    noise=[noise[k] for k in ['gyroscope_noise_density', 'accelerometer_noise_density',
                                              'gyroscope_random_walk', 'accelerometer_random_walk']],
                    camera_offset=0.0)
    for key, values in expected.items():
        if isinstance(values, list):
            if len(actual[key]) != len(values) or any(not math.isclose(a, b, rel_tol=1e-12, abs_tol=1e-12) for a, b in zip(actual[key], values)):
                raise ValueError(f'executable calibration differs from publisher: {key}')
        elif actual[key] != values:
            raise ValueError('additional camera correction changed')
    return dict(status='executable_intrinsics_transform_noise_verified_against_publisher', values=actual)


def inspect(directory):
    stamps = []
    with (directory / 'estimate.tum').open() as stream:
        for line in stream:
            fields = line.split()
            if len(fields) != 8:
                raise ValueError('invalid trajectory record')
            sec, ns = fields[0].split('.')
            stamp = int(sec)*10**9 + int(ns)
            values = list(map(float, fields[1:]))
            if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1) > 1e-6:
                raise ValueError('nonfinite or nonunit pose')
            if stamps and stamp <= stamps[-1]:
                raise ValueError('non-increasing native output clock')
            stamps.append(stamp)
    if not stamps or stamps[-1] != 1700605095125093130:
        raise ValueError('no initialized trajectory at full native last camera')
    with (directory / 'estimate.events.tsv').open() as stream:
        rows = list(csv.DictReader(stream, delimiter='\t'))
    cameras = [r for r in rows if r['type'] == 'C']
    pressure = [r for r in rows if r['type'] == 'P']
    if [int(r['ordinal']) for r in cameras] != list(range(6368)) or [int(r['ordinal']) for r in pressure] != list(range(15926)):
        raise ValueError('native sensor ordinals not fully accounted for')
    estimated = [int(r['native_effective_ns']) for r in cameras if r['disposition'] == 'estimated_imu_pose']
    if estimated != stamps:
        raise ValueError('pose/native event correspondence changed')
    outside = [r for r in pressure if r['disposition'] == 'outside_available_imu_support']
    if len(outside) != 1 or int(outside[0]['ordinal']) != 15925 or int(outside[0]['native_effective_ns']) != 1700605095194369664:
        raise ValueError('last unsupported pressure not explicitly retained')
    excluded = [int(r['ordinal']) for r in cameras if r['disposition'] != 'estimated_imu_pose']
    return dict(poses=len(stamps), support_ns=[stamps[0], stamps[-1]],
                initialized_or_missing_state_camera_exclusions=excluded,
                native_camera_events=len(cameras), native_pressure_events=len(pressure),
                pressure_updates=0, pressure_support_exclusions=outside)


def run(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    if subprocess.check_output(['git', '-C', str(UPSTREAM), 'rev-parse', 'HEAD'], text=True).strip() != PIN:
        raise ValueError('upstream commit changed')
    if subprocess.check_output(['git', '-C', str(UPSTREAM), 'status', '--porcelain'], text=True).strip():
        raise ValueError('upstream checkout changed')
    calibration = check_calibration()
    subprocess.run([str(SDK / 'openvins-observability-test')], check=True)
    inputs = ROOT / 'full-input-v1'
    verification = json.loads((inputs / 'wire-verification.json').read_text())
    wire_hash = sha(inputs / 'input.bin')
    if verification['input_sha256'] != wire_hash or verification['verified_counts'] != {'I': 63698, 'C': 6368, 'P': 15926}:
        raise ValueError('full independently verified input required')
    prior = ROOT / 'full-comparison-v1/freeze.json'
    prior_freeze = json.loads(prior.read_text())
    if prior_freeze['integrity_status'] != 'verified' or prior_freeze['reference_values_read'] or len(prior_freeze['trials']) != 4:
        raise ValueError('prior four-trial pre-reference freeze invalid')
    sources = [Path(__file__), REPO / 'scripts/benchmarks/openvins_ntnu_v5.cpp',
               REPO / 'scripts/benchmarks/build_openvins_ntnu_v5.bash', CONFIG, BINARY,
               BINARY.parent / 'resolved-profile.log', inputs / 'plan.json', inputs / 'result.json',
               inputs / 'wire-verification.json', prior, ROOT / 'openvins-comparison-v1/freeze.json', ROOT / 'openvins-comparison-v2/freeze.json', ROOT / 'openvins-comparison-v3/freeze.json', ROOT / 'openvins-comparison-v4/freeze.json',
               REPO / 'cfg/benchmarks/ntnu_cam0_water.yaml', UPSTREAM / 'config/euroc_mav/estimator_config.yaml', UPSTREAM / 'ov_init/src/dynamic/DynamicInitializer.cpp',
               REPO / 'tests/benchmarks/test_openvins_initialization_observability.cpp',
               SDK / 'openvins-observability-test', SDK / 'openvins-observability-test.log']
    sources.extend(sorted(Path('/workspace/.aqua-cloud/openvins-build').glob('libov_*_lib.so')))
    # Bind actual solver/vision and transitive shared libraries in this runtime.
    for line in subprocess.check_output(['ldd', str(BINARY)], text=True).splitlines():
        if '=> /' in line:
            library = Path(line.split('=> ', 1)[1].split(' ', 1)[0])
            if library not in sources:
                sources.append(library)
    publisher = SDK / 'hf-calibrations/calibrations/cam0_cam1_stereo/extrinsics_air'
    sources.extend([publisher / 'camchain-imucam-stereo-extrinsics-air.yaml', publisher / 'imu0_alphasense_noise.yaml'])
    hashes = {str(p): sha(p) for p in sources}
    plan = dict(schema='ntnu-openvins-mono-baseline-plan-v5', upstream_commit=PIN,
                source_runtime_input_receipt_sha256=hashes, input_sha256=wire_hash,
                prior_four_trial_freeze_sha256=sha(prior), prior_failed_openvins_freeze_sha256={v:sha(ROOT / f'openvins-comparison-{v}/freeze.json') for v in ['v1','v2','v3','v4']}, revision_reason='v4 never initialized; source couples minimum per-feature views to int(window seconds), so1s admits single-view depth degeneracy and4s requires4 sampled views; use upstream euroc_mav configuration2s window to require2 views; retain1000-iteration and5s caps, all strict covariance/observability tests, clock/calibration/noise/input unchanged; no reference access, clock/calibration/noise/input unchanged', calibration_verification=calibration,
                scope='full native cam0 mono; same AQMONO01 input as frozen ReAqROVIO ablation',
                method='standard pinned OpenVINS numerical SDK with standalone mono native replay adapter; no pressure update',
                pressure='every native pressure tick retained with explicit no-update/support reason; no SI units inferred',
                clock='native integer origin before doubles; fixed IMU -1770000ns in verified wire; zero additional camera correction',
                calibration='publisher air extrinsics plus underwater equidistant intrinsics and publisher IMU noise; no online or reference optimization',
                output_frame='IMU position in world and quaternion IMU to world, exact native camera epochs',
                initialization='SDK dynamic initializer enabled, 2second upstream euroc_mav example window rather than prior1s/4s, 10deg threshold, one optimizer thread, 5s limit; maximum iterations1000 rather than prior20/100; all exclusions reported',
                trials=['openvins-1', 'openvins-2'], timeout_seconds=600,
                scoring=dict(status='unexecuted_missing_independent_hardware_truth', alignment='SE3', scale=1,
                             maximum_interpolation_gap_seconds=.1, supports='each trial; intersection of all six successful trials',
                             optimize_clock_scale_calibration_to_reference=False),
                native_data_complete=True, independent_accuracy_ready=False, reference_values_read=False,
                sota_achieved=False, original_51cea95_reproduced=False)
    new_json(output / 'plan.json', plan)
    snapshots = output / 'sources'; snapshots.mkdir()
    for index, source in enumerate(sources):
        shutil.copyfile(source, snapshots / f'{index:02d}-{source.name}')
    outcomes = []
    env = dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    for name in plan['trials']:
        directory = output / name; directory.mkdir()
        started = time.perf_counter(); error = None; code = None; metrics = None
        try:
            with (directory / 'run.log').open('x') as log:
                completed = subprocess.run([str(BINARY), str(inputs / 'input.bin'), str(CONFIG), str(directory / 'estimate')],
                                           stdout=log, stderr=subprocess.STDOUT, env=env, timeout=600)
            code = completed.returncode
            if code:
                raise ValueError(f'estimator exit {code}')
            metrics = inspect(directory)
        except Exception as exc:
            error = str(exc)
        receipt = dict(name=name, status='failed' if error else 'success', exit_code=code, error=error,
                       wall_seconds=time.perf_counter()-started, output=metrics,
                       artifacts_sha256={p.name: sha(p) for p in sorted(directory.iterdir()) if p.is_file()})
        new_json(directory / 'outcome.json', receipt); outcomes.append(receipt)
        print(json.dumps({k: receipt[k] for k in ['name', 'status', 'error', 'wall_seconds']}), flush=True)
    changes = [name for name, digest in hashes.items() if sha(name) != digest]
    if sha(inputs / 'input.bin') != wire_hash:
        changes.append(str(inputs / 'input.bin'))
    common = None
    if all(t['status'] == 'success' for t in outcomes + prior_freeze['trials']):
        supports = [t['output']['support_ns'] for t in outcomes] + [t['output']['scoring_support_ns'] for t in prior_freeze['trials']]
        common = [max(s[0] for s in supports), min(s[1] for s in supports)]
        if common[0] > common[1]: common = None
    freeze = dict(schema='ntnu-openvins-baseline-freeze-v5', plan_sha256=sha(output / 'plan.json'),
                  trials=outcomes, common_six_trial_support_ns=common, source_changes=changes,
                  integrity_status='invalid' if changes else 'verified', reference_values_read=False,
                  scoring_status='unexecuted_missing_independent_hardware_truth', sota_achieved=False)
    new_json(output / 'freeze.json', freeze)
    if changes: raise ValueError('source/input changed during execution')
    return freeze


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    run(parser.parse_args().output)
