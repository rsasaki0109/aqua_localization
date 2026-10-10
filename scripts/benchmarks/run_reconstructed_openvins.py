"""Run the separately registered reconstruction; no reference access here."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import time

from audit_native_imu_support import CAMERA_TOPIC, IMU_TOPIC, PRESSURE_TOPIC, sha256, write_new_json


def stream_input(bag, audit, sink):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore
    store = get_typestore(Stores.ROS1_NOETIC)
    imus = []
    with Reader(Path(bag)) as r:
        for c, ns, raw in r.messages(connections=[c for c in r.connections if c.topic == IMU_TOPIC]):
            m = store.deserialize_ros1(raw, c.msgtype)
            imus.append((ns, m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                         m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
    if len(imus) < 2:
        raise ValueError('complete native IMU support required')
    sink.write(b'AQUAVIO3' + struct.pack('<qI', imus[0][0], len(imus)))
    for row in imus:
        sink.write(struct.pack('<q6d', *row))
    ordinals = {(r['sensor'], r['corrected_stamp_ns']): r['native_ordinal']
                for r in audit['records'] if r['included']}
    with Reader(Path(bag)) as reader:
        for c, ns, raw in reader.messages(connections=[c for c in reader.connections if c.topic != IMU_TOPIC]):
            m = store.deserialize_ros1(raw, c.msgtype)
            if c.topic == CAMERA_TOPIC:
                if m.encoding != 'mono8' or m.width != 640 or m.height != 512 or m.step != 640:
                    raise ValueError('registered native mono8 camera encoding/geometry violated')
                pixels = bytes(m.data)
                sink.write(b'C' + struct.pack('<qIIII', ns, ordinals[('camera', ns)],
                                              m.width, m.height, len(pixels)) + pixels)
            elif c.topic == PRESSURE_TOPIC:
                sink.write(b'P' + struct.pack('<qId', ns, ordinals[('pressure', ns)], m.fluid_pressure))
            else:
                raise ValueError('unexpected common input topic')
    sink.flush()


def check_registration(plan_path):
    plan_path = Path(plan_path)
    plan = json.loads(plan_path.read_text())
    registration = json.loads(plan_path.with_name('registration.json').read_text())
    if sha256(plan_path) != registration['plan_sha256'] or sha256(plan_path.with_name('scoring-plan.json')) != registration['scoring_plan_sha256']:
        raise ValueError('registration changed; preserve and register another version')
    from prepare_reconstructed_comparison import METHOD_SOURCES, OPENVINS_COMMIT
    expected_trials = [{'id': f'{arm}-{i}', 'arm': arm, 'repeat': i}
                       for arm in ('vio', 'pressure') for i in (1, 2)]
    if (plan.get('schema') != 'aqualoc-reconstructed-method-v1' or
        plan.get('original_frozen_method_reproduced') is not False or
        plan.get('upstream_commit') != OPENVINS_COMMIT or plan.get('trials') != expected_trials or
        set(plan.get('source_sha256', {})) != set(METHOD_SOURCES)):
        raise ValueError('complete new method/source/four-trial registration required')
    if sha256(plan_path.with_name('pressure_quality.json')) != plan['pressure_quality_sha256']:
        raise ValueError('registered pressure quality ledger changed')
    repo = Path(__file__).resolve().parents[2]
    for name, digest in plan['source_sha256'].items():
        if sha256(repo / name) != digest or sha256(plan_path.parent / 'source' / name) != digest:
            raise ValueError(f'registered source changed: {name}')
    for name, digest in plan['runtime_artifact_sha256'].items():
        if sha256(Path(name)) != digest:
            raise ValueError(f'registered binary/input changed: {name}')
    if plan['reference_access'] != 'not_performed' or plan['comparison_executable'] is not True:
        raise ValueError('method is not registered for execution before reference access')
    return plan


def run_all(plan_path):
    plan_path = Path(plan_path)
    plan = check_registration(plan_path)
    destination = plan_path.parent / 'trials'
    destination.mkdir(exist_ok=False)
    input_audit = json.loads(Path(plan['input_audit']).read_text())
    outcomes = []
    for trial in plan['trials']:
        # Validate again before each run; failed trials are retained, not replaced.
        check_registration(plan_path)
        directory = destination / trial['id']
        directory.mkdir()
        trajectory, events = directory / 'trajectory.csv', directory / 'events.csv'
        started = time.perf_counter()
        feeder_error = None
        with (directory / 'run.log').open('wb') as log:
            process = subprocess.Popen([plan['executable'], trial['arm'], str(trajectory), str(events)],
                stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT,
                env=dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1'))
            try:
                stream_input(plan['supported_bag'], input_audit, process.stdin)
            except Exception as error:
                feeder_error = f'{type(error).__name__}: {error}'
            finally:
                try:
                    process.stdin.close()
                except BrokenPipeError:
                    pass
            exit_code = process.wait()
        runtime = time.perf_counter() - started
        rows = list(csv.DictReader(trajectory.open())) if trajectory.exists() else []
        stamps = [int(r['timestamp_ns']) for r in rows]
        if rows and any(not math.isfinite(float(r[k])) for r in rows for k in ('x', 'y', 'z')):
            feeder_error = 'nonfinite trajectory'
        event_rows = list(csv.DictReader(events.open())) if events.exists() else []
        excluded = [r for r in event_rows if r['sensor'] == 'camera' and r['status'] != 'estimated']
        processed = {sensor: sum(r['sensor'] == sensor for r in event_rows) for sensor in ('camera', 'pressure')}
        if exit_code == 0 and (processed != input_audit['eligible_observation_counts'] or
                              any(b <= a for a, b in zip(stamps, stamps[1:])) or
                              len(rows) + len(excluded) != processed['camera']):
            feeder_error = 'successful exit did not account for every registered observation'
        outcome = dict(trial, status='success' if exit_code == 0 and rows and not feeder_error else 'failed',
            exit_code=exit_code, feeder_error=feeder_error, runtime_seconds=runtime,
            output_camera_count=len(rows), support_ns=[stamps[0], stamps[-1]] if stamps else None,
            processed_observation_counts=processed,
            native_counts={'imu': input_audit['native_imu_count'], **input_audit['native_observation_counts']},
            support_boundary_exclusions=input_audit['excluded_observation_counts'],
            pressure_quality_exclusions=plan['pressure_quality_summary'],
            initialization_and_missing_state_exclusions=excluded,
            pressure_event_status_counts={s: sum(r['sensor'] == 'pressure' and r['status'] == s for r in event_rows)
                for s in sorted({r['status'] for r in event_rows if r['sensor'] == 'pressure'})},
            output_sha256={p.name: sha256(p) for p in sorted(directory.iterdir()) if p.is_file()},
            reference_access='not_performed')
        write_new_json(directory / 'result.json', outcome)
        outcome['result_sha256'] = sha256(directory / 'result.json')
        outcomes.append(outcome)
        print(json.dumps({'trial': trial['id'], 'status': outcome['status'], 'runtime_seconds': runtime}), flush=True)
    check_registration(plan_path)
    write_new_json(plan_path.parent / 'freeze.json', {
        'schema': 'aqualoc-reconstructed-estimator-freeze-v1', 'all_planned_outcomes_frozen': True,
        'plan_sha256': sha256(plan_path), 'registration_sha256': sha256(plan_path.with_name('registration.json')),
        'runtime_artifact_sha256': plan['runtime_artifact_sha256'], 'source_sha256': plan['source_sha256'],
        'trials': outcomes, 'reference_acquired': False, 'reference_read': False,
        'scoring_status': 'blocked_until_reference_adapter_registered_before_reference_access',
        'common_all_four_support': 'unavailable_if_any_trial_failed; not yet scored',
        'original_frozen_method_reproduced': False, 'sota_achieved': False})
    print(json.dumps({'freeze_sha256': sha256(plan_path.parent / 'freeze.json')}), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    run_all(parser.parse_args().plan)
