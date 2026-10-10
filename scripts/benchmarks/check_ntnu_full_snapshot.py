"""Read-only full-sequence integrity and processed-tail gate; no reference access."""
import argparse
import csv
import json
from pathlib import Path
import re

from ntnu_sensor_only import sha


def check(root):
    root = Path(root)
    inputs = root / 'full-input-v1'
    directory = root / 'full-comparison-v1'
    result = json.loads((inputs / 'result.json').read_text())
    input_plan = json.loads((inputs / 'plan.json').read_text())
    verification = json.loads((inputs / 'wire-verification.json').read_text())
    if result['wire_counts'] != {'I': 63698, 'C': 6368, 'P': 15926}:
        raise ValueError('complete native replay counts changed')
    for name, key in [('input.bin', 'wire_sha256'),
                      ('native-packet-ledger.jsonl', 'ledger_sha256'),
                      ('plan.json', 'plan_sha256')]:
        if sha(inputs / name) != result[key]:
            raise ValueError(f'full input artifact changed: {name}')
    if verification['verified_counts'] != result['wire_counts'] or verification['input_sha256'] != result['wire_sha256']:
        raise ValueError('independent sensor verification mismatch')
    if sha(input_plan['source']) != input_plan['source_sha256']:
        raise ValueError('native selected source changed')
    if sha(Path(input_plan['source']).parent / 'result.json') != input_plan['isolation_result_sha256']:
        raise ValueError('native isolation receipt changed')
    plan = json.loads((directory / 'plan.json').read_text())
    freeze = json.loads((directory / 'freeze.json').read_text())
    if sha(directory / 'plan.json') != freeze['plan_sha256'] or freeze['integrity_status'] != 'verified':
        raise ValueError('full comparison freeze invalid')
    if plan['input_sha256'] != result['wire_sha256']:
        raise ValueError('registered input changed')
    for name, digest in plan['source_and_runtime_sha256'].items():
        if sha(name) != digest:
            raise ValueError(f'registered source/runtime changed: {name}')
    for version, digest in plan['prior_version_freeze_sha256'].items():
        if sha(root / f'mono-comparison-{version}' / 'freeze.json') != digest:
            raise ValueError('historical freeze changed')
    if any(x['reference_values_read'] for x in [result, verification, plan, freeze]):
        raise ValueError('reference access gate violated')
    if len(freeze['trials']) != 4:
        raise ValueError('four-trial accounting incomplete')
    imu_end = result['native_imu_support_ns'][1]
    last_camera = 1700605095125093130
    last_pressure = 1700605095194369664
    trials = []
    for trial in freeze['trials']:
        trial_dir = directory / trial['name']
        for name, digest in trial['artifacts_sha256'].items():
            if sha(trial_dir / name) != digest:
                raise ValueError('frozen trial output changed')
        if json.loads((trial_dir / 'outcome.json').read_text()) != trial:
            raise ValueError('trial receipt changed')
        if trial['status'] != 'success':
            raise ValueError(f"full-sequence execution failed: {trial['name']}")
        match = re.search(r'images=(\d+).*pressure_packets=(\d+) queued_pressure=(\d+) processed_images=(\d+) processed_pressure=(\d+)',
                          (trial_dir / 'run.log').read_text())
        if not match:
            raise ValueError('processed native counts missing')
        counts = tuple(map(int, match.groups()))
        expected = (6368, 15926, 15924 if trial['depth'] else 0, 6368, 15924 if trial['depth'] else 0)
        if counts != expected:
            raise ValueError(f'queued native observations not fully processed: {counts}')
        with (trial_dir / 'estimate.events.tsv').open() as stream:
            rows = list(csv.DictReader(stream, delimiter='\t'))
        cameras = [row for row in rows if row['type'] == 'C']
        pressures = [row for row in rows if row['type'] == 'P']
        endings = [row for row in rows if row['type'] == 'E']
        if [int(row['ordinal']) for row in cameras] != list(range(6368)) or [int(row['ordinal']) for row in pressures] != list(range(15926)):
            raise ValueError('native observation ordinals missing or reordered')
        if len(endings) != 1 or rows[-1] != endings[0] or int(endings[0]['native_effective_ns']) != imu_end:
            raise ValueError('complete-input finalization gate changed')
        outside = pressures[-1]
        if int(outside['native_effective_ns']) != last_pressure or outside['disposition'] != 'outside_available_imu_support':
            raise ValueError('unsupported pressure not explicitly retained')
        end = trial['output']['full_support_ns'][1]
        if not last_camera <= end <= imu_end:
            raise ValueError('supported tail missing or extrapolated')
        if not trial['depth'] and end != last_camera:
            raise ValueError('ablation must finish at exact last native camera')
        if any(int(row['safe_ns']) > imu_end for row in rows):
            raise ValueError('state output extrapolated beyond native IMU support')
        trials.append(dict(name=trial['name'], wall_seconds=trial['wall_seconds'],
                           support_ns=trial['output']['scoring_support_ns'],
                           initialization_excluded_pose_count=trial['output']['initialization_excluded_pose_count'],
                           processed_images=counts[3], processed_pressure=counts[4]))
    common = [max(t['support_ns'][0] for t in trials), min(t['support_ns'][1] for t in trials)]
    if common != freeze['common_support_ns']:
        raise ValueError('common four-trial support mismatch')
    return dict(schema='ntnu-full-snapshot-verification-v1', full_native_input_verified=True,
                trials=trials, common_support_ns=common, freeze_sha256=sha(directory / 'freeze.json'),
                excluded_pressure=dict(native_ordinal=15925, stamp_ns=last_pressure,
                                       after_native_imu_ns=last_pressure-imu_end,
                                       native_packet_retained=True, reason='outside_available_imu_support'),
                reference_values_read=False, independent_accuracy_evaluation_ready=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    print(json.dumps(check(parser.parse_args().root), indent=2))
