"""Read-only baseline restoration gate, preserving failed and successful outcomes."""
import argparse
import csv
import json
import math
from pathlib import Path

from ntnu_sensor_only import sha


def check(root):
    root = Path(root)
    wire_hash = sha(root / 'full-input-v1/input.bin')
    versions = []
    for version in ['v1', 'v2', 'v3', 'v4', 'v5', 'v6']:
        directory = root / f'openvins-comparison-{version}'
        plan = json.loads((directory / 'plan.json').read_text())
        freeze = json.loads((directory / 'freeze.json').read_text())
        if (sha(directory / 'plan.json') != freeze['plan_sha256'] or
                freeze['integrity_status'] != 'verified' or freeze['source_changes'] or
                plan['input_sha256'] != wire_hash or plan['reference_values_read'] or freeze['reference_values_read']):
            raise ValueError('baseline freeze/input/reference gate failed')
        for name, digest in plan['source_runtime_input_receipt_sha256'].items():
            if sha(name) != digest:
                raise ValueError(f'registered source/runtime changed: {name}')
        if len(freeze['trials']) != 2:
            raise ValueError('two-trial accounting incomplete')
        trials = []
        for trial in freeze['trials']:
            trial_dir = directory / trial['name']
            if json.loads((trial_dir / 'outcome.json').read_text()) != trial:
                raise ValueError('trial receipt changed')
            for name, digest in trial['artifacts_sha256'].items():
                if sha(trial_dir / name) != digest:
                    raise ValueError('frozen output changed')
            with (trial_dir / 'estimate.events.tsv').open() as stream:
                rows = list(csv.DictReader(stream, delimiter='\t'))
            counts = {}
            for kind, limit in [('C', 6368), ('P', 15926)]:
                actual = [int(r['ordinal']) for r in rows if r['type'] == kind]
                if actual != list(range(len(actual))) or len(actual) > limit:
                    raise ValueError('partial/native event ordinal accounting changed')
                counts[kind] = len(actual)
            stamps = []
            with (trial_dir / 'estimate.tum').open() as stream:
                for line in stream:
                    fields = line.split()
                    if len(fields) != 8:
                        raise ValueError('invalid trajectory format')
                    sec, ns = fields[0].split('.')
                    stamp = int(sec)*10**9+int(ns)
                    values = list(map(float, fields[1:]))
                    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values[3:])-1) > 1e-6:
                        raise ValueError('nonfinite/nonunit retained pose')
                    if stamps and stamp <= stamps[-1]:
                        raise ValueError('native output clock changed')
                    stamps.append(stamp)
            estimated = [int(r['native_effective_ns']) for r in rows if r['type'] == 'C' and r['disposition'] == 'estimated_imu_pose']
            if estimated != stamps:
                raise ValueError('native event/pose correspondence mismatch')
            if trial['status'] == 'success':
                if not stamps or counts != {'C': 6368, 'P': 15926} or stamps[-1] != 1700605095125093130:
                    raise ValueError('successful trial missing full native endpoint/accounting')
                if trial['output']['support_ns'] != [stamps[0], stamps[-1]]:
                    raise ValueError('successful trial support changed')
            elif trial['status'] != 'failed' or not trial['error']:
                raise ValueError('unclassified trial outcome')
            trials.append(dict(name=trial['name'], status=trial['status'],
                               event_counts=counts, poses=len(stamps),
                               initialization_or_no_state_excluded_cameras=counts['C']-len(stamps),
                               support_ns=[stamps[0], stamps[-1]] if stamps else None,
                               wall_seconds=trial['wall_seconds']))
        versions.append(dict(version=version, freeze_sha256=sha(directory / 'freeze.json'), trials=trials,
                             common_six_trial_support_ns=freeze['common_six_trial_support_ns']))
    return dict(schema='ntnu-openvins-restoration-verification-v1', integrity='verified',
                versions=versions, latest_two_trials_successful=all(t['status'] == 'success' for t in versions[-1]['trials']),
                reference_values_read=False, independent_accuracy_ready=False, sota_achieved=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    print(json.dumps(check(parser.parse_args().root), indent=2))
