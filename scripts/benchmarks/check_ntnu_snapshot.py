"""Read-only retained artifact gate; no reference coordinates and no silent rebuild."""
import argparse
import json
from pathlib import Path
import re
from ntnu_sensor_only import sha
from acquire_ntnu_hf import SIZE, SHA256


def check(root):
    root = Path(root); native=root/'fjord_1.bag'
    if native.stat().st_size != SIZE or sha(native) != SHA256:
        raise ValueError('native publisher size/SHA gate failed')
    acquisition=json.loads((root/'acquired.json').read_text())
    if acquisition['verified_sha256'] != SHA256 or not acquisition['complete']:
        raise ValueError('native acquisition receipt incomplete')
    ready=json.loads((root/'snapshot_native_ready_v1.json').read_text())
    if not ready['native_data_complete'] or ready['native_sha256'] != SHA256:
        raise ValueError('native snapshot gate incomplete')
    for name,digest in ready['receipts_sha256'].items():
        if sha(root/name) != digest: raise ValueError('native verification receipt changed')
    isolated=root/'sensor-input-v1'; result=json.loads((isolated/'result.json').read_text())
    if sha(isolated/'sensor_only.bag') != result['derived_sha256']:
        raise ValueError('isolated native packets changed')
    inputs=root/'mono-input-v1'; result=json.loads((inputs/'result.json').read_text())
    for name,key in [('input.bin','wire_sha256'),('native-packet-ledger.jsonl','ledger_sha256')]:
        if sha(inputs/name) != result[key]: raise ValueError('registered mono input changed')
    versions=[]
    for directory in sorted(root.glob('mono-comparison-v*')):
        plan=json.loads((directory/'plan.json').read_text()); freeze=json.loads((directory/'freeze.json').read_text())
        if sha(directory/'plan.json') != freeze['plan_sha256'] or freeze['integrity_status'] != 'verified':
            raise ValueError('comparison freeze invalid')
        if sha(inputs/'input.bin') != plan['input_sha256']: raise ValueError('trial input changed')
        for name,digest in plan['source_and_runtime_sha256'].items():
            if sha(name) != digest: raise ValueError(f'registered source/runtime changed: {name}')
        for trial in freeze['trials']:
            for name,digest in trial['artifacts_sha256'].items():
                if sha(directory/trial['name']/name) != digest: raise ValueError('frozen trial output changed')
            receipt=json.loads((directory/trial['name']/'outcome.json').read_text())
            if receipt != trial: raise ValueError('trial outcome changed')
            if directory.name=='mono-comparison-v6' and trial['status']=='success':
                text=(directory/trial['name']/'run.log').read_text()
                match=re.search(r'images=(\d+).*pressure_packets=(\d+) queued_pressure=(\d+) processed_images=(\d+) processed_pressure=(\d+)',text)
                if not match: raise ValueError('native processed-event counts missing')
                images,packets,queued,processed_images,processed_pressure=map(int,match.groups())
                if (images,packets)!=(result['wire_counts']['C'],result['wire_counts']['P']):
                    raise ValueError('native event count mismatch')
                if images!=processed_images or queued!=processed_pressure:
                    raise ValueError('unprocessed native observation')
                if trial['output']['full_support_ns'][1]!=result['fixed_prefix_end_ns']:
                    raise ValueError('native prefix endpoint not processed')
        if len(freeze['trials']) != 4 or freeze['reference_values_read']:
            raise ValueError('trial accounting/reference gate violated')
        versions.append(dict(version=directory.name,success=sum(t['status']=='success' for t in freeze['trials']),
            failed=sum(t['status']=='failed' for t in freeze['trials']),freeze_sha256=sha(directory/'freeze.json')))
    if not versions or versions[-1]['version']!='mono-comparison-v6' or versions[-1]['success']!=4:
        raise ValueError('current four-trial processed-event gate incomplete')
    return dict(native_complete=True,native_sha256=SHA256,versions=versions,
        reference_values_read=False,independent_accuracy_evaluation_ready=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args()
    print(json.dumps(check(a.root),indent=2))
