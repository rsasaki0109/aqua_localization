"""Exclusive pre-reference registration, all four outcomes, and immutable freeze."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time
from ntnu_sensor_only import sha, new_json

REPO = Path('/workspace/aqua_localization')
SDK = Path('/workspace/aqua-next/ntnu')
UPSTREAM = Path('/workspace/aqua-next/third-party/reaqrovio')
BINARY = SDK/'replay-sdk-build-v4/reaq_replay'
CONFIG = REPO/'cfg/benchmarks/ntnu_mono_reprojection.info'
CAMERA = REPO/'cfg/benchmarks/ntnu_cam0_water.yaml'
CORE = SDK/'sdk-build/libreaq_core.a'


def inspect_output(prefix):
    stamps = []; poses = 0
    with Path(str(prefix)+'.tum').open() as f:
        for line in f:
            parts = line.split()
            if len(parts) != 8: raise ValueError('invalid trajectory record')
            sec,nsec = parts[0].split('.'); stamp = int(sec)*10**9+int(nsec)
            values = list(map(float,parts[1:]))
            if not all(math.isfinite(v) for v in values): raise ValueError('nonfinite trajectory')
            if stamps and stamp <= stamps[-1]: raise ValueError('trajectory clock not increasing')
            if abs(sum(q*q for q in values[3:])-1) > 1e-6: raise ValueError('nonunit quaternion')
            stamps.append(stamp); poses += 1
    with Path(str(prefix)+'.events.tsv').open() as f:
        rows = list(csv.DictReader(f,delimiter='\t'))
    tracking = [int(r['safe_ns']) for r in rows if r['type']=='C' and int(r['features'])>0]
    if not stamps or not tracking: raise ValueError('no tracked trajectory')
    return dict(poses=poses, full_support_ns=[stamps[0],stamps[-1]],
        tracking_start_ns=min(tracking), scoring_support_ns=[min(tracking),stamps[-1]],
        initialization_excluded_pose_count=sum(t<min(tracking) for t in stamps),
        native_image_events=sum(r['type']=='C' for r in rows),
        native_pressure_events=sum(r['type']=='P' for r in rows),
        event_dispositions={reason:sum(r['disposition']==reason for r in rows)
            for reason in sorted({r['disposition'] for r in rows})})


def run(inputs, output):
    inputs, output = Path(inputs), Path(output)
    output.mkdir(parents=True,exist_ok=False)
    subprocess.run([str(SDK/'imu-merge-fixed-test')],check=True)
    verified = json.loads((inputs/'wire-verification.json').read_text())
    input_sha = sha(inputs/'input.bin')
    if input_sha != verified['input_sha256']: raise ValueError('verified wire changed')
    pins = {UPSTREAM:'7fc95c1fe97c8a4e89a1477d5699807718f37e43',
        UPSTREAM/'lightweight_filtering':'b35bfbf5d2dc9ef91ee877ce50966f34c73c44b7',
        UPSTREAM.parent/'kindr':'22a19ddd30ccfeba72bb2c35361e090bca7d16ec'}
    for tree,pin in pins.items():
        if subprocess.check_output(['git','-C',str(tree),'rev-parse','HEAD'],text=True).strip()!=pin:
            raise ValueError('dependency pin changed')
        if subprocess.check_output(['git','-C',str(tree),'status','--porcelain'],text=True).strip():
            raise ValueError('upstream worktree changed')
    sources = [Path(__file__),REPO/'scripts/benchmarks/reaq_replay.cpp',
        REPO/'scripts/benchmarks/reaq_pressure_events.hpp',
        REPO/'scripts/benchmarks/reaq_overlay/rovio/BaroUpdate.hpp',
        REPO/'scripts/benchmarks/prepare_ntnu_mono.py',REPO/'scripts/benchmarks/verify_ntnu_mono.py',
        REPO/'scripts/benchmarks/reaq_replay_build/CMakeLists.txt',BINARY,CORE,CONFIG,CAMERA,
        REPO/'scripts/benchmarks/reaq_overlay_v4/lightweight_filtering/Prediction.hpp',
        REPO/'tests/benchmarks/test_reaq_imu_merge.cpp',SDK/'imu-merge-fixed-test',
        SDK/'imu-merge-original.log',SDK/'imu-merge-fixed.log',
        SDK/'hf-calibrations/calibrations/cam0_cam1_stereo/intrinsics_water/camchain-stereo-intrinsics-underwater.yaml',
        inputs/'plan.json',inputs/'result.json',inputs/'wire-verification.json']
    bindings = {str(p):sha(p) for p in sources}
    env = dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    resolved = output/'resolved-profile.info'
    command = [str(BINARY),'--profile-only',str(CONFIG),str(CAMERA),str(output/'unused'),'0',str(resolved)]
    with (output/'profile-resolution.log').open('x') as log:
        subprocess.run(command,check=True,env=env,stdout=log,stderr=subprocess.STDOUT)
    plan = dict(schema='ntnu-mono-pressure-ablation-plan-v5', input=str(inputs/'input.bin'),
        input_sha256=input_sha, source_and_runtime_sha256=bindings,
        fixed_refractive_index=1.33, online_refractive_calibration=False,
        merged_imu_mean_sign_corrected=True, direct_intensity_updates=False, numeric_regression_expected='gyro mean 0.5; partial interval 1/3; upstream produced -0.5',
        previous_failed_freeze_sha256={v:sha(output.parent/f'mono-comparison-{v}/freeze.json') for v in ['v1','v2','v3','v4']},
        revision_reason='v1-v4 numerical failures retained; use supported feature-reprojection EKF updates instead of iterative direct-intensity updates; same v4 corrected SDK, native camera calibration and input; development before reference access',
        pinned_git={str(k):v for k,v in pins.items()}, resolved_profile_sha256=sha(resolved),
        method='ROVIO from pinned ReAqROVIO numerical core, publisher underwater equidistant calibration, mono cam0 adapter and pressure-noise bounds/IMU mean sign patches; not the published refractive method',
        upstream_byte_identical=False, old_aqualoc_method_reproduction=False,
        trials=[dict(name=f'{arm}-{repeat}',depth=depth) for arm,depth in [('vio',False),('pressure',True)] for repeat in [1,2]],
        scope='development prefix of native cam0 ordinal 0..1799, not full-sequence or independent accuracy evaluation',
        pressure=dict(offset=2660,scale=241,relative_anchor='first supported initialized pressure at current safe z; fixed thereafter',
            noise_covariance_diagonal=[0.01]*3,fixed_offsets=[0,0,0],same_pressure_clock_ticks_both_arms=True),
        output_clock='subtract integer native IMU origin before SDK double time; map safe times back through exact native event ledger',
        threads=1, timeout_seconds_per_trial=300,
        scoring=dict(status='blocked_missing_independent_hardware_truth',alignment='SE3',scale=1,
            maximum_interpolation_gap_seconds=0.1,initialization_exclusion='before first cam0 event with live visual features',
            supports='each trial and common intersection of all four successful trials',
            time_scale_calibration_reference_optimization=False,independent_correspondence_and_horn_verification=True,
            candidate_hf_tum='publisher ReAqROVIO estimate, never independent hardware truth'),
        reference_values_read=False, freeze_before_reference=True)
    new_json(output/'plan.json',plan)
    snapshots = output/'sources'; snapshots.mkdir()
    for i,p in enumerate(sources): shutil.copyfile(p,snapshots/f'{i:02d}-{p.name}')
    outcomes=[]
    for trial in plan['trials']:
        directory=output/trial['name']; directory.mkdir(); prefix=directory/'estimate'
        runtime_profile=directory/'resolved-profile.info'
        args=[str(BINARY),str(inputs/'input.bin'),str(CONFIG),str(CAMERA),str(prefix),str(int(trial['depth'])),str(runtime_profile)]
        start=time.perf_counter(); error=None; code=None
        try:
            with (directory/'run.log').open('x') as log:
                completed=subprocess.run(args,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=300)
            code=completed.returncode
            if code: raise ValueError(f'estimator exit {code}')
            if sha(runtime_profile)!=plan['resolved_profile_sha256']: raise ValueError('resolved profile changed')
            metrics=inspect_output(prefix)
            required=json.loads((inputs/'result.json').read_text())['wire_counts']
            if metrics['native_image_events']!=required['C'] or metrics['native_pressure_events']!=required['P']:
                raise ValueError('estimator did not account for every observation')
        except Exception as exc:
            error=str(exc); metrics=None
        outcome=dict(**trial,status='failed' if error else 'success',exit_code=code,error=error,
            wall_seconds=time.perf_counter()-start,output=metrics,
            artifacts_sha256={p.name:sha(p) for p in sorted(directory.iterdir()) if p.is_file()})
        new_json(directory/'outcome.json',outcome); outcomes.append(outcome)
        print(json.dumps({k:outcome[k] for k in ['name','status','error','wall_seconds']}),flush=True)
    changed=[p for p,h in bindings.items() if sha(p)!=h]
    if sha(inputs/'input.bin')!=input_sha: changed.append(str(inputs/'input.bin'))
    successes=[o for o in outcomes if o['status']=='success']
    common=None
    if len(successes)==4:
        common=[max(o['output']['scoring_support_ns'][0] for o in successes),
                min(o['output']['scoring_support_ns'][1] for o in successes)]
        if common[0]>common[1]: common=None
    freeze=dict(schema='ntnu-mono-comparison-freeze-v5',plan_sha256=sha(output/'plan.json'),
        trials=outcomes,common_support_ns=common,source_changes=changed,
        integrity_status='invalid' if changed else 'verified',reference_values_read=False,
        scoring_status='not_executed_no_independent_hardware_truth')
    new_json(output/'freeze.json',freeze)
    if changed: raise ValueError('source/input changed during trials; freeze invalid')
    return freeze

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();run(a.input,a.output)
