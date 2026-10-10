"""Freeze a sensor-only GICP/KISS-ICP comparison before hardware-reference access."""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import struct
import subprocess
import time

import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

FRAMES=1600
TOPIC='/wassp/detections'
SOURCES=['scripts/benchmarks/'+n for n in ('mbes_blind_comparison.py','mbes_offline.cpp',
    'mbes_native_build/CMakeLists.txt','mbes_hardware_score.py','vio_metrics.py','check_vio_scores.py')]
SOURCES+=['aqua_sonar_loc/src/scan_matcher.cpp','aqua_sonar_loc/include/aqua_sonar_loc/scan_matcher.hpp',
          'aqua_sonar_loc/include/aqua_sonar_loc/sonar_cloud_preprocessor.hpp']


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()


def save(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2);f.write('\n')


def points(message):
    fields={f.name:f for f in message.fields}
    if not all(name in fields and fields[name].datatype==7 and fields[name].count==1 for name in ('x','y','z')):
        raise ValueError('unregistered point coordinate schema')
    endian='>' if message.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,
                    'offsets':[fields[n].offset for n in ('x','y','z')],'itemsize':message.point_step})
    array=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                     strides=(message.row_step,message.point_step))
    xyz=np.column_stack([array[n].reshape(-1) for n in ('x','y','z')]).astype('<f4')
    finite=np.all(np.isfinite(xyz),axis=1);distance=np.linalg.norm(xyz.astype(float),axis=1)
    keep=finite & (distance>0) & (distance<60)
    return xyz[keep],{'native_points':len(xyz),'nonfinite':int(sum(~finite)),
                     'finite_outside_common_range':int(sum(finite & ~keep)),'common_points':int(sum(keep))}


def kiss_config():
    from kiss_icp.config import KISSConfig
    # Explicit values avoid environment-dependent BaseSettings overrides.
    return KISSConfig(data={'max_range':60.0,'min_range':0.0,'deskew':False},
        registration={'max_num_iterations':500,'convergence_criterion':0.0001,'max_num_threads':1},
        mapping={'voxel_size':0.6,'max_points_per_voxel':20},
        adaptive_threshold={'fixed_threshold':None,'initial_threshold':2.0,'min_motion_th':0.1},out_dir='unused')


def register(native,root,binary,urdf):
    import kiss_icp
    repo=Path(__file__).resolve().parents[2];root.mkdir(parents=True,exist_ok=False)
    # Full native hash before deriving inputs; retained native contains withheld truth.
    native_sha=sha(native);store=get_typestore(Stores.ROS1_NOETIC);ledger=[]
    with Reader(native) as r, (root/'points.bin').open('xb') as wire:
        selected=[c for c in r.connections if c.topic==TOPIC]
        if not selected or any(c.msgtype!='sensor_msgs/msg/PointCloud2' for c in selected):raise ValueError('WASSP source missing')
        wire.write(b'MBESPC01'+struct.pack('<I',FRAMES));previous=None
        for ordinal,(c,bag_ns,raw) in enumerate(r.messages(connections=selected)):
            if ordinal==FRAMES:break
            m=store.deserialize_ros1(raw,c.msgtype);ns=int(m.header.stamp.sec)*1000000000+int(m.header.stamp.nanosec)
            if previous is not None and ns<=previous:raise ValueError('nonmonotonic native point header')
            if m.header.frame_id!='wassp':raise ValueError('unregistered sonar frame')
            previous=ns;xyz,counts=points(m)
            wire.write(struct.pack('<qII',ns,ordinal,len(xyz)));wire.write(xyz.tobytes())
            ledger.append(dict(native_ordinal=ordinal,header_ns=ns,bag_ns=bag_ns,
                raw_sha256=hashlib.sha256(raw).hexdigest(),frame=m.header.frame_id,**counts))
    if len(ledger)!=FRAMES:raise ValueError('short native point window')
    save(root/'input-ledger.json',ledger)
    artifacts=[Path(binary),Path(urdf),root/'points.bin',root/'input-ledger.json']
    package=Path(kiss_icp.__file__).parent
    artifacts+=sorted(p for p in package.rglob('*') if p.suffix in ('.py','.so'))
    artifacts+=list(Path('/workspace/aqua-next/comparison/wheels').glob('kiss_icp-1.3.0-*.whl'))
    plan=dict(schema='mbes-hardware-blind-comparison-v1',native=str(native),native_sha256=native_sha,
        window='first 1600 native WASSP point packets, in ordinal order; no reference-selected window',
        topic=TOPIC,sensor_frame='wassp',input=str(root/'points.bin'),frames=FRAMES,
        input_contract='native integer header time; offset=0; finite Float32 XYZ with 0<range<60m; report every packet/filter',
        minimum_common_points=60,sensors_used='point clouds only; same mask; no IMU/GNSS/TF input to either arm',
        gicp=dict(correspondence_distance=1.5,iterations=80,epsilon=1e-6,fitness_gate=3.0,
            translation_gate=1.0,rotation_gate=0.4,submap_size=1,motion_prior=False),
        kiss_version=importlib.metadata.version('kiss-icp'),kiss_config=kiss_config().model_dump(),
        upstream_git_clone='authentication unavailable; official pinned PyPI wheel/source hashes used',
        binary=str(binary),urdf=str(urdf),source_sha256={n:sha(repo/n) for n in SOURCES},
        artifact_sha256={str(p):sha(p) for p in artifacts},
        trials=[dict(id=f'{arm}-{repeat}',arm=arm,repeat=repeat) for arm in ('gicp','kiss') for repeat in (1,2)],
        hardware_scoring=dict(position_topic='/nav/sensors/navsat/ubx_pos/fix',
            quality_topic='/nav/sensors/navsat/ubx_pos/raw/pvt',carrier_solution=2,
            gnss_fix_ok=True,differential=True,invalid_llh=False,valid_time=True,fully_resolved=True,
            max_horizontal_accuracy_m=0.1,max_vertical_accuracy_m=0.2,max_quality_time_difference_ns=20000000,
            clock='native header nanoseconds, no time optimization',scale=1,alignment='rigid SE(3)',
            interpolation='estimate to quality-qualified reference epochs; no extrapolation',max_gap_ns=100000000,
            antenna_lever_arm='fixed publisher URDF; require reference frame in fixed chain; no fitted lever arm'),
        reference_values_read=False,sequence_role='new sequence for this implementation; public history cannot establish universal independence',
        sota_achieved=False)
    if plan['kiss_version']!='1.3.0':raise ValueError('wrong competitor release')
    for n in SOURCES:
        p=root/'source'/n;p.parent.mkdir(parents=True,exist_ok=True)
        with p.open('xb') as f:f.write((repo/n).read_bytes())
    save(root/'plan.json',plan);return plan


def validate(root):
    repo=Path(__file__).resolve().parents[2];plan=json.loads((root/'plan.json').read_text())
    for n,digest in plan['source_sha256'].items():
        if sha(repo/n)!=digest or sha(root/'source'/n)!=digest:raise ValueError('registered source changed')
    for n,digest in plan['artifact_sha256'].items():
        if sha(n)!=digest:raise ValueError('registered artifact changed')
    return plan


def frames(path):
    with Path(path).open('rb') as f:
        if f.read(8)!=b'MBESPC01':raise ValueError('wrong wire contract')
        total=struct.unpack('<I',f.read(4))[0]
        for _ in range(total):
            ns,ordinal,n=struct.unpack('<qII',f.read(16));b=f.read(n*12)
            if len(b)!=n*12:raise ValueError('truncated point stream')
            yield ns,ordinal,np.frombuffer(b,dtype='<f4').reshape(n,3).astype(float)
        if f.read(1):raise ValueError('trailing points')


def run_kiss(input_path,target):
    from kiss_icp.kiss_icp import KissICP
    from scipy.spatial.transform import Rotation
    model=KissICP(kiss_config())
    with (target/'trajectory.csv').open('x') as out,(target/'events.csv').open('x') as events:
        writer=csv.writer(out);writer.writerow(['timestamp_ns','native_ordinal','x','y','z','qx','qy','qz','qw'])
        event=csv.writer(events);event.writerow(['timestamp_ns','native_ordinal','status'])
        for ns,ordinal,xyz in frames(input_path):
            if len(xyz)<60:event.writerow([ns,ordinal,'insufficient_common_points']);continue
            model.register_frame(xyz,np.array([],dtype=float));pose=model.last_pose
            if not np.all(np.isfinite(pose)):raise ValueError('nonfinite KISS pose')
            writer.writerow([ns,ordinal,*pose[:3,3],*Rotation.from_matrix(pose[:3,:3]).as_quat()])
            event.writerow([ns,ordinal,'estimated'])


def run_all(root):
    plan=validate(root);results=[]
    for trial in plan['trials']:
        validate(root);target=root/'trials'/trial['id'];target.mkdir(parents=True,exist_ok=False)
        start=time.monotonic();error=None
        try:
            if trial['arm']=='gicp':
                with (target/'run.log').open('xb') as log:
                    subprocess.run([plan['binary'],plan['input'],str(target/'trajectory.csv'),str(target/'events.csv')],check=True,stdout=log,stderr=log)
            else:run_kiss(plan['input'],target)
        except Exception as exc:error=str(exc)
        runtime=time.monotonic()-start
        trajectory=target/'trajectory.csv';rows=list(csv.DictReader(trajectory.open())) if trajectory.exists() else []
        events=target/'events.csv';counts={}
        if events.exists():
            for e in csv.DictReader(events.open()):counts[e['status']]=counts.get(e['status'],0)+1
        if error is None and (sum(counts.values())!=plan['frames'] or len(rows)<3):error='incomplete event processing or too few outputs'
        result=dict(**trial,status='failed' if error else 'success',error=error,runtime_seconds=runtime,
                    output_count=len(rows),event_status_counts=counts,
                    support_ns=[int(rows[0]['timestamp_ns']),int(rows[-1]['timestamp_ns'])] if rows else None,
                    output_sha256={p.name:sha(p) for p in target.iterdir() if p.is_file()},reference_values_read=False)
        save(target/'result.json',result);results.append(dict(result,result_sha256=sha(target/'result.json')))
        print(json.dumps({k:result[k] for k in ('id','status','runtime_seconds','output_count','error')}),flush=True)
    validate(root)
    save(root/'freeze.json',dict(schema='mbes-all-four-freeze-v1',plan_sha256=sha(root/'plan.json'),
         trials=results,all_four_outcomes_frozen=True,reference_values_read=False,sota_achieved=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('stage',choices=['register','run','kiss'])
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--native',type=Path)
    parser.add_argument('--binary',type=Path);parser.add_argument('--urdf',type=Path);args=parser.parse_args()
    if args.stage=='register':register(args.native,args.root,args.binary,args.urdf)
    elif args.stage=='run':run_all(args.root)
    else:run_kiss(args.native,args.root)
