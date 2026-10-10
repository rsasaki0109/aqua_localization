"""Post-reference-access clock repair; immutable v1 scorer and trials stay bound."""
import argparse
import bisect
import collections
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from mbes_blind_comparison import sha, save, validate
from vio_metrics import correspond, rigid_score
from check_vio_scores import independent_correspond, horn_score


def frozen(root):
    plan=validate(root);f=json.loads((root/'freeze.json').read_text())
    if (f.get('all_four_outcomes_frozen') is not True or f['plan_sha256']!=sha(root/'plan.json') or
        [x['id'] for x in f['trials']]!=[x['id'] for x in plan['trials']]):
        raise ValueError('all four outcomes must be frozen before reference access')
    for t in f['trials']:
        p=root/'trials'/t['id']
        if t['status']!='success' or sha(p/'result.json')!=t['result_sha256']:raise ValueError('failed/mutated trial blocks reference scoring')
        result=json.loads((p/'result.json').read_text())
        if dict(result,result_sha256=t['result_sha256'])!=t:raise ValueError('freeze/result disagreement')
        for n,d in t['output_sha256'].items():
            if Path(n).name!=n or sha(p/n)!=d:raise ValueError('mutated trajectory blocks reference access')
    if sha(plan['native'])!=plan['native_sha256']:raise ValueError('native object changed')
    return plan,f


def fixed_transforms(path):
    tree=ET.parse(path);joints=[]
    for j in tree.findall('joint'):
        if j.attrib.get('type')!='fixed':raise ValueError('nonfixed publisher URDF')
        parent=j.find('parent').attrib['link'];child=j.find('child').attrib['link'];origin=j.find('origin')
        xyz=[float(x) for x in origin.attrib.get('xyz','0 0 0').split()] if origin is not None else [0,0,0]
        rpy=[float(x) for x in origin.attrib.get('rpy','0 0 0').split()] if origin is not None else [0,0,0]
        transform=np.eye(4);transform[:3,3]=xyz;transform[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix()
        joints.append((parent,child,transform))
    roots={a for a,b,t in joints}-{b for a,b,t in joints}
    if len(roots)!=1:raise ValueError('URDF is not one fixed tree')
    poses={next(iter(roots)):np.eye(4)}
    while joints:
        pending=[]
        for parent,child,t in joints:
            if parent in poses:poses[child]=poses[parent]@t
            else:pending.append((parent,child,t))
        if len(pending)==len(joints):raise ValueError('disconnected URDF')
        joints=pending
    return poses


def ns(header):return int(header.stamp.sec)*1000000000+int(header.stamp.nanosec)


def gps_timestamp_ns(m):
    return int(m.timestamp.sec)*1000000000+int(m.timestamp.nanosec)


def quality_reasons(m,policy):
    reasons=[]
    for key,expected in [('gnssFixOK',True),('diffSoln',True),('invalidLlh',False),
                          ('validTime',True),('fullyResolved',True),('carrSoln',2)]:
        if getattr(m,key)!=expected:reasons.append(key)
    if m.raw.fixType!=3:reasons.append('not_3d_gnss_fix')
    for key,bound in [('hAcc',policy['max_horizontal_accuracy_m']),('vAcc',policy['max_vertical_accuracy_m'])]:
        v=float(getattr(m,key))
        if not math.isfinite(v) or v<0 or v>bound:reasons.append(key)
    return reasons


def ecef(lat,lon,alt):
    phi,lam=math.radians(lat),math.radians(lon);e2=6.6943799901413165e-3
    n=6378137.0/math.sqrt(1-e2*math.sin(phi)**2)
    return np.array([(n+alt)*math.cos(phi)*math.cos(lam),(n+alt)*math.cos(phi)*math.sin(lam),
                     (n*(1-e2)+alt)*math.sin(phi)])


def score(root):
    plan,freeze=frozen(root)
    registration=json.loads((root/'hardware-scoring-v2-plan.json').read_text())
    if registration['source_sha256']!=sha(Path(__file__)) or registration['freeze_sha256']!=sha(root/'freeze.json'):
        raise ValueError('v2 scoring registration changed')
    plan=dict(plan,hardware_scoring=dict(plan['hardware_scoring'],quality_clock='native UBX GPS timestamp; zero offset'))
    target=root/'hardware-scored-v2';target.mkdir(exist_ok=False)
    save(target/'reference_access_started.json',dict(freeze_sha256=sha(root/'freeze.json'),
         plan_sha256=sha(root/'plan.json'),policy=plan['hardware_scoring']))
    policy=plan['hardware_scoring'];store=get_typestore(Stores.ROS1_NOETIC)
    positions=[];quality=[];native_counts=collections.Counter()
    with Reader(plan['native']) as r:
        chosen=[c for c in r.connections if c.topic in (policy['position_topic'],policy['quality_topic'])]
        if {c.topic for c in chosen}!={policy['position_topic'],policy['quality_topic']}:raise ValueError('hardware topics absent')
        for c in chosen:
            if c.msgtype not in store.types:store.register(get_types_from_msg(c.msgdef.data,c.msgtype))
        for c,t,raw in r.messages(connections=chosen):
            m=store.deserialize_ros1(raw,c.msgtype);native_counts[c.topic]+=1
            if c.topic==policy['quality_topic']:quality.append((gps_timestamp_ns(m),quality_reasons(m,policy)))
            else:positions.append((ns(m.header),float(m.latitude),float(m.longitude),float(m.altitude),m.header.frame_id))
    quality.sort();qstamps=[x[0] for x in quality];qualified=[];rejected=[]
    for stamp,lat,lon,alt,frame in sorted(positions):
        index=bisect.bisect_left(qstamps,stamp)
        choices=[i for i in (index-1,index) if 0<=i<len(quality)]
        if not choices:rejected.append(dict(timestamp_ns=stamp,reasons=['no_quality_packet']));continue
        i=min(choices,key=lambda i:(abs(qstamps[i]-stamp),qstamps[i]))
        reasons=list(quality[i][1])
        if abs(qstamps[i]-stamp)>policy['max_quality_time_difference_ns']:reasons.append('quality_clock_gap')
        if not (math.isfinite(lat) and -90<=lat<=90 and math.isfinite(lon) and -180<=lon<=180 and math.isfinite(alt)):
            reasons.append('invalid_position')
        if reasons:rejected.append(dict(timestamp_ns=stamp,reasons=reasons));continue
        qualified.append((stamp,ecef(lat,lon,alt),frame))
    save(target/'reference-quality.json',dict(native_counts=dict(native_counts),qualified_count=len(qualified),
         rejection_count=len(rejected),rejections=rejected,frame_counts=dict(collections.Counter(x[2] for x in qualified)),
         policy=policy,hash_provenance='native bag local verified hash; hardware quality is conditional, not new publisher ground truth certification'))
    if len(qualified)<3:raise ValueError('insufficient fixed-RTK quality-qualified positions; no hardware accuracy claim')
    if len({x[0] for x in qualified})!=len(qualified):raise ValueError('duplicate qualified reference timestamps')
    frames=fixed_transforms(plan['urdf']);reference_frames={x[2] for x in qualified}
    if len(reference_frames)!=1:raise ValueError('antenna frame changes; not fitted')
    frame=next(iter(reference_frames))
    if frame not in frames or plan['sensor_frame'] not in frames:raise ValueError('unregistered hardware antenna/sensor frame')
    lever=(np.linalg.inv(frames[plan['sensor_frame']])@frames[frame])[:3,3]
    # ECEF reference minus a fixed first qualified point is only conditioning;
    # SE(3) translation is subsequently estimated once, with scale fixed to 1.
    rstamps=[x[0] for x in qualified];origin=qualified[0][1];rpoints=[x[1]-origin for x in qualified]
    matched_by_trial={};scores=[]
    for trial in freeze['trials']:
        rows=list(csv.DictReader((root/'trials'/trial['id']/'trajectory.csv').open()))
        estamp=[];epoints=[]
        for row in rows:
            estamp.append(int(row['timestamp_ns']));p=np.array([float(row[k]) for k in ('x','y','z')])
            q=[float(row[k]) for k in ('qx','qy','qz','qw')]
            epoints.append(p+Rotation.from_quat(q).apply(lever))
        # Query at reference epochs; interpolate estimator intervals <=0.1s.
        matched,truth,estimate,excluded=correspond(rstamps,rpoints,estamp,epoints)
        other,otruth,oestimate=independent_correspond(rstamps,rpoints,estamp,epoints)
        if matched!=other or not np.allclose(estimate,oestimate,rtol=1e-10,atol=1e-10):raise ValueError('independent time matching disagrees')
        primary=rigid_score(estimate,truth);independent=horn_score(oestimate,otruth)
        for k in ('rotation','translation_m','rmse_m'):
            if not np.allclose(primary[k],independent[k],rtol=1e-8,atol=1e-8):raise ValueError('independent Horn disagrees')
        matched_by_trial[trial['id']]={t:(a,b) for t,a,b in zip(matched,estimate,truth)}
        scores.append(dict(id=trial['id'],score=primary,independent_horn=independent,
            matched_timestamps_ns=matched,interpolation_exclusions=excluded,
            runtime_seconds=trial['runtime_seconds'],output_count=trial['output_count'],support_ns=trial['support_ns']))
    common=sorted(set.intersection(*(set(v) for v in matched_by_trial.values())))
    common_scores={}
    for trial_id,pairs in matched_by_trial.items():
        a=[pairs[t][0] for t in common];b=[pairs[t][1] for t in common]
        primary=rigid_score(a,b);check=horn_score(a,b)
        if not np.allclose(primary['rmse_m'],check['rmse_m'],rtol=1e-8,atol=1e-8):raise ValueError('common Horn disagreement')
        common_scores[trial_id]=dict(primary,independent_horn=check)
    result=dict(schema='mbes-hardware-position-scores-v2',post_reference_access_repair=True,scoring_plan_sha256=sha(root/'hardware-scoring-v2-plan.json'),freeze_sha256=sha(root/'freeze.json'),
        qualified_reference_count=len(qualified),antenna_frame=frame,antenna_in_sensor_m=lever.tolist(),
        trials=scores,common_all_four=dict(count=len(common),timestamps_ns=common,scores=common_scores),
        scale=1,max_interpolation_gap_ns=100000000,sota_achieved=False,
        limitation='short-window point-only odometry on a surface-vessel MBES dataset, not full underwater SLAM SoTA')
    save(target/'scores.json',result);return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True);args=parser.parse_args()
    try: result=score(args.root);print(json.dumps(result['common_all_four'],indent=2))
    except Exception as exc:
        save(args.root/'hardware_scoring_v2_failure.json',dict(error=str(exc),status='failed_no_accuracy_claim'));raise
