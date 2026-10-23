"""Offline accuracy, path endpoints and actual moving-insertion evidence."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3
import sys
import zlib

import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Imu, Image as CameraImage
from PIL import Image

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from analyze_lio_dataset import load
from trajectory_metrics import evaluate

p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if not (a.case/'RUNNER_RESULT.json').exists():raise SystemExit('Capture still active')
a.output.mkdir(exist_ok=False)
analysis_sources=[Path(__file__).resolve(),Path(__file__).resolve().parent.parent/'warehouse_lio/analyze_lio_dataset.py',Path(__file__).resolve().parent.parent/'warehouse_lio/trajectory_metrics.py',Path(__file__).resolve().parent.parent/'warehouse_lio/lio_nav_math.py']
(a.output/'analysis_source').mkdir()
for source in analysis_sources: shutil.copy2(source,a.output/'analysis_source'/source.name)
(a.output/'analysis_source.sha256').write_text(''.join(hashlib.sha256(source.read_bytes()).hexdigest()+'  analysis_source/'+source.name+'\n' for source in analysis_sources))
db=sqlite3.connect(f'file:{a.case/"recording/sensors.sqlite3"}?mode=ro',uri=True)
truth=load(db,'/truth/odom',Odometry);raw=load(db,'/lio/odometry',Odometry)
metrics,series=evaluate(truth,raw)
report=dict(raw_estimator_accuracy=metrics,truth_usage='offline evaluation only; no controller/estimator input',
            source_case=str(a.case),source_case_config_sha256=hashlib.sha256((a.case/'CASE.json').read_bytes()).hexdigest())
config=json.loads((a.case/'CASE.json').read_text())
if config.get('imu_noise_enabled') is False:
    native={r['stamp']:r for r in load(db,'/evaluation/imu_ideal',Imu)}
    supplied=load(db,'/lio/imu',Imu)
    paired=[(r,native[r['stamp']]) for r in supplied if r['stamp'] in native]
    maximum=max((max(abs(x-y) for field in ['acc','gyro'] for x,y in zip(r[field],n[field])) for r,n in paired),default=None)
    report['native_imu_proof']=dict(paired_samples=len(paired),supplied_samples=len(supplied),maximum_acceleration_gyro_difference=maximum,
        artificial_noise_disabled=True,exact_samples_preserved=bool(paired) and maximum==0.,
        note='Historical noise-profile seed/bias fields can remain in metadata but are inactive; supplied vectors are checked directly')
    if not paired or maximum!=0.:raise ValueError('Native IMU bypass verification failed')
nav=json.loads((a.case/'navigation/navigation_check.json').read_text())
plans=[]
for wall,payload in db.execute('SELECT wall,payload FROM records WHERE topic=? ORDER BY wall',('/plan',)):
    m=deserialize_message(zlib.decompress(payload),NavPath)
    if m.poses:
        q=m.poses[-1].pose.position;plans.append(dict(wall=wall,endpoint=[q.x,q.y]))
report['native_plan_evidence_source']='recorded /plan CDR'
if not plans:
    telemetry=a.case/'navigation/telemetry.json'
    if telemetry.exists():
        plans=[dict(wall=r['wall'],endpoint=r['xy'][-1]) for r in json.loads(telemetry.read_text()).get('plans',[]) if r.get('xy')]
        report['native_plan_evidence_source']='navigation observer /plan telemetry; CDR absent'
checks=[]
for i,leg in enumerate(nav.get('legs',[])):
    start=leg['start_pose']['wall']+.5
    end=nav['legs'][i+1]['start_pose']['wall'] if i+1<len(nav['legs']) else float('inf')
    observed=[r for r in plans if start<=r['wall']<end]
    errors=[float(np.linalg.norm(np.array(r['endpoint'])-leg['goal'])) for r in observed]
    checks.append(dict(leg=i,goal=leg['goal'],native_plans=len(observed),
                       endpoint_error_max_m=max(errors) if errors else None))
report['native_plan_goal_checks']=checks
applied=a.case/'navigation/APPLIED.json'
if applied.exists():
    state=json.loads(applied.read_text());stamp=state['simulation_stamp_s']
    images=[]
    for label,target in [('before',stamp-.2),('after',stamp+.2),('after_1s',stamp+1.)]:
        row=db.execute('SELECT stamp_ns,payload FROM records WHERE topic=? AND stamp_ns>=? ORDER BY stamp_ns,id LIMIT 1',('/livo/image_raw',round(target*1e9))).fetchone()
        if row is None:continue
        m=deserialize_message(zlib.decompress(row[1]),CameraImage)
        if m.encoding not in ['rgb8','bgr8']:raise ValueError('Unexpected recorded image encoding')
        pixels=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='bgr8':pixels=pixels[:,:,::-1]
        f=a.output/('insertion_'+label+'.png');Image.fromarray(pixels).save(f)
        images.append(dict(file=f.name,stamp_s=row[0]/1e9,sha256=hashlib.sha256(f.read_bytes()).hexdigest(),source='actual recorded native RGB, no generated rendering'))
    report['recorded_insertion_images']=images
    t=np.array([r['stamp'] for r in truth]);xyz=np.array([r['xyz'] for r in truth])
    before=xyz[(t>=5.)&(t<=stamp),:2]
    report['reference_path_before_insertion_m']=float(np.linalg.norm(np.diff(before,axis=0),axis=1).sum())
    visits=[]
    for leg in nav.get('legs',[]):
        if leg.get('waypoints'):
            visits.append(dict(native_action=leg.get('native_action'),
                minimum_remaining=min((v['remaining'] for v in leg.get('feedback',[])),default=None),
                closest_reference_distances_m=[dict(xy=xy,distance=float(np.linalg.norm(xyz[:,:2]-xy,axis=1).min())) for xy in leg.get('pre_insertion_waypoints',leg['waypoints'])]))
    report['continuous_prelude_evidence']=visits
    start=stamp-.1;end=stamp+.1
    pos=np.column_stack([np.interp([start,end],t,xyz[:,i]) for i in range(3)])
    speed=float(np.linalg.norm(pos[1,:2]-pos[0,:2])/(end-start))
    selected=xyz[t>=stamp,:2];clearance=float('inf')
    for b in state['dynamic_boxes']:
        lo,hi=np.array(b['min'][:2]),np.array(b['max'][:2])
        clearance=min(clearance,float(np.linalg.norm(np.maximum(np.maximum(lo-selected,selected-hi),0),axis=1).min()))
    report['actual_insertion']=dict(stamp_s=stamp,reference_planar_speed_mps=speed,
        motion_confirmed=speed>.1,minimum_center_to_added_aabb_m=clearance,
        body_center_outside_obstacle_margin=clearance>.45,reference_used_for_control=False)
    box=state['dynamic_boxes'][0]
    band=selected[(selected[:,0]>=box['min'][0]-.1)&(selected[:,0]<=box['max'][0]+.1)]
    report['actual_insertion']['crossing_y_range_m']=[float(band[:,1].min()),float(band[:,1].max())] if len(band) else None
    if state['kind']=='C':
        traversal=bool(len(selected)) and selected[:,0].max()<box['min'][0]-.45
        report['actual_insertion']['stayed_before_full_barrier']=bool(traversal)
    else:
        traversal=bool(len(band)) and selected[-1,0]>box['max'][0]+.45
        report['actual_insertion']['physically_passed_obstacle']=bool(traversal)
    report['physical_checks_passed']=bool(speed>.1 and clearance>.45 and traversal)
(a.output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
with gzip.open(a.output/'trajectories.json.gz','wt') as f:
    json.dump(dict(raw_truth=truth,raw_estimates=raw,aligned_series=series,plan_endpoints=plans),f)
print(json.dumps(report,indent=2))
