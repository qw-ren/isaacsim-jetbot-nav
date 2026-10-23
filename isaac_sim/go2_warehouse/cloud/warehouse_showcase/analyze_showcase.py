"""Offline showcase facts, native IMU equality and display audit; no control output."""
import argparse,gzip,hashlib,json,sqlite3,sys
from pathlib import Path
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from analyze_lio_dataset import load
from trajectory_metrics import evaluate
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
assert (a.case/'RUNNER_RESULT.json').exists();a.output.mkdir(exist_ok=False)
(a.output/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
(a.output/'analysis_source.sha256').write_text(hashlib.sha256(Path(__file__).read_bytes()).hexdigest()+'  analysis_source.py\n')
db=sqlite3.connect(f'file:{a.case/"recording/sensors.sqlite3"}?mode=ro',uri=True)
truth=load(db,'/truth/odom',Odometry);est=load(db,'/lio/odometry',Odometry);accuracy,series=evaluate(truth,est)
t=np.array([v['stamp'] for v in truth]);xy=np.array([v['xyz'][:2] for v in truth]);keep=np.r_[True,np.diff(t)>0];t,xy=t[keep],xy[keep]
path=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
def at(stamp):return np.array([np.interp(stamp,t,xy[:,i]) for i in range(2)])
def speed(stamp):return float(np.linalg.norm(at(stamp+.1)-at(stamp-.1))/.2)
navfile=a.case/'navigation/navigation_check.json'
nav=json.loads(navfile.read_text()) if navfile.exists() else dict(passed=False,error='requested stop; native completion report missing',checks={},legs=[],events=json.loads((a.case/'navigation/EVENTS.json').read_text()) if (a.case/'navigation/EVENTS.json').exists() else [])
events=[]
for e in nav.get('events',[]):
 stamp=e['applied']['simulation_stamp_s'];index=np.searchsorted(t,stamp)
 boxes=e['applied']['dynamic_boxes'];selected=xy[index:];clear=[]
 for b in boxes:
  lo,hi=np.array(b['min'][:2]),np.array(b['max'][:2]);clear.append(float(np.linalg.norm(np.maximum(np.maximum(lo-selected,selected-hi),0),axis=1).min()))
 events.append(dict(name=e['name'],kind=e['kind'],simulation_stamp_s=stamp,actual_path_before_m=float(np.interp(stamp,t,path)),actual_path_remaining_m=float(path[-1]-np.interp(stamp,t,path)),actual_insertion_speed_mps=speed(stamp),minimum_center_to_present_envelopes_m=min(clear),active=e['active']))
native={v['stamp']:v for v in load(db,'/evaluation/imu_ideal',Imu)};supplied=load(db,'/lio/imu',Imu);paired=[(r,native[r['stamp']]) for r in supplied if r['stamp'] in native]
maximum=max((max(abs(x-y) for field in ['acc','gyro'] for x,y in zip(r[field],v[field])) for r,v in paired),default=None)
audit=[json.loads(row) for row in (a.case/'simulation/display_paths_audit.jsonl').read_text().splitlines()]
grid=np.arange(max(5.,t[0]+.1),t[-1]-.1,.1);speeds=np.array([speed(v) for v in grid])
report=dict(speed_summary=dict(max_mps=float(speeds.max()),moving_median_mps=float(np.median(speeds[speeds>.2])),seconds_above_1_mps=float((speeds>1.).sum()*.1),seconds_above_1_5_mps=float((speeds>1.5).sum()*.1)),case=a.case.name,navigation_passed=nav['passed'],native_checks=nav['checks'],error=nav.get('error'),actual_path_m=float(path[-1]),events=events,
 raw_estimator_accuracy=accuracy,native_imu=dict(paired_samples=len(paired),maximum_difference=maximum,exact_vectors_preserved=bool(paired) and maximum==0),
 display=dict(audit_samples=len(audit),samples_local_path=sum(d['counts'].get('local',0)>1 for d in audit),actual_topics=['/local_plan'],global_display_requested=False,local_shape='actual DWB local plan relative to estimated body, anchored at visible actor for display only',ribbon_width_m=.38,arrow_base_width_m=.85),
 truth_usage='offline accuracy/speed and viewport only; no navigation input',power_operation=False)
camera_files=sorted((a.case/'simulation/follow_video').glob('frame_*.json'))
max_offset=0.;max_captured_offset=0.;local_frames=0
for f in camera_files:
    d=json.loads(f.read_text())
    if 'robot_world' not in d:continue
    robot=d['robot_world'];w,x,y,z=robot[3:7];yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z));co,si=np.cos(yaw),np.sin(yaw);delta=np.array(d['camera_eye_world'])-robot[:3]
    actual=np.array([co*delta[0]+si*delta[1],-si*delta[0]+co*delta[1],delta[2]])
    max_offset=max(max_offset,float(np.abs(actual-[-2.4,0.,1.7]).max()));local_frames+=bool(d.get('local'))
    if d['requested_sim_s']>=20.:
        actual_eye=np.linalg.inv(np.array(d['view']))[3,:3];delta=actual_eye-robot[:3];offset=np.array([co*delta[0]+si*delta[1],-si*delta[0]+co*delta[1],delta[2]])
        max_captured_offset=max(max_captured_offset,float(np.abs(offset-[-2.4,0.,1.7]).max()))
report['fixed_camera_proof']=dict(frame_snapshots=len(camera_files),frames_with_actual_local_plan=local_frames,maximum_requested_relative_offset_error_m=max_offset,maximum_captured_relative_offset_error_after20s_m=max_captured_offset,adaptive_reposition=False)
contract=json.loads((a.case/'simulation/follow_video/contract.json').read_text())
clean_display=contract.get('scene_draw_calls') is False and not (a.case/'SENSOR_ISOLATION_INVALID.json').exists()
report['sensor_display_isolation']=dict(architecture='raw viewport capture; path polygons composed in independent image process only' if clean_display else 'legacy scene drawing; not valid for clean algorithm validation',old_scene_debug_drawing_disabled=clean_display)
report['algorithm_validation_valid']=bool(nav['passed'] and clean_display)
revision=a.case/'DISPLAY_REVISION.json'
if revision.exists():
    rev=json.loads(revision.read_text());source=a.case/'record_display_paths_localonly.py'
    localonly=[d for d in audit if d['sim_s']>rev['requested_sim_s']+2.]
    report['display_revision']=dict(rev,source_verified=hashlib.sha256(source.read_bytes()).hexdigest()==rev['source_sha256'],samples_after_switch=len(localonly),samples_with_global_after_switch=sum(d['counts'].get('global',0)>0 for d in localonly))
(a.output/'metrics.json').write_text(json.dumps(report,indent=2)+'\n')
with gzip.open(a.output/'trajectories.json.gz','wt') as f:json.dump(dict(raw_truth=truth,raw_estimates=est,aligned_series=series),f)
telemetry=a.case/'navigation/telemetry.json'
guards=json.loads(telemetry.read_text())['guard'] if telemetry.exists() else []
lastguard=next((v for v in reversed(guards) if v['reason'] not in ['DISARMED','ENABLED']),None)
outcome=dict(case=a.case.name,native_navigation_passed=nav['passed'],native_status=nav['legs'][-1].get('status') if nav['legs'] else None,error=nav.get('error'),guard_reason=lastguard['reason'] if lastguard else None,guard_stop_sim_s=lastguard.get('sim') if lastguard else None,success_sim_s=nav['legs'][-1]['arrival_pose']['stamp'] if nav['passed'] else None)
(a.output/'VIDEO_OUTCOME.json').write_text(json.dumps(outcome,indent=2)+'\n');print(json.dumps(report,indent=2))
