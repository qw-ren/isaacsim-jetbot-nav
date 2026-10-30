"""Offline SDK frame-time validation using reference poses; never imported online."""
import argparse,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();r=a.case;assert(r/'RUNNER_RESULT.json').exists()
rows=[json.loads(l)for l in(r/'native_time_audit.jsonl').read_text().splitlines()if l.strip()];truth=[json.loads(l)for l in(r/'truth_trajectory.jsonl').read_text().splitlines()if l.strip()];by={v['stamp']:v for v in truth};ts=np.array(sorted(by));xyz=np.array([by[t]['xyz']for t in ts]);q=np.array([by[t]['wxyz'][1:]+by[t]['wxyz'][:1]for t in ts]);rot=Rotation.from_quat(q);slerp=Slerp(ts,rot);mount=json.loads((r/'camera_contract.json').read_text())['mounts']['jetbot/lidar'];mt=np.array(mount['T']);mr=Rotation.from_matrix(mount['R']);records=[]
def sensor_at(t):
 rr=slerp([t])[0];position=np.array([np.interp(t,ts,xyz[:,i])for i in range(3)])+rr.apply(mt);return position,rr*mr
for row in rows:
 t=row['capture'];duration=np.ceil(row['delta_max_ns']/1e9/.005)*.005
 if t-duration-.025<ts[0]or t+.025>ts[-1]:continue
 before=sensor_at(t-.02);after=sensor_at(t);speed=np.linalg.norm(after[0]-before[0])/.02;angular=(before[1].inv()*after[1]).magnitude()/.02
 if speed<.08 and angular<.1:continue
 values={}
 candidates=[('end_at_capture',t,'transform'),('end_one_frame_earlier',t-.02,'transform'),('end_one_frame_later',t+.02,'transform'),('start_at_capture_minus_span',t-duration,'transform_start')]
 if 'monotonic_now'in row:
  measured_end=t-(row['monotonic_now']-row['native']-duration)
  if ts[0]<=measured_end-duration<=measured_end<=ts[-1]:candidates.extend([('end_at_native_mono_clock',measured_end,'transform'),('start_at_native_mono_clock',measured_end-duration,'transform_start')])
 for clock_name,end_stamp in [('render_product',row.get('render_times',{}).get('simulationTime')),('timeline',None if 'timeline_now'not in row else t-(row['timeline_now']-row['native']-duration))]:
  if end_stamp is not None and ts[0]<=end_stamp-duration<=end_stamp<=ts[-1]:candidates.extend([('end_at_'+clock_name,end_stamp,'transform'),('start_at_'+clock_name,end_stamp-duration,'transform_start')])
 for name,stamp,field in candidates:
  matrix=np.array(row[field]);expected,rr=sensor_at(stamp);sdk=Rotation.from_matrix(matrix[:3,:3].T);values[name]=dict(position_m=float(np.linalg.norm(matrix[3,:3]-expected)),orientation_rad=float((rr.inv()*sdk).magnitude()))
 records.append(dict(capture=t,duration_s=float(duration),speed_m_s=float(speed),angular_rad_s=float(angular),errors=values))
assert len(records)>30;summary={}
for key in sorted({k for row in records for k in row['errors']}):
 summary[key]={'frames':sum(key in row['errors']for row in records)}
 for metric in ['position_m','orientation_rad']:
  v=np.array([row['errors'][key][metric]for row in records if key in row['errors']]);summary[key][metric]=dict(median=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(v.max()))
models=[('end_at_render_product','start_at_render_product'),('end_at_timeline','start_at_timeline'),('end_at_native_mono_clock','start_at_native_mono_clock'),('end_at_capture','start_at_capture_minus_span')];valid=[]
for end_key,start_key in models:
 if end_key in summary and start_key in summary and summary[end_key]['frames']>=30 and summary[start_key]['frames']>=30 and summary[end_key]['position_m']['p95']<.001 and summary[end_key]['orientation_rad']['p95']<.005 and summary[start_key]['position_m']['p95']<.001 and summary[start_key]['orientation_rad']['p95']<.005:valid.append(end_key)
passed=bool(valid);end_key=valid[0]if valid else None
report=dict(passed=passed,validated_clock_candidate=end_key,scope='offline moving-frame timing validation only; no reference pose is used to alter online timestamps or estimates',moving_frames=len(records),summary=summary,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),inputs={n:hashlib.sha256((r/n).read_bytes()).hexdigest()for n in ['native_time_audit.jsonl','truth_trajectory.jsonl','camera_contract.json']});out=r/'NATIVE_FRAME_TIME_VALIDATION.json';assert not out.exists();out.write_text(json.dumps(report,indent=2)+'\n');out.with_name('NATIVE_FRAME_TIME_VALIDATION_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(report,indent=2))
