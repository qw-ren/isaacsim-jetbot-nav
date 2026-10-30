"""Offline-only reference evaluation; no estimator input or full-trajectory fit."""
import argparse,json,sqlite3,zlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--case',type=Path,required=True);a=p.parse_args()
def readrows(f):return [json.loads(x)for x in f.read_text().splitlines()if x.strip()]
def unique(rows):
 rows={r['stamp']:r for r in rows};return [rows[k]for k in sorted(rows)]
def metric(e):return dict(rmse=float(np.sqrt(np.mean(e**2))),median=float(np.median(e)),p95=float(np.percentile(e,95)),maximum=float(np.max(e)),final=float(e[-1]))
def tilt(up):
 up=up/np.linalg.norm(up);v=np.cross(up,[0.,0.,1.]);c=up[2];assert c>-.99
 K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
 return Rotation.from_matrix(np.eye(3)+K+K@K/(1+c))
truth=unique(readrows(a.case/'truth_trajectory.jsonl'));tt=np.array([r['stamp']for r in truth]);tp=np.array([r['xyz']for r in truth]);tq=Rotation.from_quat([r['wxyz'][1:]+r['wxyz'][:1]for r in truth]);ty=tq.as_euler('xyz')[:,2]
contract=json.loads((a.case/'camera_contract.json').read_text());lever=np.array(contract['mounts']['jetbot/imu']['T'])
db=sqlite3.connect('file:'+str(a.root/'capture/sensors.sqlite3')+'?mode=ro',uri=True);ims=[]
for ns,data in db.execute('SELECT stamp_ns,payload FROM records WHERE topic=? AND stamp_ns BETWEEN ? AND ? ORDER BY stamp_ns',('/lio/imu',1000000000,3000000000)):
 m=deserialize_message(zlib.decompress(data),Imu);ims.append((ns/1e9,[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]))
it=np.array([x[0]for x in ims]);ia=np.array([x[1]for x in ims]);assert len(ims)>100
report=dict(scope='single recorded JetBot run; raw IMU-origin estimator outputs converted to chassis; offline reference only',calibration_window_s=[1,3],evaluation_begin_s=7.,alignment='stationary IMU mean (1..3s) rotated by first estimator pose for gravity tilt, first chassis origin -> declared (0,6); declared yaw primary, initial estimated heading normalization secondary; no full trajectory fitting',imu_lever_m=lever.tolist(),methods={})
first_motion=next(r['sim']for r in readrows(a.case/'control_audit.jsonl')if max(map(abs,r['output']))>1e-4);report['first_motion_stamp_s']=first_motion
streams={}
for mode in ['lio','livo','livo_no_image']:
 doc=json.loads((a.root/mode/'odometry.json').read_text());rows=unique(doc['poses']);ts=np.array([r['stamp']for r in rows]);xyz=np.array([r['xyz']for r in rows]);rot=Rotation.from_quat([r['xyzw']for r in rows]);assert np.isfinite(xyz).all();assert ts[0]<first_motion, 'First estimate must precede actual first motion in this captured dataset'
 up=rot[0].apply(np.mean(ia,axis=0));align=tilt(up);pos=xyz-rot.apply(lever);origin=pos[0].copy();pos=align.apply(pos-origin);ori=align*rot;yaw=np.unwrap(ori.as_euler('xyz')[:,2]);yaw0=float(yaw[0]);report['methods'][mode]=dict(output_samples=len(ts),first_stamp_s=float(ts[0]),last_stamp_s=float(ts[-1]),maximum_output_gap_s=float(np.max(np.diff(ts))),initial_heading_deg=float(np.rad2deg(yaw0)),input=doc['statistics']);streams[mode]=(ts,pos,yaw,yaw0)
start=7.;end=min(tt[-1],*(v[0][-1]for v in streams.values()));grid=np.arange(start,end,.05);assert len(grid)>100;ref=np.column_stack([np.interp(grid,tt,tp[:,i])for i in range(3)]);refyaw=np.interp(grid,tt,np.unwrap(ty));report['reference_end_s']=float(tt[-1]);report['common_interval_s']=[start,float(end)];report['common_grid_samples']=len(grid);report['grid_dt_s']=.05
series={'stamp':grid.tolist(),'truth_xy':ref[:,:2].tolist()}
for mode,(ts,pos,yaw,yaw0)in streams.items():
 base=np.column_stack([np.interp(grid,ts,pos[:,i])for i in range(3)]);y=np.interp(grid,ts,yaw)
 for name,heading in [('declared_yaw',0.),('initial_heading',yaw0)]:
  pred=Rotation.from_euler('z',-heading).apply(base)+[0,6,0];err=np.linalg.norm(pred[:,:2]-ref[:,:2],axis=1);ye=np.abs(np.arctan2(np.sin(y-heading-refyaw),np.cos(y-heading-refyaw)));gap_index=np.clip(np.searchsorted(ts,grid),1,len(ts)-1);valid=(ts[gap_index]-ts[gap_index-1])<=.3
  report['methods'][mode][name]=dict(position_xy_m=metric(err),yaw_deg=metric(np.rad2deg(ye)),coverage_grid_fraction=float(valid.mean()),position_xy_valid_only_m=metric(err[valid]))
  if name=='initial_heading':series[mode+'_xy']=pred[:,:2].tolist();series[mode+'_error_m']=err.tolist()
 report['methods'][mode]['relative_translation_1s_m']=metric(np.linalg.norm((base[20:,:2]-base[:-20,:2])-(Rotation.from_euler('z',yaw0).apply(np.c_[ref[20:,:2]-ref[:-20,:2],np.zeros(len(ref)-20)])[:,:2]),axis=1))
 report['methods'][mode]['evaluation_coverage_until_s']=float(min(ts[-1],tt[-1]));report['methods'][mode]['reference_end_gap_s']=float(tt[-1]-min(ts[-1],tt[-1]));report['methods'][mode]['reference_interval_coverage_fraction']=float(max(0.,min(ts[-1],tt[-1])-max(start,ts[0]))/(tt[-1]-start))
keys=['point_input_sha256','imu_input_sha256','imu_timestamps_sha256','image_input_sha256'];report['identical_replayed_inputs']={k:len({report['methods'][m]['input'][k]for m in streams})==1 for k in keys};assert all(report['identical_replayed_inputs'].values())
report['truth_xy_path_m']=float(np.linalg.norm(np.diff(tp[:,:2],axis=0),axis=1).sum());out=a.root/'COMPARISON.json';assert not out.exists();out.write_text(json.dumps(report,indent=2));(a.root/'COMPARISON_SERIES.json').write_text(json.dumps(series));print(json.dumps(report,indent=2))
