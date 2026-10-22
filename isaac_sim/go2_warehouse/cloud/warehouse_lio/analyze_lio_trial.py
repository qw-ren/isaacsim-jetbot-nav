"""First-pose SE(3) alignment only; simulation truth is evaluation-only."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
p=argparse.ArgumentParser();p.add_argument('--observer',type=Path,required=True);p.add_argument('--motion',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--plot',type=Path);a=p.parse_args();raw=json.loads((a.observer/'telemetry.json').read_text())
def unique(rows):
 values={x['stamp']:x for x in rows};return [values[k] for k in sorted(values)]
gt=unique(raw['truth']);lio=unique(raw['lio'])
if len(gt)<20 or len(lio)<10:
 a.output.write_text(json.dumps({'passed':False,'reason':'Insufficient independent odometry','truth':len(gt),'lio':len(lio)},indent=2)+'\n');raise SystemExit(1)
tg=np.array([x['stamp'] for x in gt]);pg=np.array([x['xyz'] for x in gt]);qg=Rotation.from_quat(np.array([x['xyzw'] for x in gt]));lio=[x for x in lio if tg[0]<=x['stamp']<=tg[-1]];tl=np.array([x['stamp'] for x in lio]);pl=np.array([x['xyz'] for x in lio]);ql=Rotation.from_quat(np.array([x['xyzw'] for x in lio]));truth=np.column_stack([np.interp(tl,tg,pg[:,i]) for i in range(3)]);truth_rot=Slerp(tg,qg)(tl)
align=truth_rot[0]*ql[0].inv();translation=truth[0]-align.apply(pl[0]);aligned=align.apply(pl)+translation;aligned_rot=align*ql;error=aligned-truth;xy=np.linalg.norm(error[:,:2],axis=1);z=abs(error[:,2]);angle=(truth_rot.inv()*aligned_rot).magnitude()
motion=json.loads(a.motion.read_text());start=motion.get('start',{}).get('sim');end=motion.get('forward_end',{}).get('sim');moving=(tl>=start)&(tl<=end) if start is not None and end is not None else np.zeros(len(tl),dtype=bool)
def stats(values):return {'mean':float(np.mean(values)),'p95':float(np.quantile(values,.95)),'max':float(np.max(values)),'final':float(values[-1])}
metrics={'xy_error_m':stats(xy),'z_error_m':stats(z),'rotation_error_rad':stats(angle),'moving_xy_error_m':stats(xy[moving]) if moving.any() else None}
checks={'valid_moving_output':int(moving.sum())>=15,'short_run_xy_tracking':float(np.quantile(xy,.95))<.15 and xy.max()<.3,'short_run_rotation_tracking':float(np.quantile(angle,.95))<.15,'vertical_stability':float(np.quantile(z,.95))<.15}
checks={k:bool(v) for k,v in checks.items()}
result={'scope':'isolated 20ms capture-time approximation; short straight segment, NOT high-precision or production acceptance','alignment':'single first pose SE3, no scale fit or full-trajectory fitting','alignment_sim_time':float(tl[0]),'evaluated_messages':len(tl),'moving_messages':int(moving.sum()),'sim_range_s':[float(tl[0]),float(tl[-1])],'metrics':metrics,'engineering_checks':checks,'engineering_tracking_passed':all(checks.values()),'centimeter_precision_info':{'xy_p95_below_5cm':float(np.quantile(xy,.95))<.05,'rotation_p95_below_0_05rad':float(np.quantile(angle,.95))<.05},'continuous_per_ray_timing_validated':False,'production_odom_switch_allowed':False,'limitations':['One short straight trial; no turn, loop closure, long-duration drift or independent restart acceptance.','20ms frame capture time is an explicit approximation; native per-ray timing not validated.','Noise parameters are upstream defaults, not calibrated IMU covariance.']}
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if a.plot:
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(1,3,figsize=(14,4));axes[0].plot(truth[:,0],truth[:,1],label='simulation truth',color='#1976d2');axes[0].plot(aligned[:,0],aligned[:,1],label='FAST-LIO2 aligned',color='#ef6c00');axes[0].set(xlabel='world x (m)',ylabel='world y (m)',title='First-pose alignment only');axes[0].axis('equal');axes[0].legend();axes[1].plot(tl,xy*100,color='#ef6c00');axes[1].set(xlabel='simulation time (s)',ylabel='XY error (cm)',title='20 ms capture-time trial');axes[2].plot(tl,np.rad2deg(angle),color='#388e3c');axes[2].set(xlabel='simulation time (s)',ylabel='rotation error (deg)',title='Full orientation error')
 for ax in axes:
  ax.grid(alpha=.25)
  if ax!=axes[0] and start is not None:ax.axvspan(start,end,alpha=.12,color='#1976d2')
 fig.tight_layout();a.plot.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.plot,dpi=150);plt.close(fig)
