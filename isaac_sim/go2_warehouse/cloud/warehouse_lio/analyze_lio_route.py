"""Evaluate long/turning trial against truth, using one first-pose alignment only."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
p=argparse.ArgumentParser();p.add_argument('--observer',type=Path,required=True);p.add_argument('--route',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--plot',type=Path);a=p.parse_args();raw=json.loads((a.observer/'telemetry.json').read_text());route=json.loads(a.route.read_text())
def unique(rows):
 values={x['stamp']:x for x in rows};return [values[k] for k in sorted(values)]
def stats(v):return {'mean':float(np.mean(v)),'p95':float(np.quantile(v,.95)),'max':float(np.max(v)),'final':float(v[-1])} if len(v) else None
gt=unique(raw['truth']);lio=unique(raw['lio'])
if len(gt)<20 or len(lio)<10:raise SystemExit('Insufficient actual trajectory data')
tg=np.array([x['stamp'] for x in gt]);pg=np.array([x['xyz'] for x in gt]);rg=Rotation.from_quat([x['xyzw'] for x in gt]);lio=[x for x in lio if tg[0]<=x['stamp']<=tg[-1]];tl=np.array([x['stamp'] for x in lio]);pl=np.array([x['xyz'] for x in lio]);rl=Rotation.from_quat([x['xyzw'] for x in lio]);truth=np.column_stack([np.interp(tl,tg,pg[:,i]) for i in range(3)]);tr=Slerp(tg,rg)(tl)
align=tr[0]*rl[0].inv();translation=truth[0]-align.apply(pl[0]);est=align.apply(pl)+translation;erot=align*rl;delta=est-truth;xy=np.linalg.norm(delta[:,:2],axis=1);z=abs(delta[:,2]);rot=(tr.inv()*erot).magnitude()
def yaw(r):
 v=r.as_matrix();return np.arctan2(v[:,1,0],v[:,0,0])
yawerr=abs(np.arctan2(np.sin(yaw(erot)-yaw(tr)),np.cos(yaw(erot)-yaw(tr))));kinds=np.full(len(tl),'other',dtype=object)
for seg in route['segments']:kinds[(tl>=seg['begin_sim'])&(tl<=seg['end_sim'])]=seg['kind']
phase={name:{'messages':int((kinds==name).sum()),'xy_m':stats(xy[kinds==name]),'heading_rad':stats(yawerr[kinds==name])} for name in ['forward','turn','hold','curve']}
travel=float(np.linalg.norm(np.diff(truth[:,:2],axis=0),axis=1).sum());esttravel=float(np.linalg.norm(np.diff(est[:,:2],axis=0),axis=1).sum());rpe={}
for seconds in [1.,5.]:
 i=np.arange(len(tl));j=np.searchsorted(tl,tl+seconds);ok=j<len(tl);i=i[ok];j=j[ok]
 if len(i):
  dg=tr[i].inv().apply(truth[j]-truth[i]);de=erot[i].inv().apply(est[j]-est[i]);relative_rot=((tr[i].inv()*tr[j]).inv()*(erot[i].inv()*erot[j])).magnitude();rpe[str(seconds)]={'translation_m':stats(np.linalg.norm(de-dg,axis=1)),'rotation_rad':stats(relative_rot)}
checks={'route_complete':route['passed'],'moving_and_turning_coverage':(phase['forward']['messages']>=50 and phase['turn']['messages']>=50) or phase['curve']['messages']>=200,'short_route_xy_p95_lt_10cm':float(np.quantile(xy,.95))<.1,'short_route_xy_max_lt_20cm':float(xy.max())<.2,'heading_p95_lt_0_1rad':float(np.quantile(yawerr,.95))<.1,'vertical_p95_lt_15cm':float(np.quantile(z,.95))<.15}
result={'trajectory':route.get('trajectory','out/back'),'scope':'independent cold-start, original warehouse, 20ms capture-time approximation, isolated odometry; no production switch','reference':'/unitree_go2/odom is direct simulation root_state_w, NOT an old noisy estimator','accuracy_improvement_vs_previous_estimator_established':False,'reason':'Previous navigation odometry is simulation truth. This measures FAST-LIO2 error against that reference, not improvement over it.','alignment':'first pose only SE3, no trajectory/scale fitting','sim_range_s':[float(tl[0]),float(tl[-1])],'messages':len(tl),'truth_path_at_lio_times_m':travel,'lio_path_m':esttravel,'metrics':{'xy_m':stats(xy),'vertical_m':stats(z),'full_rotation_rad':stats(rot),'heading_rad':stats(yawerr)},'by_phase':phase,'relative_pose_error':rpe,'endpoint_xy_error_fraction_of_path':float(xy[-1]/travel) if travel else None,'checks':checks,'passed':all(checks.values()),'continuous_per_ray_timing_validated':False,'production_odom_switch_allowed':False}
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if a.plot:
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axs=plt.subplots(1,3,figsize=(14,4));axs[0].plot(truth[:,0],truth[:,1],label='simulation truth',color='#1976d2');axs[0].plot(est[:,0],est[:,1],label='FAST-LIO2 aligned',color='#ef6c00');axs[0].axis('equal');axs[0].set(xlabel='world x (m)',ylabel='world y (m)',title='Original warehouse '+route.get('trajectory','out/back'));axs[0].legend();axs[1].plot(tl,xy*100,color='#ef6c00');axs[1].set(xlabel='simulation time (s)',ylabel='XY error (cm)',title='First-pose alignment only');axs[2].plot(tl,np.rad2deg(yawerr),color='#388e3c');axs[2].set(xlabel='simulation time (s)',ylabel='heading error (deg)',title='20 ms capture-time approximation')
 for seg in route['segments']:
  if seg['kind'] in ['turn','curve']:
   for ax in axs[1:]:ax.axvspan(seg['begin_sim'],seg['end_sim'],alpha=.13,color='#1976d2')
 for ax in axs:ax.grid(alpha=.25)
 fig.tight_layout();fig.savefig(a.plot,dpi=150);plt.close(fig)
