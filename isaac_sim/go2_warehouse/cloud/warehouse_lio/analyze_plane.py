"""Independent static plane ray-range comparison; no ROS, no inferred per-ray labels."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
p=argparse.ArgumentParser();p.add_argument('--simulation',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
meta=json.loads((a.simulation/'calibration_plane.json').read_text());lo=np.asarray(meta['min']);hi=np.asarray(meta['max']);plane=float(lo[0]);root=a.simulation/'lio_sensors/time_audit'
rows=[json.loads(s) for s in (root/'frames.jsonl').read_text().splitlines()];frames={x['frame_id']:x for x in rows};out=[]
for f in sorted(root.glob('frame_*.npz')):
 with np.load(f) as raw:
  fid=int(raw['frameId']);row=frames[fid];d=raw['distances'];az=np.deg2rad(raw['azimuths']);el=np.deg2rad(raw['elevations']);u=np.c_[np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)]
  start=raw['transformStart'].reshape(4,4);end=raw['transform'].reshape(4,4);alpha=raw['deltaTimes'].astype(float)/20000000.
  if not len(d):continue
  if np.any((alpha<0)|(alpha>1.001)):raise RuntimeError('Point times outside 20ms render interval')
  rr=Rotation.from_matrix(np.stack([start[:3,:3].T,end[:3,:3].T]));ri=Slerp([0,1],rr)(np.clip(alpha,0,1))
  directions={'end':u@end[:3,:3],'start':u@start[:3,:3],'interpolated':ri.apply(u)}
  origins={'end':np.tile(end[3,:3],(len(d),1)),'start':np.tile(start[3,:3],(len(d),1)),'interpolated':start[3,:3]+alpha[:,None]*(end[3,:3]-start[3,:3])}
  predicted={};mask=np.isfinite(d)&(d>.3)&(d<30)
  for name in directions:
   dr=directions[name];origin=origins[name];den=dr[:,0];safe=np.where(abs(den)>1e-8,den,np.nan);distance=(plane-origin[:,0])/safe;hit=origin+distance[:,None]*dr;predicted[name]=distance
   mask &= (den>.6)&(distance>.3)&(hit[:,1]>lo[1]+.15)&(hit[:,1]<hi[1]-.15)&(hit[:,2]>lo[2]+.15)&(hit[:,2]<hi[2]-.15)
  # Broad 0.5m gate excludes occluding warehouse geometry; it is >20x model differences.
  mask &= abs(d-predicted['end'])<.5
  if mask.sum()<50:continue
  result={'frame_id':fid,'sim_time':row['sim_time'],'rays':int(mask.sum()),'truth_xy_speed_mps':float(np.linalg.norm(row['root_state_w_xyz_wxyz_vel'][7:9]))}
  residual={k:d[mask]-v[mask] for k,v in predicted.items()}
  for name,res in residual.items():result[name]={'bias_m':float(np.mean(res)),'mae_m':float(np.mean(abs(res))),'rmse_m':float(np.sqrt(np.mean(res*res))),'p95_abs_m':float(np.quantile(abs(res),.95))}
  shift=(predicted['interpolated']-predicted['end'])[mask];den=float(shift@shift);result['interpolated_vs_end_regression_beta']=float(shift@residual['end']/den) if den>1e-10 else None
  result['object_ids']=np.unique(raw['objectIds'][mask]).astype(int).tolist();out.append(result)
moving=[x for x in out if x['truth_xy_speed_mps']>.6];stationary=[x for x in out if x['truth_xy_speed_mps']<.03]
summary={}
for group,items in [('moving',moving),('stationary',stationary)]:
 summary[group]={'frames':len(items),'rays':sum(x['rays'] for x in items)}
 for model in ['end','start','interpolated']:
  summary[group][model+'_median_frame_rmse_m']=float(np.median([x[model]['rmse_m'] for x in items])) if items else None
  summary[group][model+'_median_frame_bias_m']=float(np.median([x[model]['bias_m'] for x in items])) if items else None
 beta=[x['interpolated_vs_end_regression_beta'] for x in items if x['interpolated_vs_end_regression_beta'] is not None];summary[group]['median_beta']=float(np.median(beta)) if beta else None
m=summary['moving'];checks={'target_front_plane_is_expected':abs(plane-6.45)<1e-6,'motion_coverage':m['frames']>=8 and m['rays']>=2000,'stationary_coverage':summary['stationary']['frames']>=3,'end_pose_absolute_range_accuracy':bool(m['frames'] and m['end_median_frame_rmse_m']<.01),'end_pose_beats_interpolated':bool(m['frames'] and m['end_median_frame_rmse_m']<.25*m['interpolated_median_frame_rmse_m']),'end_pose_beats_start':bool(m['frames'] and m['end_median_frame_rmse_m']<.25*m['start_median_frame_rmse_m']),'regression_supports_end_pose':bool(m['median_beta'] is not None and abs(m['median_beta'])<.15)}
result={'plane':meta,'selection':'ray intersects interior of analytic target under all three pose models; observed range within broad 0.5m end-model gate to reject occlusions','summary':summary,'checks':checks,'end_frame_sampling_supported':all(checks.values()),'frames':out,'limitations':['Single fixed-plane run: repeat cold-start before enabling a production point stream.','This sensor/render configuration only; not a claim about real Hesai timing.','deltaTimes remain native emission-profile metadata; use only timing consistent with measured geometry.']}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['frames','plane']},indent=2))
