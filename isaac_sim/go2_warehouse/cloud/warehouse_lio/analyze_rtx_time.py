"""Offline render-time and motion-geometry audit; cannot authorize LIO input by itself."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation,Slerp
p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
all_rows=[json.loads(s) for s in (args.input/'frames.jsonl').read_text().splitlines()]
rows=[r for r in all_rows if r['timestamp_ns']>0 and r['points']>0]
excluded=[{'frame_id':r['frame_id'],'reason':'startup sentinel: timestamp or beam count zero'} for r in all_rows if r['timestamp_ns']<=0 or r['points']<=0]
if len(rows)<10:raise SystemExit('Insufficient unique frames')
sim=np.array([r['sim_time'] for r in rows]);native=np.array([r['timestamp_ns']/1e9 for r in rows]);state=np.array([r['root_state_w_xyz_wxyz_vel'] for r in rows]);q=state[:,3:7][:,[1,2,3,0]];rot=Rotation.from_quat(q)
expected=state[:,:3]+rot.apply(np.tile([.2,0,.2],(len(rows),1)))
end=np.array([r['transform'] for r in rows]).reshape(-1,4,4)
start=np.array([r['transform_start'] for r in rows]).reshape(-1,4,4)
end_rotation=Rotation.from_matrix(end[:,:3,:3].transpose(0,2,1))
pos_error=np.linalg.norm(end[:,3,:3]-expected,axis=1)
angle_error=(end_rotation.inv()*rot).magnitude()
# Native frame-start transform compared to preceding end, independently of absolute timestamp.
chain_pos=np.linalg.norm(start[1:,3,:3]-end[:-1,3,:3],axis=1)
chain_angle=(Rotation.from_matrix(start[1:,:3,:3].transpose(0,2,1)).inv()*end_rotation[:-1]).magnitude()
geometry=[]
for f in sorted(args.input.glob('frame_*.npz')):
 a=np.load(f);n=len(a['distances']);distance=a['distances'];valid=np.isfinite(distance)&(distance>.3)&(distance<30)
 if not valid.any() or 'scan_buffer_world' not in a:continue
 az=np.deg2rad(a['azimuths'][valid]);el=np.deg2rad(a['elevations'][valid]);d=distance[valid]
 xyz=np.c_[d*np.cos(el)*np.cos(az),d*np.cos(el)*np.sin(az),d*np.sin(el)]
 t0=a['transformStart'].reshape(4,4);t1=a['transform'].reshape(4,4)
 duration=.02;alpha=a['deltaTimes'][valid].astype(float)/1e9/duration
 if not np.all((alpha>=0)&(alpha<=1.001)):raise RuntimeError('Native point times exceed nominal render interval')
 rotations=Rotation.from_matrix(np.stack([t0[:3,:3].T,t1[:3,:3].T]));rr=Slerp([0,1],rotations)(np.clip(alpha,0,1))
 interpolated=rr.apply(xyz)+t0[3,:3]+alpha[:,None]*(t1[3,:3]-t0[3,:3])
 at_end=xyz@t1[:3,:3]+t1[3,:3]
 world=np.asarray(a['scan_buffer_world']).reshape(-1,3);world=world[np.isfinite(world).all(axis=1)&(np.linalg.norm(world,axis=1)>1e-5)]
 if not len(world):continue
 tree=cKDTree(world);di=tree.query(interpolated)[0];de=tree.query(at_end)[0]
 geometry.append({'file':f.name,'points':int(valid.sum()),'interpolated_nearest_world_p50_m':float(np.median(di)),'interpolated_nearest_world_p95_m':float(np.quantile(di,.95)),'frame_end_nearest_world_p50_m':float(np.median(de)),'frame_end_nearest_world_p95_m':float(np.quantile(de,.95))})
result={'startup_sentinels_excluded':excluded,'frames':len(rows),'sim_range_s':[float(sim.min()),float(sim.max())],'native_minus_sim_range_s':[float((native-sim).min()),float((native-sim).max())],'native_minus_sim_spread_s':float(np.ptp(native-sim)),
 'core_sim_vs_env_max_s':max(abs(r['core_sim_time']-r['sim_time']) for r in rows),'render_history_vs_env_max_s':max(abs(r.get('render_sim_time',r['sim_time'])-r['sim_time']) for r in rows),'history_query_failures':sum('history_query_error' in r for r in rows),
 'end_pose_position_error_max_m':float(pos_error.max()),'end_pose_rotation_error_max_rad':float(angle_error.max()),'start_vs_previous_end_position_max_m':float(chain_pos.max()),'start_vs_previous_end_rotation_max_rad':float(chain_angle.max()),
 'maximum_truth_speed_mps':float(np.linalg.norm(state[:,7:10],axis=1).max()),'cruise_frames_over_0_6_mps':int((np.linalg.norm(state[:,7:10],axis=1)>.6).sum()),'geometry':geometry,
 'checks':{'strict_frame_time':bool(np.all(np.diff(sim)>0)),'native_time_increment_matches_render':bool(np.max(np.abs(np.diff(native)-np.diff(sim)))<1e-6),'clock_matches_physics':all(abs(r['core_sim_time']-r['sim_time'])<1e-6 for r in rows),'render_history_available':all('history_query_error' not in r for r in rows),'lidar_end_pose_matches_truth':bool(pos_error.max()<1e-4 and angle_error.max()<1e-4),'frame_start_matches_previous_end':bool(chain_pos.max()<1e-4 and chain_angle.max()<1e-4)},
 'lio_input_approved':False,'limitations':['One run is insufficient: require independent cold-start mapping validation.','Nearest-world geometry is diagnostic; full-buffer nearest neighbors do not establish per-ray correspondence.','Frame timestamp is documented as start, while transform is end. Explicit point-stamp mapping still requires validated render phase.']}
args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='geometry'},indent=2))
