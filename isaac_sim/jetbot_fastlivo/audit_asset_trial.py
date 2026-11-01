"""Completed case audit. Reference trajectory is offline evaluation only."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.ndimage import label
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();c=a.case;assert(c/'RUNNER_RESULT.json').exists()
def rows(name):
 f=c/name;return [json.loads(x)for x in f.read_text().splitlines()if x.strip()]if f.exists()else[]
cfg=json.loads((c/'CASE.json').read_text());nav=json.loads((c/'NAVIGATION_RESULT.json').read_text());gate=json.loads((c/'GATE_RESULT.json').read_text());events=rows('obstacle_events.jsonl');assert len(events)==1;e=events[0];kind=e['kind'];parts=e['actual_parts'];bounds=np.array([[p['min'][:2],p['max'][:2]]for p in parts]);truth=rows('truth_trajectory.jsonl');t=np.array([x['stamp']for x in truth]);xy=np.array([x['xyz'][:2]for x in truth]);goal=nav['goal'][:2];radius=cfg['motion_limits']['footprint_radius_m'];mask=t>=e['sim'];distance=min(float(np.linalg.norm(np.maximum(np.maximum(bounds[:,0]-v,v-bounds[:,1]),0),axis=1).min())for v in xy[mask]);paths=[v for v in rows('navigation_paths.jsonl')if v['topic']=='/plan'and len(v['xy'])>1]
def intersects(path):
 v=np.array(path['xy']);return bool(any(np.any((v[:,0]>=lo[0])&(v[:,0]<=hi[0])&(v[:,1]>=lo[1]-.05)&(v[:,1]<=hi[1]+.05))for lo,hi in bounds))
before=[v for v in paths if v['stamp']<e['sim']];after=[v for v in paths if v['stamp']>=e['sim']+.5];post=[v for v in after if not intersects(v)];snapshots=[]
for f in sorted((c/'costmap_snapshots').glob('*.npz'))if(c/'costmap_snapshots').exists()else[]:
 data=np.load(f)
 if float(data['stamp'])>=e['sim']+.3:snapshots.append((f,data))
f=c/'global_costmap_terminal.npz'
if f.exists():snapshots.append((f,np.load(f)))
if not snapshots:
 f=c/('global_costmap_'+kind+'.npz')
 if f.exists():snapshots=[(f,np.load(f))]
connection=None
if snapshots:
 f,data=max(snapshots,key=lambda v:float(v[1]['stamp']));g=data['data'];res=float(data['resolution']);origin=data['origin'];free=(g>=0)&(g<99);labels,n=label(free);position=xy[np.argmin(abs(t-float(data['stamp'])))];cell=lambda v:(int((v[1]-origin[1])/res),int((v[0]-origin[0])/res));si,gi=cell(position),cell(goal);connection=dict(snapshot=f.name,stamp=float(data['stamp']),start_cell=int(g[si]),goal_cell=int(g[gi]),connected=bool(labels[si]and labels[si]==labels[gi]))
# Actual progress crossing the barrier Y outside its blocked X interval proves a different aisle.
y=float(e['trigger_estimated_pose']['xy'][1]);low=bounds[:,0].min(axis=0);high=bounds[:,1].max(axis=0);cross=xy[mask & (abs(xy[:,1]-((low[1]+high[1])/2))<.25)];outside=bool(len(cross)and np.any((cross[:,0]<low[0]-radius)|(cross[:,0]>high[0]+radius)))
checks=dict(native_expected_status=nav.get('native_status')==(6 if kind=='C' else 4),no_gate_fault=gate['fault']is None,shutdown_zero=gate['shutdown_zero_published'],inserted_while_driving=e['gate']['output'][0]>.03,real_asset_collisions=all(p.get('authored_colliders')and not p.get('procedural_geometry',True)for p in parts),physical_clearance=distance-radius>0.,pre_event_plan_crossed_obstacle=any(intersects(v)for v in before))
if kind=='B':checks.update(post_event_plan_avoids_barrier=bool(post),actual_alternate_aisle_crossing=outside,goal_still_reachable=bool(connection and connection['connected']))
if kind=='C':checks.update(observed_global_disconnection=bool(connection and not connection['connected']),goal_not_occupied=bool(connection and 0<=connection['goal_cell']<99),native_not_timeout_or_rejected=nav.get('native_status')==6 and 'error'not in nav)
summary=dict(kind=kind,checks=checks,passed=all(checks.values()),actual_xy_path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),actual_endpoint_error_m=float(np.linalg.norm(xy[-1]-goal)),minimum_reference_center_clearance_m=distance,approximate_footprint_clearance_m=distance-radius,paths_before=len(before),paths_after=len(after),avoiding_paths_after=len(post),actual_barrier_crossings_xy=cross.tolist(),global_costmap_connectivity=connection,map_sha256=hashlib.sha256((c/'map/map.pgm').read_bytes()).hexdigest(),truth_input=False,reference_usage='offline audit only',geometry_scope='asset AABB and circular vehicle footprint approximation')
output=c/'ASSET_TRIAL_AUDIT.json';assert not output.exists();output.write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items()if k!='actual_barrier_crossings_xy'},indent=2))
