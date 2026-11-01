"""Offline reference summary; never imported by navigation or actuator code."""
import argparse,json,math,hashlib
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);a=p.parse_args();assert(a.case/'RUNNER_RESULT.json').exists(), 'Completed cases only'
def rows(name):
 f=a.case/name
 return [json.loads(line)for line in f.read_text().splitlines() if line.strip()] if f.exists()else []
truth=rows('truth_trajectory.jsonl');assert truth
xy=np.array([r['xyz'][:2] for r in truth]);times=np.array([r['stamp']for r in truth]);cfg=json.loads((a.case/'CASE.json').read_text());goal=cfg.get('navigation',{}).get('goals',[[1.5,6,0]])[-1];events=rows('obstacle_events.jsonl');boxes=json.loads((a.case/'map/collision_boxes.json').read_text());bounds=np.array([[b['min'][:2],b['max'][:2]] for b in boxes])
def clearance(points,bs):
 return float(min(np.linalg.norm(np.maximum(np.maximum(bs[:,0]-v,v-bs[:,1]),0),axis=1).min() for v in points)) if len(points)else None
summary=dict(scope='offline simulator reference, never navigation input; circular footprint approximation',actual_xy_path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),initial_xyz=truth[0]['xyz'],final_xyz=truth[-1]['xyz'],actual_goal_distance_m=float(np.linalg.norm(xy[-1]-goal[:2])),source_samples=len(truth),static_center_clearance_m=clearance(xy,bounds),events=[])
radius=float(cfg.get('motion_limits',{}).get('footprint_radius_m',.16));summary['footprint_radius_m']=radius;summary['approximate_static_footprint_clearance_m']=summary['static_center_clearance_m']-radius
for e in events:
 bs=np.array([[np.array(p['center'][:2])-np.array(p['size'][:2])/2,np.array(p['center'][:2])+np.array(p['size'][:2])/2]for p in e['actual_parts']]);points=xy[times>=e['sim']];minimum=clearance(points,bs);summary['events'].append(dict(kind=e['kind'],sim=e['sim'],trigger_estimated_xy=e['trigger_estimated_pose']['xy'],minimum_reference_center_clearance_m=minimum,approximate_footprint_clearance_m=minimum-radius))
if(a.case/'NAVIGATION_RESULT.json').exists():nav=json.loads((a.case/'NAVIGATION_RESULT.json').read_text())
else:
 operator=json.loads((a.case/'OPERATOR_RESULT.json').read_text());assert operator['missions'];nav=operator['missions'][-1];goal=nav['goal'];summary['actual_goal_distance_m']=float(np.linalg.norm(xy[-1]-goal[:2]));summary['scope']+='; operator whole-session path and last completed mission endpoint';summary['final_mission_id']=nav['id'];summary['all_session_missions_passed']=all(v['passed']for v in operator['missions'])and operator['error']is None
summary['native_status']=nav.get('native_status');summary['navigation_passed']=nav['passed'];summary['obstacles_requested']=len(cfg.get('navigation',{}).get('obstacles',[]));summary['obstacles_inserted']=len(events)
summary['source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();out=a.case/'OFFLINE_REFERENCE.json';assert not out.exists();out.write_text(json.dumps(summary,indent=2)+'\n');out.with_name('OFFLINE_REFERENCE_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(summary,indent=2))
