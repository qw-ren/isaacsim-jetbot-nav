from pathlib import Path
import json,numpy as np,hashlib,shutil
import argparse
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();c=a.case;out=a.output;out.mkdir(exist_ok=True,parents=True)
read=lambda n:[json.loads(x)for x in(c/n).read_text().splitlines()];truth=read('truth_trajectory.jsonl');t=np.array([x['stamp']for x in truth]);xy=np.array([x['xyz'][:2]for x in truth]);events=read('obstacle_events.jsonl');paths=[p for p in read('navigation_paths.jsonl')if p['topic']=='/plan'];nav=json.loads((c/'NAVIGATION_RESULT.json').read_text());gate=json.loads((c/'GATE_RESULT.json').read_text());ev=[]
for e in events:
 b=np.array([[p['min'][:2],p['max'][:2]]for p in e['actual_parts']]);mask=t>=e['sim'];dist=min(np.linalg.norm(np.maximum(np.maximum(b[:,0]-v,v-b[:,1]),0),axis=1).min()for v in xy[mask]);lo=b[:,0].min(axis=0);hi=b[:,1].max(axis=0);cross=xy[mask&(abs(xy[:,1]-(lo[1]+hi[1])/2)<.25)]
 def hit(p):
  q=np.array(p['xy']);return any(np.any((q[:,0]>=l[0])&(q[:,0]<=h[0])&(q[:,1]>=l[1]-.05)&(q[:,1]<=h[1]+.05))for l,h in b)
 checks={'spawned_while_moving':e['gate']['output'][0]>.03,'authored_asset_colliders':all(p['authored_colliders']and not p['procedural_geometry']for p in e['actual_parts']),'pre_event_plan_intersected':any(hit(p)for p in paths if p['stamp']<e['sim']),'post_event_plan_avoids':any(not hit(p)for p in paths if p['stamp']>e['sim']+.5),'actual_path_passed_outside_asset':bool(len(cross)and np.any((cross[:,0]<lo[0]-.34)|(cross[:,0]>hi[0]+.34))),'positive_approximate_clearance':bool(dist>.34)}
 ev.append({'kind':e['kind'],'sim':e['sim'],'checks':checks,'approximate_circle_asset_clearance_m':float(dist-.34),'crossing_xy':cross.tolist()})
summary={'passed':nav['passed']and gate['fault']is None and gate['shutdown_zero_published']and all(all(e['checks'].values())for e in ev),'events':ev,'actual_xy_path_m':float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),'actual_endpoint_error_m':float(np.linalg.norm(xy[-1]-nav['goal'][:2])),'estimated_endpoint_error_m':nav['estimated_goal_error_m'],'truth_usage':'offline audit only','geometry':'conservative circle vs USD asset AABB; not a contact-force collision metric'}
(out/'SHOWCASE_AUDIT.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
for name in ['NAVIGATION_RESULT.json','RUNNER_RESULT.json','SENSOR_RESULT.json','INPUT_AUDIT.json','GATE_RESULT.json','CASE.json','obstacle_events.jsonl','navigation_paths.jsonl','truth_trajectory.jsonl','estimated_odom.jsonl','camera_contract.json']:
 shutil.copy2(c/name,out/name)
