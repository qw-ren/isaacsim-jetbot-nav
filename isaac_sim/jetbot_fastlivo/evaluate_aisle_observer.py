"""Closed-case offline verification of actual aisle traversal and observer limits."""
import argparse,json,math,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();r=a.case
assert json.loads((r/'RUNNER_RESULT.json').read_text())['exit_code']==0
assert json.loads((r/'NAVIGATION_RESULT.json').read_text())['passed']
out=r/'AISLE_OBSERVER_AUDIT.json';assert not out.exists()
def rows(name):return [json.loads(s)for s in(r/name).read_text().splitlines()if s.strip()]
t=rows('truth_trajectory.jsonl');o=rows('observer_pose.jsonl');segments={}
for label,x in [('northbound',-8.),('southbound',-13.)]:
 length=0.;samples=0
 for before,after in zip(t,t[1:]):
  b,c=before['xyz'],after['xyz']
  if abs(b[0]-x)<1.8 and abs(c[0]-x)<1.8 and 9.2<b[1]<24.5 and 9.2<c[1]<24.5:
   length+=math.dist(b[:2],c[:2]);samples+=1
 segments[label]={'actual_distance_in_existing_rack_aisle_m':length,'samples':samples}
rates=[abs(b['observer_yaw']-a['observer_yaw'])/(b['sim']-a['sim'])for a,b in zip(o,o[1:])if b['sim']>a['sim']]
events=rows('obstacle_events.jsonl')
# Camera-line geometry checked against the vehicle at each recorded timestamp.
from observer_follow import intersects
from bisect import bisect_left
static=json.loads((r/'map/collision_boxes.json').read_text());times=[v['stamp']for v in t];blocked=0
for q in o:
 i=min(bisect_left(times,q['sim']),len(t)-1);vehicle=t[i]['xyz'];anchor=[vehicle[0],vehicle[1],.15]
 boxes=static+[dict(min=[v['center'][j]-v['size'][j]/2 for j in range(3)],max=[v['center'][j]+v['size'][j]/2 for j in range(3)])for e in events if e['sim']<=q['sim']for v in e['actual_parts']]
 blocked+=any(intersects(anchor,q['eye'],box,0.)for box in boxes)

checks={'north_aisle_traversed':segments['northbound']['actual_distance_in_existing_rack_aisle_m']>12.,'adjacent_aisle_traversed':segments['southbound']['actual_distance_in_existing_rack_aisle_m']>6.,'both_obstacles_inside_aisle':len(events)==2 and all(all(abs(p['center'][0]+8)<1.2 and 14<p['center'][1]<24 for p in e['actual_parts'])for e in events),'fixed_eye_and_target_height':all(abs(q['eye'][2]-2.6)<1e-8 and abs(q['target'][2]-.35)<1e-8 for q in o),'observer_yaw_rate_limited':max(rates)<=.40001}
checks['recorded_camera_sightline_clear']=blocked==0
report={'blocked_camera_samples':blocked,'minimum_observer_distance_m':min(v.get('observer_distance',2.8)for v in o),'scope':'offline reference used to verify traversal only, never navigation input; numerical camera checks do not replace visual review','passed':all(checks.values()),'checks':checks,'aisles':segments,'max_observer_yaw_rate_rad_s':max(rates),'inputs':{n:hashlib.sha256((r/n).read_bytes()).hexdigest()for n in('truth_trajectory.jsonl','observer_pose.jsonl','obstacle_events.jsonl')},'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
out.write_text(json.dumps(report,indent=2)+'\n');(r/'AISLE_OBSERVER_AUDIT_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(report,indent=2))
