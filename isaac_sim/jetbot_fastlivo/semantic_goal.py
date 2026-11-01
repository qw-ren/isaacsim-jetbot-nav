"""Resolve a declared place to an existing immutable Nav2 mission request. No perception/LLM."""
import argparse,json,hashlib,subprocess,sys
from pathlib import Path
from extension_core import resolve_place
p=argparse.ArgumentParser();p.add_argument('--catalog',type=Path,required=True);p.add_argument('--place',required=True);p.add_argument('--case',type=Path);p.add_argument('--id');p.add_argument('--timeout',type=float,default=240);a=p.parse_args()
catalog=json.loads(a.catalog.read_text());assert catalog['frame']=='map';place=resolve_place(catalog,a.place)
proof=dict(place_id=place['id'],name=place['name'],goal=place['approach_pose_map'],catalog_sha256=hashlib.sha256(a.catalog.read_bytes()).hexdigest(),source='manual CAD semantic annotation, not visual recognition',truth_input=False)
if a.case:
 assert a.id,'Mission id is required for submission'
 x,y,yaw=proof['goal'];subprocess.run([sys.executable,str(Path(__file__).with_name('submit_goal.py')),'--case',str(a.case),'--id',a.id,'--x',str(x),'--y',str(y),'--yaw',str(yaw),'--timeout',str(a.timeout)],check=True)
 # Immutable audit sidecar; submit_goal enforces owned case, unique id, idle session.
 target=a.case/'mission_requests'/(a.id+'.semantic');target.write_text(json.dumps(proof,indent=2)+'\n')
print(json.dumps(proof,ensure_ascii=False,indent=2))
