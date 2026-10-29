"""Before simulation: put declared STATIC asset fixtures in both USD and CAD prior.
Dynamic obstacles are never painted into the online prior.
"""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from pxr import Usd
from asset_fixture import place_event
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();cfg=json.loads((a.case/'CASE.json').read_text());events=cfg.get('navigation',{}).get('static_fixtures',[])
overrides=cfg.get('navigation',{}).get('parameter_overrides',{})
if overrides:
 target=a.case/'source/nav2.yaml';before=hashlib.sha256(target.read_bytes()).hexdigest();params=yaml.safe_load(target.read_text())
 for path,value in overrides.items():
  keys=path.split('/');node=params
  for key in keys[:-1]:node=node[key]
  assert (keys[-1]in node or (keys[-1]=='required_movement_angle'and node.get('plugin')=='nav2_controller::PoseProgressChecker'))and isinstance(value,(str,int,float,bool)),path
  node[keys[-1]]=value
 target.write_text(yaml.safe_dump(params,sort_keys=False));(a.case/'PARAMETER_OVERRIDES.json').write_text(json.dumps(dict(overrides=overrides,before_sha256=before,after_sha256=hashlib.sha256(target.read_bytes()).hexdigest()),indent=2))
 old=a.case/'source_files.sha256';old.rename(a.case/'source_files_before_scenario.sha256');old.write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(a.case))+'\n'for f in sorted((a.case/'source').glob('*'))if f.is_file()))
if not events:raise SystemExit(0)
assert not(a.case/'STATIC_FIXTURES.json').exists();scene=a.case/'jetbot_resolved.usda';s=Usd.Stage.Open(str(scene));parts=[]
for i,e in enumerate(events):parts+=place_event(s,'/World/DeclaredStaticFixtures/event_'+str(i),e)
s.GetRootLayer().Save();meta=yaml.safe_load((a.case/'map/map.yaml').read_text());image=a.case/'map'/meta['image'];grid=np.array(Image.open(image).convert('L'));res=meta['resolution'];ox,oy=meta['origin'][:2];h,w=grid.shape
for part in parts:
 lo,hi=part['min'],part['max'];x0,x1=int(np.floor((lo[0]-ox)/res)),int(np.ceil((hi[0]-ox)/res));y0,y1=int(np.floor((lo[1]-oy)/res)),int(np.ceil((hi[1]-oy)/res));grid[max(0,h-y1):min(h,h-y0),max(0,x0):min(w,x1)]=0
Image.fromarray(grid).save(image);boxes=json.loads((a.case/'map/collision_boxes.json').read_text());boxes.extend(dict(path=p['path'],min=p['min'],max=p['max'])for p in parts);(a.case/'map/collision_boxes.json').write_text(json.dumps(boxes));(a.case/'STATIC_FIXTURES.json').write_text(json.dumps(dict(scope='declared fixed scenario geometry before startup; dynamic closure excluded from prior',parts=parts,map_sha256=hashlib.sha256(image.read_bytes()).hexdigest()),indent=2))
