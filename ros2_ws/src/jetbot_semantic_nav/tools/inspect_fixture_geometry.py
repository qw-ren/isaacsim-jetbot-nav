"""Read-only USD asset/scene audit in an anonymous layer; no simulation/ROS."""
import argparse
import json
import math
import sys
from pathlib import Path
from pxr import Usd,UsdGeom

p=argparse.ArgumentParser();p.add_argument('--config',required=True,type=Path);p.add_argument('--runtime',required=True,type=Path)
p.add_argument('--output',required=True,type=Path);a=p.parse_args();sys.path.insert(0,str(a.runtime))
from asset_fixture import place_asset
stage=Usd.Stage.CreateInMemory();config=json.loads(a.config.read_text())
boxes=json.loads(Path('/home/admin/jetbot-warehouse/map_prior_20261023_01/collision_boxes.json').read_text())
rows=[]
for fixture in config['semantic']['fixtures']:
    part=place_asset(stage,'/World/'+fixture['id'],fixture['asset_kind'],fixture['center'],fixture.get('yaw_rad',0.))
    distances=[]
    for b in boxes:
        distances.append(math.hypot(*[max(b['min'][i]-part['max'][i],part['min'][i]-b['max'][i],0.)for i in(0,1)]))
    rows.append(dict(fixture=fixture,geometry=part,min_static_aabb_gap_m=min(distances)))
with a.output.open('x')as f:json.dump(dict(parts=rows,scene_not_modified=True),f,indent=2)
print(json.dumps([dict(id=r['fixture']['id'],size=r['geometry']['size'],gap=r['min_static_aabb_gap_m'])for r in rows]))
