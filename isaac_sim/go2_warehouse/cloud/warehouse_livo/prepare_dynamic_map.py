"""Prepare declared prior geometry and A/B/C topology before dynamic execution."""
import argparse
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from PIL import Image
import yaml
from scipy.ndimage import distance_transform_edt

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from build_warehouse_route import dijkstra

parser=argparse.ArgumentParser()
parser.add_argument('--kind',choices=['A','B','C'],required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--observable-closure',action='store_true',help='C only: declared U-shaped prior with sensor-visible closing wall')
parser.add_argument('--source-map',type=Path,required=True)
parser.add_argument('--source-boxes',type=Path,required=True)
args=parser.parse_args()
args.output.mkdir(exist_ok=False)
prior=[dict(center=[4.3,-1.3,.5],size=[2.,.4,1.])] if args.kind=='B' else []
if args.kind=='A':obstacles=[dict(center=[4.3,0.,.5],size=[.8,.8,1.])]
elif args.kind=='B':obstacles=[dict(center=[4.3,1.4,.5],size=[.6,5.,1.])]
else:obstacles=[dict(center=[4.3,1.,.5],size=[.6,43.,1.])]
if args.observable_closure:
    if args.kind!='C':raise ValueError('Observable closure is C only')
    prior=[dict(center=[2.,y,.5],size=[5.,.4,1.]) for y in [-2.,2.]]
    prior.append(dict(center=[-.5,0.,.5],size=[.4,4.4,1.]))
    obstacles=[dict(center=[4.3,0.,.5],size=[.6,4.4,1.])]
config=dict(kind=args.kind,prior=prior,obstacles=obstacles,start_xy=[1.,0.],goal_xy=[6.,0.],
 description='controlled mid-motion local bypass / alternate passage / full cross-warehouse closure',
 expected_after_connected=args.kind!='C',observable_closure=args.observable_closure)
meta=yaml.safe_load(args.source_map.read_text());origin=np.array(meta['origin'][:2]);resolution=float(meta['resolution'])
pixels=np.flipud(np.asarray(Image.open(args.source_map.parent/meta['image']))).copy()
def overlay(image,boxes):
    for b in boxes:
        c,s=np.array(b['center'][:2]),np.array(b['size'][:2])
        lo=np.maximum(np.floor((c-s/2-origin)/resolution).astype(int),0)
        hi=np.minimum(np.ceil((c+s/2-origin)/resolution).astype(int)+1,np.array(image.shape[::-1]))
        image[lo[1]:hi[1],lo[0]:hi[0]]=0
overlay(pixels,prior)
Image.fromarray(np.flipud(pixels)).save(args.output/'map.pgm')
meta['image']='map.pgm';(args.output/'map.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
after=pixels.copy();overlay(after,obstacles)
def plan(image):
    free=(distance_transform_edt(image==254)*resolution>=.6)[::2,::2]
    def cell(xy):
        i=np.rint(((np.asarray(xy)-origin)/resolution-.5)/2).astype(int);return (int(i[1]),int(i[0]))
    a,b=cell(config['start_xy']),cell(config['goal_xy'])
    if not free[a] or not free[b]:raise ValueError('Endpoint occupied: invalid topology test')
    distances,previous=dijkstra(free,a,resolution*2)
    if not np.isfinite(distances[b]):return dict(connected=False,goal_free=True,path=[])
    path=[b]
    while path[-1]!=a:path.append(previous[path[-1]])
    path.reverse()
    points=[(origin+(np.array(v[::-1])*2+.5)*resolution).tolist() for v in path]
    return dict(connected=True,goal_free=True,path=points,length_m=float(distances[b]))
topology=dict(before=plan(pixels),after=plan(after),basis='offline CAD proposal, not native Nav2 or execution evidence')
if not topology['before']['connected'] or topology['after']['connected']!=config['expected_after_connected']:raise ValueError('Wrong proposed topology')
boxes=json.loads(args.source_boxes.read_text())
for b in prior:
    c,s=np.array(b['center']),np.array(b['size']);boxes.append(dict(min=(c-s/2).tolist(),max=(c+s/2).tolist(),source='declared fixed prior controlled test geometry'))
(args.output/'safety_boxes.json').write_text(json.dumps(boxes)+'\n')
(args.output/'dynamic_config.json').write_text(json.dumps(config,indent=2)+'\n')
(args.output/'topology_proposal.json').write_text(json.dumps(topology,indent=2)+'\n')
print(json.dumps({k:{x:v for x,v in d.items() if x!='path'} for k,d in topology.items() if isinstance(d,dict)}))
