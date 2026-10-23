"""Keep warehouse raster fixed; place repeat-route endpoints in open turn areas."""
import argparse
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from build_warehouse_route import dijkstra

p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(exist_ok=False)
for n in ['map.yaml','map.pgm']:shutil.copy2(a.source/n,a.output/n)
meta=yaml.safe_load((a.source/'map.yaml').read_text());origin=np.array(meta['origin'][:2]);res=float(meta['resolution'])
image=np.flipud(np.asarray(Image.open(a.source/'map.pgm')))
distance=distance_transform_edt(image==254)*res
free=(distance>=.85)[::2,::2]
def cell(xy):
    v=np.rint(((np.array(xy)-origin)/res-.5)/2).astype(int);return int(v[1]),int(v[0])
current=cell([2.,-1.]);total=0.;goals=[]
for xy in [[-4.5,9.],[3.5,-8.],[2.,-1.]]*2:
    target=cell(xy)
    if not free[target]:raise ValueError('Endpoint is not free')
    d,_=dijkstra(free,current,res*2)
    if not np.isfinite(d[target]):raise ValueError('Endpoint disconnected')
    total+=float(d[target]);goals.append(dict(xy=xy,cad_center_clearance_m=float(distance[target[0]*2,target[1]*2])))
    current=target
(a.output/'route.json').write_text(json.dumps(dict(goals=goals,planned_route_m=total,map_occupancy_unchanged=True,
 reason='previous top corner reached, then native turn made no progress; use open turn areas, no protection relaxation',
 scope='proposal, actual path and completion require recorded navigation'),indent=2)+'\n')
print(total)
