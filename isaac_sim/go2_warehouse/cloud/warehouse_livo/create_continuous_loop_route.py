"""Declared full-warehouse loop proposal; runtime acceptance is separate."""
import argparse,json,shutil,sys
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from build_warehouse_route import dijkstra
p=argparse.ArgumentParser();p.add_argument('--map',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--profile',choices=['standard','fast-video'],default='standard');a=p.parse_args()
m=yaml.safe_load(a.map.read_text());o=np.array(m['origin'][:2]);r=m['resolution'];im=np.flipud(np.array(Image.open(a.map.parent/m['image'])));free=(distance_transform_edt(im==254)*r>=.6)[::2,::2]
goals=[[4.,-6.],[2.,-8.],[-2.,-8.],[-4.,-6.],[-4.,13.],[-2.,15.],[2.,15.],[2.,12.],[4.,10.],[4.,4.],[2.,0.],[-3.,-4.],[-5.,-2.],[-4.,0.]]
if a.profile=='fast-video':
 goals=[[4.,-6.],[2.,-8.],[-2.,-8.],[-4.,-6.],[-4.,13.],[-2.,15.],[2.,15.],[2.,12.],[4.,10.],[4.,-5.],[2.,-7.],[-2.,-7.],[-4.,-5.],[-4.,-1.3]]
def cell(xy):
 i=np.rint(((np.array(xy)-o)/r-.5)/2).astype(int);return int(i[1]),int(i[0])
length=0.;start=[1.,0.]
# Also validate the appended entrance and final goal for this profile.
for xy in ([[-2.,-1.3],[6.,0.]] if a.profile=='fast-video' else [[-2.,0.],[6.,0.]]):
 if not free[cell(xy)]:raise ValueError('Unsafe appended entrance/goal '+str(xy))
for goal in goals:
 if not free[cell(goal)]:raise ValueError('Occupied waypoint '+str(goal))
 d,_=dijkstra(free,cell(start),r*2)
 if not np.isfinite(d[cell(goal)]):raise ValueError('Disconnected waypoint')
 length+=float(d[cell(goal)]);start=goal
a.output.mkdir(exist_ok=False)
(a.output/'route.json').write_text(json.dumps(dict(goals=[dict(xy=g) for g in goals],planned_route_m=length,execution='native NavigateThroughPoses, RemovePassedGoals radius .7m, then continuous entry and dynamic goal in the same action',scope='CAD proposal only; no truth or actual acceptance'),indent=2)+'\n')
print(length)
