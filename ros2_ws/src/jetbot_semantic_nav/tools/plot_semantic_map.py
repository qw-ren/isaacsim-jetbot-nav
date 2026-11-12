"""Offline registered-object/route map, never an online localization correction."""
import argparse
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle
from PIL import Image
import yaml

p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
assert not a.output.exists();meta=yaml.safe_load((a.case/'map/map.yaml').read_text())
image=Image.open(a.case/'map'/meta['image']);ox,oy=meta['origin'][:2];res=meta['resolution']
objects=json.loads((a.case/'SEMANTIC_OBJECTS.json').read_text())['objects']
truth=[json.loads(s)for s in(a.case/'truth_trajectory.jsonl').read_text().splitlines()]
fig,ax=plt.subplots(figsize=(9,10));ax.imshow(image,cmap='gray',origin='upper',vmin=0,vmax=255,
    extent=(ox,ox+image.width*res,oy,oy+image.height*res),alpha=.65)
colors=dict(cart='#e69f00',pallet='#009e73',box='#cc79a7',shelf='#56b4e9',sign='#d55e00')
for row in objects:
    x,y=row['position'][:2];hx,hy=[v/2 for v in row['dimensions'][:2]];q=row['orientation_xyzw']
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2));c,s=math.cos(yaw),math.sin(yaw)
    points=[[x+c*dx-s*dy,y+s*dx+c*dy]for dx,dy in[(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]
    ax.add_patch(Polygon(points,color=colors.get(row['class_name'],'#777777'),alpha=.6))
    ax.text(x+.15,y+.15,row['object_id'].split('/')[-1],fontsize=8)
ax.plot([r['xyz'][0]for r in truth],[r['xyz'][1]for r in truth],color='#0072b2',lw=2,label='Executed truth XY (offline only)')
for path in sorted((a.case/'semantic_tasks').glob('*.json')):
    task=json.loads(path.read_text())
    if task['state']!='SUCCEEDED':continue
    x,y,yaw=task['goal'];ax.add_patch(Circle((x,y),.4,fill=False,lw=1.3,color='#d55e00'))
    ax.arrow(x,y,.55*math.cos(yaw),.55*math.sin(yaw),color='#d55e00',head_width=.16,length_includes_head=True)
    ax.text(x+.2,y-.35,task['request']['task_id'],fontsize=8,color='#a33700')
ax.set(xlim=(-17,3),ylim=(3,27),xlabel='map X (m)',ylabel='map Y (m)',
       title='USD oracle object layer and executed semantic missions\nStatic planning map unchanged; robot uses FAST-LIVO2 + AMCL')
ax.set_aspect('equal');ax.grid(alpha=.18);ax.legend(loc='upper right',fontsize=8)
fig.tight_layout();fig.savefig(a.output,dpi=180);plt.close(fig)
