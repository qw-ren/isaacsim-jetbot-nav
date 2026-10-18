"""Recompute barrier lethal growth and render recorded native paths; no cloud/ROS access."""
from pathlib import Path
import json,gzip
import numpy as np,yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
r=Path(__file__).resolve().parent
# Support the compact review copy and the complete original artifact layout.
p=r/'plan_01/planning'
if not p.exists():p=r/'planning'
m=r/'maps/slam_b_prior_20261018_v2'
if not m.exists():m=r/'slam_b_prior_20261018_v2'
result=json.loads((p/'branch_plan_check.json').read_text());before=np.load(p/'before.npz');after=np.load(p/'blocked.npz')
meta=yaml.safe_load((m/'map_nav.yaml').read_text());gray=np.asarray(Image.open(m/'map.pgm'))[::-1];grid=np.where(gray>=250,0,np.where(gray<=100,100,-1));origin=meta['origin'];z=meta['resolution']
fig,ax=plt.subplots(figsize=(10,10));ax.imshow(grid,origin='lower',extent=[origin[0],origin[0]+grid.shape[1]*z,origin[1],origin[1]+grid.shape[0]*z],vmin=-1,vmax=100,cmap='gray_r')
for key,color in [('divider','royalblue'),('barrier','orangered')]:
    b=result['fixture'][key];c=np.array(b['center'][:2]);h=np.array(b['size'][:2])/2;corners=c+np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*h;corners=corners@after['map_to_odom_rotation'].T+after['map_to_odom_translation'];ax.add_patch(Polygon(corners,fill=False,edgecolor=color,lw=2,label='Prior divider' if key=='divider' else 'New physical barrier'))
for data,color,label in [(before,'deepskyblue','Before: upper branch (5.57 m)'),(after,'darkorange','Blocked: lower branch (9.13 m)')]:
    xy=data['path'];ax.plot(*xy.T,c=color,lw=2,label=label)
nav=r/'plan_01/navigation_01/telemetry.json'
if nav.exists() or nav.with_suffix('.json.gz').exists():
    telemetry=json.loads(nav.read_text() if nav.exists() else gzip.decompress(nav.with_suffix('.json.gz').read_bytes()));points=np.array([[x['x'],x['y']] for x in telemetry['map_poses']]);ax.plot(*points.T,c='green',lw=1.5,label='Actual localized trajectory')
ax.set(xlim=(-.5,8.5),ylim=(-4.,4.8),xlabel='map x (m)',ylabel='map y (m)',title='B: measured SLAM prior and recorded Navfn branch switch');ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper left');fig.tight_layout();fig.savefig(r/'branch_paths.png',dpi=150)
growth=result['blocked']['barrier_region_lethal_cells']-result['before']['barrier_region_lethal_cells']
supplement={'source':'recorded stationary JSON/NPZ; offline recheck, no rerun or rewriting original result','prior_region_lethal_cells':result['before']['barrier_region_lethal_cells'],'blocked_region_lethal_cells':result['blocked']['barrier_region_lethal_cells'],'lethal_cell_growth':growth,'note':'ROI overlaps the prior divider, so positive count alone is insufficient. Require growth as well.','checks':{'barrier_increases_lethal_cells':growth>=20,'before_and_after_native_success':result['before']['status']==result['blocked']['status']==4}}
supplement['passed']=all(supplement['checks'].values());(r/'branch_plan_supplement.json').write_text(json.dumps(supplement,indent=2)+'\n');print(json.dumps(supplement,indent=2))
