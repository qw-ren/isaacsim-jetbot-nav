"""Plot per-trial localization and arrival as distinct quantities.
Run here with system Python (numpy/matplotlib); uses checked-in summaries only.
Different configurations/routes and individual trials are not pooled statistics.
"""
import csv
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root=Path(__file__).resolve().parent
labels={78:'78 original',79:'79 beams360',80:'80 beams360',82:'82 mesh',83:'83 mesh',
        88:'88 final BA',89:'89 final route',91:'91 final BA',92:'92 final route'}
rows=[]
for n,label in labels.items():
    p=root/f'VALIDATION{n}.json'
    if not p.exists():continue
    x=json.loads(p.read_text())
    if not x['navigation_succeeded']:continue
    amcl=x['localization']['amcl_published_pose_samples'];nav=x['localization']['continuous_navigation_global_pose'];end=x['endpoint']
    rows.append(dict(case=x['case'],label=label,amcl_xy_rmse_cm=100*amcl['xy_rmse_m'],
        amcl_xy_p95_cm=100*amcl['xy_p95_m'],amcl_yaw_rmse_deg=amcl['yaw_rmse_deg'],
        nav_xy_rmse_cm=100*nav['xy_rmse_m'],nav_distance_weighted_xy_rmse_cm=100*nav['distance_weighted_xy_rmse_m'],
        true_stopped_xy_cm=100*end['final_actual_xy_m'],true_stopped_yaw_deg=end['final_actual_yaw_deg'],
        estimated_feedback_xy_cm=100*end['estimated_xy_at_feedback_m']))
with (root/'ACCURACY_TRIALS.csv').open('w',newline='')as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]),lineterminator='\n');w.writeheader();w.writerows(rows)
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(2,1,figsize=(11.5,7.5),sharex=True)
t=np.arange(len(rows));width=.35
for ax,fields,colors,title in [
    (axes[0],['amcl_xy_rmse_cm','amcl_xy_p95_cm'],['#24779b','#83bed2'],'Global localization: recorded AMCL pose samples (scan timestamps)'),
    (axes[1],['true_stopped_xy_cm','estimated_feedback_xy_cm'],['#bd633b','#dfb08e'],'Arrival: true stopped error and estimated feedback residual')]:
    names=['XY RMSE','XY P95']if ax is axes[0]else['True stopped error','Estimated feedback residual']
    for off,field,color,name in zip([-.5,.5],fields,colors,names):
        y=[r[field]for r in rows];ax.bar(t+off*width,y,width,color=color,label=name)
        for x,v in zip(t+off*width,y):ax.text(x,v+.16,f'{v:.2f}',ha='center',va='bottom',fontsize=8)
    ax.set_ylabel('cm');ax.set_title(title,loc='left',fontsize=12);ax.legend(loc='upper right',ncol=2)
    ax.set_ylim(0,max(r[f]for r in rows for f in fields)*1.23);ax.grid(axis='y',alpha=.16);ax.set_axisbelow(True)
axes[0].axhline(3,color='#777777',linestyle=':',linewidth=1)
axes[0].text(.02,.93,'Dotted line: 3cm reference; metric definition still to be agreed',transform=axes[0].transAxes,fontsize=8,color='#555555')
axes[1].set_xticks(t,[r['label']for r in rows],rotation=25,ha='right')
fig.suptitle('JetBot 3x warehouse navigation — localization is distinct from arrival',fontsize=14,x=.06,ha='left')
fig.text(.06,.018,'Each bar is one run; configurations/routes differ. Frozen profiles use 5cm / 0.05rad goal tolerance. No trajectory fitting.',fontsize=9,color='#555555')
fig.tight_layout(rect=[0,.06,1,.95])
for suffix in ['png','pdf']:fig.savefig(root/f'ACCURACY_COMPARISON.{suffix}',dpi=160)
(root/'PLOT_PROVENANCE.json').write_text(json.dumps(dict(inputs_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in root.glob('VALIDATION*.json')},generator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),note=__doc__),indent=2)+'\n')
