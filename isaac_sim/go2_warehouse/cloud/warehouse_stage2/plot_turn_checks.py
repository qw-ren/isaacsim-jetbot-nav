#!/usr/bin/env python3
"""Plot measured turning-navigation telemetry and conservative collision boxes."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
p=argparse.ArgumentParser();p.add_argument('--runs',nargs='+',type=Path,required=True);p.add_argument('--boxes',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
f,axes=plt.subplots(3,1,figsize=(10,10),gridspec_kw={'height_ratios':[1.2,1,1]})
for b in json.loads(a.boxes.read_text()):
 lo,hi=b['min'],b['max']
 if hi[0]>=-1 and lo[0]<=8 and hi[1]>=-3 and lo[1]<=3:axes[0].add_patch(Rectangle(lo[:2],hi[0]-lo[0],hi[1]-lo[1],color='grey',alpha=.15))
for d in a.runs:
 data=json.loads((d/'telemetry.json').read_text());r=json.loads((d/'navigation_check.json').read_text());o=data['odom'];t=np.array([x['sim']-r['start']['sim'] for x in o]);line,=axes[0].plot([x['x'] for x in o],[x['y'] for x in o],label=d.name);c=line.get_color();axes[0].scatter(r['goal']['x'],r['goal']['y'],marker='x',color=c)
 axes[1].plot(t,[x['vx'] for x in o],label=d.name,color=c);axes[2].plot(t,[x['wz'] for x in o],label=d.name,color=c)
 for ax in axes[1:]:
  if 'action_end' in r:ax.axvline(r['action_end']['sim']-r['start']['sim'],color=c,ls=':',alpha=.6)
axes[0].set(xlim=(-1,8),ylim=(-3,3),xlabel='Map x (m)',ylabel='Map y (m)',title='Actual turning navigation; shaded geometry is a USD development map');axes[0].set_aspect('equal',adjustable='box');axes[1].axhline(.6,ls='--',color='black',alpha=.5,label='Cruise requirement');axes[1].set(xlabel='Simulation time since goal start (s)',ylabel='Body forward velocity (m/s)');axes[2].set(xlabel='Simulation time since goal start (s)',ylabel='Yaw rate (rad/s)')
for ax in axes:ax.grid();ax.legend(fontsize=8)
f.tight_layout();f.savefig(a.output,dpi=160)
