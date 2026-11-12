"""Offline sampled-score evidence; incomplete DWB totals are explicit lower bounds."""
import argparse
import hashlib
import json
import math
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--task',default='farthest_diag')
p.add_argument('--output',required=True,type=Path);a=p.parse_args();assert not a.output.exists()
task=json.loads((a.case/'semantic_tasks'/(a.task+'.json')).read_text());end=task['finished_stamp'];goal=task['goal'][:2]
source=a.case/'controller_evaluations.jsonl';candidates=[]
for line in source.read_text().splitlines():
    row=json.loads(line)
    if end-5<=row['stamp']<end-.2 and 0<=row['best_index']<len(row['candidates']):
        best=row['candidates'][row['best_index']]
        forward=[c for c in row['candidates']if c['velocity'][0]>1e-5 and c['total']>=0]
        if abs(best['velocity'][0])<1e-5 and forward:candidates.append((row,best,forward))
if not candidates:raise ValueError('No qualifying stalled-score sample')
row,best,forward=candidates[-1];minimum=min(c['total']for c in forward)
path_rows=[]
for line in(a.case/'semantic_plan_endpoints.jsonl').read_text().splitlines():
    r=json.loads(line)
    if r['topic']in('/plan','/received_global_plan')and row['stamp']-1<=r['stamp']<=row['stamp']+.2:
        path_rows.append(dict(topic=r['topic'],stamp=r['stamp'],endpoint=r['last']['xyz'],request_xy_delta_m=math.dist(r['last']['xyz'][:2],goal)))
report=dict(case=a.case.name,task=a.task,native_status=task.get('native_status'),state=task['state'],
    score_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),sample_sim_s=row['stamp'],
    winning_candidate=best,valid_forward_count=len(forward),
    valid_forward_total_lower_bound_min=minimum,all_forward_partial_totals_above_winner=minimum>best['total'],
    least_forward_candidates=sorted(forward,key=lambda c:c['total'])[:4],
    forward_invalid_count=sum(c['velocity'][0]>1e-5 and c['total']<0 for c in row['candidates']),
    nearby_map_path_endpoints=path_rows,
    limitations=['Single sampled instant; not a global causal attribution.',
        'Short-circuit scores are partial lower bounds, not fully scored alternatives.',
        'Independent object and scan geometry need not occupy identical grid cells.'],truth_input=False)
snapshots=list((a.case/'local_costmap_diagnostic').glob('*.npz'))
if snapshots:
    import numpy as np
    chosen=[]
    for path in snapshots:
        with np.load(path)as z:chosen.append((abs(float(z['stamp'])-row['stamp']),path))
    _,path=min(chosen);z=np.load(path);stamp=float(z['stamp'])
    transforms=[json.loads(s)for s in(a.case/'localization_probe.jsonl').read_text().splitlines()]
    transforms=[t for t in transforms if 'map->odom'in t['transforms']]
    t=min(transforms,key=lambda t:abs(t['stamp']-stamp));tr=t['transforms']['map->odom'];q=tr['xyzw']
    yaw=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2));dx=goal[0]-tr['xyz'][0];dy=goal[1]-tr['xyz'][1]
    xy=[math.cos(yaw)*dx+math.sin(yaw)*dy,-math.sin(yaw)*dx+math.cos(yaw)*dy]
    assert str(z['frame'])=='odom'
    q=z['origin_quaternion'];angle=math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    d=np.array(xy)-z['origin'];c,s=math.cos(angle),math.sin(angle);res=float(z['resolution'])
    ix=int(math.floor((c*d[0]+s*d[1])/res));iy=int(math.floor((-s*d[0]+c*d[1])/res))
    grid=z['data'];assert 0<=iy<grid.shape[0]and 0<=ix<grid.shape[1]
    report['nearby_local_goal_cell']=dict(snapshot=str(path.relative_to(a.case)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        stamp=stamp,tf_sample_delta_s=t['stamp']-stamp,frame='odom',goal_cell=[ix,iy],cost=int(grid[iy,ix]),resolution_m=res,
        estimated_tf_projection_only=True)
with a.output.open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
print(json.dumps({k:report[k]for k in('case','state','sample_sim_s','valid_forward_total_lower_bound_min','all_forward_partial_totals_above_winner')}))
