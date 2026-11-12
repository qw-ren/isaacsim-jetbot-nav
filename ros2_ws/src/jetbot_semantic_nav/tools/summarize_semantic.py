"""Offline task/process/geometry audit; truth is read here for scoring only."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
a=p.parse_args();assert not a.output.exists();sys.path.insert(0,str(a.case/'source'))
from jetbot_semantic_nav.core import distance_to_object

def read(name):return json.loads((a.case/name).read_text())
def rows(name):return[json.loads(s)for s in(a.case/name).read_text().splitlines()if s.strip()]
truth=rows('truth_trajectory.jsonl');events=rows('semantic_tasks.jsonl');gate=rows('gate_states.jsonl')
tasks=[]
for path in sorted((a.case/'semantic_tasks').glob('*.json')):
    r=json.loads(path.read_text());task=r['request']['task_id'];end=r['finished_stamp']
    # Nearest recorded simulation stamp, no time shift/trajectory fit.
    sample=min(truth,key=lambda t:abs(t['stamp']-end));w,x,y,z=sample['wxyz']
    actual_yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    row=dict(task_id=task,state=r['state'],native_status=r.get('native_status'),
        disarmed=r.get('disarmed'),goal=r.get('goal'),result_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        finished_sim_s=end,truth_sample_delta_s=sample['stamp']-end)
    if r['state']=='SUCCEEDED':
        g=r['goal'];row['true_goal_error_m']=math.dist(sample['xyz'][:2],g[:2])
        row['true_goal_yaw_error_deg']=math.degrees(abs(math.atan2(math.sin(actual_yaw-g[2]),math.cos(actual_yaw-g[2]))))
        row['true_center_to_object_surface_m']=distance_to_object(sample['xyz'][:2],r['selected_object'])
    tasks.append(row)
objects=read('SEMANTIC_OBJECTS.json');case=read('CASE.json');provenance=read('SEMANTIC_ORACLE_PROVENANCE.json')
code={str(p.relative_to(a.case/'source')):hashlib.sha256(p.read_bytes()).hexdigest()
      for p in(a.case/'source').rglob('*')if p.is_file()and '__pycache__'not in p.parts}
report=dict(case=a.case.name,simulation_only=True,tasks=tasks,source_sha256=code,
    mode=case['navigation']['semantic']['mode'],semantic_parameters=case['navigation']['semantic'],
    object_count=len(objects['objects']),observation_batches=objects['observation_batches'],
    rejected_observation_batches=objects['rejected_batches'],gate=read('GATE_RESULT.json')if(a.case/'GATE_RESULT.json').exists()else None,
    observed_gate_faults=sorted({r['fault']for r in gate if r.get('fault')}),
    runner=read('RUNNER_RESULT.json')if(a.case/'RUNNER_RESULT.json').exists()else None,
    planning_map_unchanged=hashlib.sha256((a.case/'map/map.pgm').read_bytes()).hexdigest()==case['navigation']['semantic']['world_to_map']['planning_map_sha256'],
    prepared_scene_unchanged=hashlib.sha256((a.case/'jetbot_resolved.usda').read_bytes()).hexdigest()==case['navigation']['semantic']['world_to_map']['source_scene_sha256'],
    registration=provenance['registration'],offline_truth_scoring_only=True,
    truth_metric_definition='True robot chassis XY/yaw vs requested free-space approach goal at recorded completion time; not distance to object center or full-trajectory AMCL accuracy')
with a.output.open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2);f.write('\n')
print(json.dumps(dict(case=a.case.name,tasks=tasks),ensure_ascii=False,indent=2))
