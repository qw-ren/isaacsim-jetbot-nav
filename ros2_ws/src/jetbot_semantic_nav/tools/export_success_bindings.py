"""Export only recorded successful approach poses; does not infer unvisited goals."""
import argparse,json,hashlib
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
case=json.loads((a.case/'CASE.json').read_text());objects=[];seen=set()
for path in sorted((a.case/'semantic_tasks').glob('*.json')):
    task=json.loads(path.read_text())
    if task.get('state')!='SUCCEEDED' or task.get('native_status')!=4 or not task.get('disarmed'):continue
    o=task['selected_object']
    if o['object_id'] in seen:raise ValueError('Multiple successful poses for same ID require explicit human choice')
    seen.add(o['object_id']);objects.append(dict(object_id=o['object_id'],goal=task['goal'],object_position=o['position'],object_dimensions=o['dimensions'],object_orientation_xyzw=o['orientation_xyzw'],source_task=path.name,source_task_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
if not objects:raise ValueError('No successful approach goal available')
a.output.write_text(json.dumps(dict(version=1,frame_id='map',scene_id=case['navigation']['semantic']['world_to_map']['source_scene_sha256'],objects=objects,source_case=a.case.name,world_to_map=case['navigation']['semantic']['world_to_map'],requires_runtime_revalidation=True),ensure_ascii=False,indent=2)+'\n')
