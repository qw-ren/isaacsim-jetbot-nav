"""Freeze a new mainline case and source snapshot; never overwrite evidence."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

parser = argparse.ArgumentParser()
parser.add_argument('--case', choices=['branch', 'closed'], required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
root = Path('/home/admin/go2-warehouse')
output = args.output.resolve()
source = Path(__file__).resolve().parent
if output.parent != root / 'artifacts' or output.exists():
    raise SystemExit('Expected a new direct child of cloud artifacts; existing evidence is never overwritten')
prior = root / 'artifacts/m4_global_b_20261018/prior_02'
pose = root / 'artifacts/live_branch_20261019/case_01/initial_pose.json'
config = json.loads((prior / 'simulation/box_config.json').read_text())
boxes = json.loads((prior / 'safety_boxes.json').read_text())
initial = json.loads(pose.read_text())
if args.case == 'closed':
    tree = ast.parse((source / 'closed_route_fixture.py').read_text())
    walls = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == 'WALLS' for t in n.targets))
    for wall in walls:
        c, size = wall['center'], wall['size']
        boxes.append(dict(min=[c[i]-size[i]/2 for i in range(3)], max=[c[i]+size[i]/2 for i in range(3)], source='C physical enclosure '+wall['name']))
else:
    barrier = config['barrier'];c, size = barrier['center'], barrier['size']
    boxes.append(dict(min=[c[i]-size[i]/2 for i in range(3)], max=[c[i]+size[i]/2 for i in range(3)], source='B physical new barrier'))
(output / 'simulation').mkdir(parents=True, exist_ok=False)
(output / 'simulation/box_config.json').write_text(json.dumps(config, indent=2)+'\n')
(output / 'initial_pose.json').write_text(json.dumps(initial, indent=2)+'\n')
(output / 'safety_boxes.json').write_text(json.dumps(boxes)+'\n')
if args.case == 'closed':
    (output / 'simulation/obstacle_request.json').write_text(json.dumps(dict(id='initial_open_for_localization', enabled=False, enclosure_enabled=False))+'\n')
shutil.copytree(source, output / 'source', ignore=shutil.ignore_patterns('__pycache__', '*.pyc', '.git'))
files = sorted(p for p in (output / 'source').rglob('*') if p.is_file())
(output / 'source_files.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(output))+'\n' for p in files))
(output / 'CASE.json').write_text(json.dumps(dict(case=args.case, runtime='motion_live_closed_20261019' if args.case == 'closed' else 'motion_live_branch_20261019', map=str(root / 'nav_stage2/maps/slam_b_prior_20261018_v2/map_nav.yaml'), spawn_odom=[1.,0.,0.], source_files=len(files), scope='stationary enclosure abort/reopen' if args.case == 'closed' else 'preblocked alternate-passage traversal', lio_navigation_input=False), indent=2)+'\n')
print(output)
