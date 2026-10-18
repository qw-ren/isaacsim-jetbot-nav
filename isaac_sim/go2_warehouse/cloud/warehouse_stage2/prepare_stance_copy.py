#!/usr/bin/env python3
"""Derive isolated instrumented stance copy without altering stable runtimes."""
import ast,hashlib,json,shutil
from pathlib import Path
root=Path('/home/admin/go2-warehouse');src=root/'motion_work_20261015';dst=root/'motion_stance_20261015';scripts=Path(__file__).resolve().parent
if dst.exists():raise SystemExit('Stance copy exists; inspect, do not overwrite')
assert (src/'MOTION_COPY.json').is_file()
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('outputs','__pycache__','.git'))
p=dst/'isaac_go2_ros2.py';s=p.read_text();s=s.replace('parser.add_argument("--spawn-x",','parser.add_argument("--stance-hold",action="store_true")\nparser.add_argument("--spawn-x",',1);s=s.replace('import ros2.go2_ros2_bridge as go2_ros2_bridge','import ros2.go2_ros2_bridge as go2_ros2_bridge\nfrom go2.stance_adapter import StanceAdapter',1);s=s.replace('obs, _ = env.reset()','obs, _ = env.reset()\n    stance = StanceAdapter(env,dm,args_cli.stance_hold,args_cli.artifact_dir,obs)',1);s=s.replace('actions = policy(obs)','actions = stance.adapt(policy(obs))',1);s=s.replace('    run_simulator()','    try:\n        run_simulator()\n    except BaseException:\n        simulation_app.close()\n        raise',1);ast.parse(s);p.write_text(s);shutil.copy2(scripts/'stance_adapter.py',dst/'go2/stance_adapter.py')
meta={'source':str(src),'target':str(dst),'default_hold_enabled':False,'profile':'calibrated_reference_v3','entry_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'adapter_sha256':hashlib.sha256((dst/'go2/stance_adapter.py').read_bytes()).hexdigest(),'physics_root_write':False,'weights_changed':False};(dst/'STANCE_COPY.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta,indent=2))
