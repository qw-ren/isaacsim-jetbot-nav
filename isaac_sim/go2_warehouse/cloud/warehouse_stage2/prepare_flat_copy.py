#!/usr/bin/env python3
"""Compare the already supplied flat policy in an isolated guarded runtime copy."""
import ast,hashlib,json,shutil
from pathlib import Path
root=Path('/home/admin/go2-warehouse');src=root/'motion_work_20261015';dst=root/'motion_flat_20261015'
if dst.exists():raise SystemExit('Flat copy exists; inspect, do not overwrite')
assert (src/'MOTION_COPY.json').exists()
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('outputs','__pycache__','.git'))
p=dst/'isaac_go2_ros2.py';s=p.read_text();old='env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)';assert s.count(old)==1;s=s.replace(old,'env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)');ast.parse(s);p.write_text(s)
meta={'source':str(src),'target':str(dst),'change':'Existing flat inference policy instead of rough, guards unchanged','entry_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'flat_weight_sha256':hashlib.sha256((dst/'ckpts/unitree_go2/flat_model_6800.pt').read_bytes()).hexdigest(),'bridge_sha256':hashlib.sha256((dst/'ros2/go2_ros2_bridge.py').read_bytes()).hexdigest()};(dst/'FLAT_COPY.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta,indent=2))
