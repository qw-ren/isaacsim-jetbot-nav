"""Compatibility experiment only: native IMU with CPU physics, identical rough weights."""
from pathlib import Path
import shutil,ast,json,hashlib
r=Path('/home/admin/go2-warehouse');src=r/'motion_lio_sensors_20261018';dst=r/'motion_lio_cpu_sensors_20261018'
if dst.exists():raise SystemExit('CPU LIO runtime exists; inspect rather than overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
p=dst/'isaac_go2_ros2.py';s=p.read_text();anchor='go2_env_cfg.scene.num_envs = cfg.num_envs';assert s.count(anchor)==1;s=s.replace(anchor,anchor+'\n    go2_env_cfg.sim.device = "cpu"\n    go2_ctrl.unitree_go2_rough_cfg["device"] = "cpu"');ast.parse(s);p.write_text(s)
(dst/'CPU_LIO_COPY.json').write_text(json.dumps({'source':str(src),'target':str(dst),'change':'CPU physics and CPU inference device for native IMU compatibility; same rough weights/sensors/guard limits; RTX rendering remains GPU','entry_sha256':hashlib.sha256(p.read_bytes()).hexdigest()},indent=2)+'\n')
