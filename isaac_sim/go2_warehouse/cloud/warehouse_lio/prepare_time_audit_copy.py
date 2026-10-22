"""Prepare a new CPU sensor audit copy without touching accepted runtimes."""
import argparse,ast,hashlib,json,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--target',type=Path,required=True);args=p.parse_args()
root=Path('/home/admin/go2-warehouse').resolve();src=args.source.resolve();dst=args.target.resolve()
if src.parent!=root or dst.parent!=root or not dst.name.startswith('motion_lio_time_'):raise SystemExit('Expected sibling motion_lio_time_* target under cloud root')
if dst.exists():raise SystemExit('Target exists: inspect rather than overwrite')
s=(src/'isaac_go2_ros2.py').read_text()
a='from native_imu_adapter import NativeImuAdapter';b='from rtx_time_audit import TimeAuditAdapter as NativeImuAdapter'
if s.count(a)!=1 or 'go2_env_cfg.sim.device = "cpu"' not in s:raise SystemExit('Source must be the validated CPU native IMU runtime')
s=s.replace(a,b);ast.parse(s)
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
(dst/'isaac_go2_ros2.py').write_text(s)
shutil.copy2(Path(__file__).with_name('rtx_time_audit.py'),dst/'rtx_time_audit.py')
(dst/'TIME_AUDIT_COPY.json').write_text(json.dumps({'source':str(src),'target':str(dst),'entry_sha256':hashlib.sha256(s.encode()).hexdigest(),'purpose':'diagnostics only, no LIO points or odometry output; unchanged CPU rough policy and safety'},indent=2)+'\n')
print(dst)
