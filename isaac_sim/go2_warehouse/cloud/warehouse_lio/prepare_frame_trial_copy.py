"""Clone the plane-calibration runtime for explicitly approximate point-stream trials."""
import argparse,ast,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--target',type=Path,required=True);a=p.parse_args();r=Path('/home/admin/go2-warehouse');src=r/'motion_lio_plane_20261019';dst=a.target.resolve()
if dst.parent!=r or not dst.name.startswith('motion_lio_frame20ms_') or dst.exists():raise SystemExit('Require new motion_lio_frame20ms_* sibling')
s=(src/'isaac_go2_ros2.py').read_text();old='from plane_audit import PlaneAuditAdapter as NativeImuAdapter';assert s.count(old)==1;s=s.replace(old,'from frame20ms_points import Frame20msTrialAdapter as NativeImuAdapter');ast.parse(s)
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'));(dst/'isaac_go2_ros2.py').write_text(s)
shutil.copy2(Path(__file__).with_name('frame20ms_points.py'),dst/'frame20ms_points.py');print(dst)
