"""New sibling calibration runtime and guard geometry; never overwrite existing data."""
import argparse,ast,json,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--target',required=True,type=Path);p.add_argument('--boxes-output',required=True,type=Path);a=p.parse_args()
r=Path('/home/admin/go2-warehouse');src=r/'motion_lio_time_20261019';dst=a.target.resolve()
if dst.parent!=r or not dst.name.startswith('motion_lio_plane_') or dst.exists():raise SystemExit('Require a new motion_lio_plane_* sibling')
if a.boxes_output.exists():raise SystemExit('Guard box output exists; do not overwrite')
s=(src/'isaac_go2_ros2.py').read_text();old='from rtx_time_audit import TimeAuditAdapter as NativeImuAdapter';assert s.count(old)==1;s=s.replace(old,'from plane_audit import PlaneAuditAdapter as NativeImuAdapter');anchor='    # Sensor setup';assert s.count(anchor)==1;s=s.replace(anchor,'    from plane_audit import add_calibration_plane\n    add_calibration_plane(args_cli.artifact_dir)\n\n'+anchor);ast.parse(s)
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'));(dst/'isaac_go2_ros2.py').write_text(s)
for name in ['plane_audit.py','rtx_time_audit.py']:shutil.copy2(Path(__file__).with_name(name),dst/name)
boxes=json.loads((r/'artifacts/m4_open_a_20261018/height_clipped/static_boxes.json').read_text());boxes.append({'path':'/World/LioCalibrationTarget','min':[6.45,-4.,0.],'max':[6.55,2.,3.],'purpose':'calibration only'})
a.boxes_output.parent.mkdir(parents=True,exist_ok=True);a.boxes_output.write_text(json.dumps(boxes,indent=2)+'\n');print(dst)
