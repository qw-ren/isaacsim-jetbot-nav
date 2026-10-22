"""Fresh warehouse route runtime, without the calibration target; original scene retained."""
import argparse,ast,shutil,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--target',type=Path,required=True);a=p.parse_args();r=Path('/home/admin/go2-warehouse');src=r/'motion_lio_frame20ms_20261019';dst=a.target.resolve()
if dst.parent!=r or not dst.name.startswith('motion_lio_route_') or dst.exists():raise SystemExit('Require new motion_lio_route_* sibling')
s=(src/'isaac_go2_ros2.py').read_text();old='from frame20ms_points import Frame20msTrialAdapter as NativeImuAdapter';assert s.count(old)==1;s=s.replace(old,'from route_trial_adapter import RouteTrialAdapter as NativeImuAdapter');target='    from plane_audit import add_calibration_plane\n    add_calibration_plane(args_cli.artifact_dir)\n';assert s.count(target)==1;s=s.replace(target,'');ast.parse(s)
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'));(dst/'isaac_go2_ros2.py').write_text(s)
for name in ['route_trial_adapter.py','frame20ms_points.py']:shutil.copy2(Path(__file__).with_name(name),dst/name)
(dst/'ROUTE_COPY.json').write_text(json.dumps({'source':str(src),'change':'remove calibration target; original warehouse only; bounded diagnostics; unchanged CPU rough policy, sensors, 20ms input and TF isolation'},indent=2)+'\n');print(dst)
