"""Hold controller unchanged; replay known scan fault with world-coordinate LiDAR."""
from pathlib import Path
import shutil,json,ast,hashlib
r=Path('/home/admin/go2-warehouse');src=r/'motion_lidar_world_20261015';control=r/'motion_stance_20261015';dst=r/'motion_lidar_world_stance_20261015';scripts=Path(__file__).resolve().parent
if dst.exists():raise SystemExit('World/stance replay copy exists; refusing overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
p=dst/'isaac_go2_ros2.py';s=(control/'isaac_go2_ros2.py').read_text();needle='    dm = go2_ros2_bridge.RobotDataManager';assert s.count(needle)==1;s=s.replace(needle,'    os.environ["GO2_LIDAR_WORLD_ARTIFACTS"] = os.path.join(args_cli.artifact_dir, "sensor")\n'+needle,1);ast.parse(s);p.write_text(s)
shutil.copy2(control/'go2/stance_adapter.py',dst/'go2/stance_adapter.py');shutil.copy2(scripts/'world_lidar_adapter.py',dst/'ros2/world_lidar_adapter.py')
meta={'sensor_source':str(src),'control_source':str(control),'target':str(dst),'control_profile':'calibrated_reference_v3_FAILED_stance_candidate','sensor_profile':'RTX_world_to_current_lidar','physics_weights_gains_changed':False,'hashes':{str(p.relative_to(dst)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [dst/'isaac_go2_ros2.py',dst/'go2/stance_adapter.py',dst/'ros2/world_lidar_adapter.py']}}
(dst/'WORLD_STANCE_COPY.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta,indent=2))
