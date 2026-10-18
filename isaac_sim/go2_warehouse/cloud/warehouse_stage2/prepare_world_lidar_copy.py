"""Derive isolated world-buffer sensor experiment from guarded rough baseline."""
from pathlib import Path
import shutil,ast,hashlib,json
r=Path('/home/admin/go2-warehouse');src=r/'motion_work_20261015';dst=r/'motion_lidar_world_20261015';scripts=Path(__file__).resolve().parent
if dst.exists():raise SystemExit('World LiDAR copy exists; inspect rather than overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
p=dst/'go2/go2_sensors.py';s=p.read_text();assert s.count('annotator.initialize(transformPoints=False)')==1;s=s.replace('annotator.initialize(transformPoints=False)','annotator.initialize(transformPoints=True)');ast.parse(s);p.write_text(s)
p=dst/'ros2/go2_ros2_bridge.py';s=p.read_text();s='from ros2.world_lidar_adapter import sensor_points\n'+s;needle='self.lidar_annotators[i].get_data()["data"].reshape(-1, 3)';assert s.count(needle)>=1;s=s.replace(needle,'sensor_points(self.lidar_annotators[i], self.env, self.get_clock().now().nanoseconds / 1e9)');ast.parse(s);p.write_text(s)
p=dst/'isaac_go2_ros2.py';s=p.read_text();s=s.replace('    dm = go2_ros2_bridge.RobotDataManager', '    os.environ["GO2_LIDAR_WORLD_ARTIFACTS"] = os.path.join(args_cli.artifact_dir, "sensor")\n    dm = go2_ros2_bridge.RobotDataManager',1);ast.parse(s);p.write_text(s)
shutil.copy2(scripts/'world_lidar_adapter.py',dst/'ros2/world_lidar_adapter.py')
meta={'source':str(src),'target':str(dst),'transformPoints':True,'auxiliary_channels':False,'canonical_cloud_frame_unchanged':True,'weights_gains_physics_changed':False,'hashes':{str(p.relative_to(dst)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [dst/'isaac_go2_ros2.py',dst/'go2/go2_sensors.py',dst/'ros2/go2_ros2_bridge.py',dst/'ros2/world_lidar_adapter.py']}}
(dst/'WORLD_LIDAR_COPY.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta,indent=2))
