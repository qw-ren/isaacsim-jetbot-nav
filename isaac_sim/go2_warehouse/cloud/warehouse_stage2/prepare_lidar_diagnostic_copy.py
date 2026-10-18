"""Derive a single-use instrumented runtime; never modify accepted work/baseline."""
from pathlib import Path
import ast,hashlib,json,shutil
r=Path('/home/admin/go2-warehouse');src=r/'motion_stance_20261015';dst=r/'motion_lidar_diag_20261015';s=Path(__file__).resolve().parent
if dst.exists():raise SystemExit('Diagnostic runtime exists; inspect rather than overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
p=dst/'go2/go2_sensors.py';v=p.read_text();old='annotator.initialize(transformPoints=False)';assert v.count(old)==1;v=v.replace(old,'annotator.initialize(transformPoints=False, outputTimestamp=True, outputObjectId=True, outputNormal=True)');ast.parse(v);p.write_text(v)
p=dst/'ros2/go2_ros2_bridge.py';v=p.read_text();v='from ros2.lidar_temporal_diagnostics import capture_scan\n'+v;old='self.lidar_annotators[i].get_data()["data"].reshape(-1, 3)';assert v.count(old)>=1;v=v.replace(old,'capture_scan(self.lidar_annotators[i], self.get_clock().now().nanoseconds / 1e9, self.env)');ast.parse(v);p.write_text(v)
shutil.copy2(s/'lidar_temporal_diagnostics.py',dst/'ros2/lidar_temporal_diagnostics.py')
meta={'source':str(src),'target':str(dst),'extra_channels_only':True,'transformPoints':False,'capture_sim_window':[17.99,18.055],'files_sha256':{str(p.relative_to(dst)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [dst/'go2/go2_sensors.py',dst/'ros2/go2_ros2_bridge.py',dst/'ros2/lidar_temporal_diagnostics.py']}}
(dst/'LIDAR_DIAGNOSTIC_COPY.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta,indent=2))
