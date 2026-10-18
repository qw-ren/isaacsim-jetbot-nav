"""Stationary check of full-buffer auxiliary alignment before using provenance."""
from pathlib import Path
import ast,shutil,json,hashlib
r=Path('/home/admin/go2-warehouse');src=r/'motion_lidar_full_20261015';dst=r/'motion_lidar_unfiltered_20261015'
if dst.exists():raise SystemExit('Full-buffer diagnostic copy exists; refusing overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
p=dst/'go2/go2_sensors.py';s=p.read_text();needle='transformPoints=False, outputTimestamp=True';assert s.count(needle)==1;s=s.replace(needle,'transformPoints=False, keepOnlyPositiveDistance=False, outputTimestamp=True');ast.parse(s);p.write_text(s)
p=dst/'ros2/lidar_temporal_diagnostics.py';s=p.read_text().replace('m1_lidar_aux_full_20261015','m1_lidar_aux_unfiltered_20261015').replace('return points','return points[np.asarray(raw["distance"]) > 0]');ast.parse(s);p.write_text(s)
meta={'source':str(src),'target':str(dst),'aux_output':'FULL','keep_only_positive_distance':False,'bridge_filters_positive_distance':True,'stationary_only':True,'hashes':{str(p.relative_to(dst)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [dst/'go2/go2_sensors.py',dst/'ros2/lidar_temporal_diagnostics.py']}}
(dst/'LIDAR_UNFILTERED_COPY.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta,indent=2))
