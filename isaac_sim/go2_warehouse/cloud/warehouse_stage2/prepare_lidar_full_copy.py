"""Isolated stationary FULL-output comparison; retain all previous diagnostics."""
from pathlib import Path
import shutil,ast,json,hashlib
r=Path('/home/admin/go2-warehouse');src=r/'motion_lidar_diag_20261015';dst=r/'motion_lidar_full_20261015'
if dst.exists():raise SystemExit('FULL diagnostic copy exists; refusing overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
p=dst/'cloud/warehouse_stage1/start_warehouse.sh';s=p.read_text();needle='go2_kit_args="';assert s.count(needle)==1;s=s.replace(needle,'go2_kit_args="--/app/sensors/nv/lidar/auxDataOutput=FULL ',1);p.write_text(s)
p=dst/'ros2/lidar_temporal_diagnostics.py';s=p.read_text().replace('artifacts/m1_lidar_temporal_20261015/sensor','artifacts/m1_lidar_aux_full_20261015/sensor').replace('17.99 <= stamp <= 18.055','1.0 <= stamp <= 1.055').replace('for i in np.unique(v)}','for i in np.unique(v)[:32]}');ast.parse(s);p.write_text(s)
meta={'source':str(src),'target':str(dst),'aux_output':'FULL','stationary_only':True,'capture_window_sim_s':[1.,1.055],'mapping_limited_to_first_unique_ids':32,'hashes':{str(p.relative_to(dst)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [dst/'cloud/warehouse_stage1/start_warehouse.sh',dst/'ros2/lidar_temporal_diagnostics.py']}}
(dst/'LIDAR_FULL_COPY.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps(meta,indent=2))
