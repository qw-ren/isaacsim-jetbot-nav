from pathlib import Path
import json,hashlib,shutil
r=Path('/home/admin/go2-warehouse');p=r/'motion_obstacles_20261018/go2/go2_sensors.py';s=p.read_text();old='annotator.initialize(transformPoints=True)';assert s.count(old)==1
out=r/'artifacts/m3_costmap_20261018/sensor_before_unfiltered';out.mkdir(exist_ok=False);shutil.copy2(p,out/p.name)
s=s.replace(old,'annotator.initialize(transformPoints=True, keepOnlyPositiveDistance=False)');p.write_text(s)
(r/'motion_obstacles_20261018/M3_UNFILTERED.json').write_text(json.dumps({'change':'keepOnlyPositiveDistance=False; no scan height/range or safety thresholds changed','sha256':hashlib.sha256(p.read_bytes()).hexdigest()},indent=2)+'\n')
