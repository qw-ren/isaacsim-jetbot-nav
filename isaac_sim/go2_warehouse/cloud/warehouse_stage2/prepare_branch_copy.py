from pathlib import Path
import shutil,hashlib,json
r=Path('/home/admin/go2-warehouse');src=r/'motion_open_obstacles_20261018';dst=r/'motion_branch_obstacles_20261018'
if dst.exists():raise SystemExit('B runtime exists; inspect rather than overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
shutil.copy2(Path(__file__).with_name('branch_obstacle_fixture.py'),dst/'branch_obstacle_fixture.py')
p=dst/'isaac_go2_ros2.py';s=p.read_text();assert s.count('from open_obstacle_fixture import ObstacleFixture')==1;p.write_text(s.replace('from open_obstacle_fixture import ObstacleFixture','from branch_obstacle_fixture import ObstacleFixture'))
(dst/'BRANCH_COPY.json').write_text(json.dumps({'source':str(src),'target':str(dst),'entry_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'change':'B fixture only: prior divider plus toggled new barrier; policy/sensors/physics/bridge unchanged'},indent=2)+'\n')
