"""New CPU-physics LIO navigation sibling, without extra scene obstacles."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

parser = argparse.ArgumentParser()
parser.add_argument('--target', type=Path, required=True)
args = parser.parse_args()
root = Path('/home/admin/go2-warehouse')
source = root/'motion_lio_route_20261019'
target = args.target.resolve()
scripts = Path(__file__).resolve().parent
stage2 = scripts.parent/'warehouse_stage2'
if target.parent != root or not target.name.startswith('motion_lio_nav_') or target.exists():
    raise SystemExit('Require a new motion_lio_nav_* sibling; existing runs are never overwritten')
entry = (source/'isaac_go2_ros2.py').read_text()
bridge = (source/'ros2/go2_ros2_bridge.py').read_text()


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise RuntimeError('Unexpected source anchor: '+before)
    return text.replace(before, after)


# Only this simulator node's truth odom/pose/dynamic TF are isolated. Existing
# sensor publishers, clock, static extrinsics and the command watchdog remain.
bridge = replace_once(bridge, 'super().__init__("robot_data_manager")', '''super().__init__("robot_data_manager", cli_args=[
            "--ros-args", "-r", "/unitree_go2/odom:=/truth/odom",
            "-r", "/unitree_go2/pose:=/truth/pose", "-r", "/tf:=/truth/bridge_tf"])''')
bridge = replace_once(bridge, 'from ros2.world_lidar_adapter import sensor_points',
                      'from live_rtx_points import sensor_points')
entry = replace_once(entry, '    lio_sensors.attach()', '''    lio_sensors.attach()
    from live_rtx_points import LiveRtxPoints
    live_points = LiveRtxPoints(sm.lio_raw_annotators[0], lidar_annotators[0], env, args_cli.artifact_dir)''')
entry = replace_once(entry, '            dm.pub_ros2_data()',
                      '            live_points.tick()\n            dm.pub_ros2_data()')
entry = replace_once(entry, '    lio_sensors.close()', '    live_points.close()\n    lio_sensors.close()')
ast.parse(entry)
ast.parse(bridge)
assert 'add_calibration_plane(' not in entry and 'ObstacleFixture' not in entry
shutil.copytree(source, target, ignore=shutil.ignore_patterns('__pycache__', 'outputs', '.git'))
(target/'isaac_go2_ros2.py').write_text(entry)
(target/'ros2/go2_ros2_bridge.py').write_text(bridge)
shutil.copy2(stage2/'live_rtx_points.py', target/'live_rtx_points.py')
files = ['isaac_go2_ros2.py', 'ros2/go2_ros2_bridge.py', 'live_rtx_points.py']
(target/'NAV_COPY.json').write_text(json.dumps(dict(
    source=str(source), target=str(target), scene='unchanged warehouse shelves; no new boxes or calibration target',
    changes='isolate Bridge truth odom/pose/TF; fresh native navigation cloud; original LIO inputs and CPU rough controller',
    truth_outputs=['/truth/odom', '/truth/pose', '/truth/bridge_tf'],
    file_sha256={name:hashlib.sha256((target/name).read_bytes()).hexdigest() for name in files}), indent=2)+'\n')
print(target)
