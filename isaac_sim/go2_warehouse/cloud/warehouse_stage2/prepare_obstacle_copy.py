"""Create an isolated M3 fixture copy; preserve stable work and sensor runtime."""
from pathlib import Path
import ast
import shutil
import json
import hashlib

root = Path('/home/admin/go2-warehouse')
source = root / 'motion_lidar_world_20261015'
target = root / 'motion_obstacles_20261018'
if target.exists():
    raise SystemExit('M3 copy exists; inspect instead of overwriting')
shutil.copytree(source, target, ignore=shutil.ignore_patterns('__pycache__', 'outputs', '.git'))
shutil.copy2(Path(__file__).with_name('m3_obstacle_fixture.py'), target / 'm3_obstacle_fixture.py')
entry = target / 'isaac_go2_ros2.py'
text = entry.read_text()
needle = '    while simulation_app.is_running() and not shutdown_requested[0]:'
assert text.count(needle) == 1
text = text.replace(needle, '    from m3_obstacle_fixture import ObstacleFixture\n    fixture = ObstacleFixture(args_cli.artifact_dir)\n' + needle)
needle = '            dm.enforce_command_timeout()'
assert text.count(needle) == 1
text = text.replace(needle, '            fixture.tick()\n' + needle)
ast.parse(text)
entry.write_text(text)
(target / 'M3_COPY.json').write_text(json.dumps({'source': str(source), 'target': str(target), 'entry_sha256': hashlib.sha256(entry.read_bytes()).hexdigest(), 'purpose': 'stationary live-obstacle sensing and planning; no velocity commands'}, indent=2) + '\n')
