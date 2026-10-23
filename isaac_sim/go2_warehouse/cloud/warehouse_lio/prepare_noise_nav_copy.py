"""Add native-IMU measurement noise only in a new experiment sibling."""
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
source = root/'motion_lio_nav_20261021'
target = args.target.resolve()
if target.parent != root or not target.name.startswith('motion_lio_noise_') or target.exists():
    raise SystemExit('Require a new motion_lio_noise_* sibling')
entry = (source/'isaac_go2_ros2.py').read_text()
old = 'from route_trial_adapter import RouteTrialAdapter as NativeImuAdapter'
assert entry.count(old) == 1
entry = entry.replace(old, 'from noisy_route_adapter import NoisyRouteAdapter as NativeImuAdapter')
ast.parse(entry)
shutil.copytree(source, target, ignore=shutil.ignore_patterns('__pycache__', 'outputs', '.git'))
(target/'isaac_go2_ros2.py').write_text(entry)
for name in ['noisy_route_adapter.py', 'imu_noise_model.py']:
    shutil.copy2(Path(__file__).with_name(name), target/name)
(target/'NOISE_COPY.json').write_text(json.dumps(dict(
    source=str(source), target=str(target), input='/lio/imu', ideal_evaluation_only='/evaluation/imu_ideal',
    unchanged='physics, controller, lidar, IMU mount/timestamps, guard and scene',
    entry_sha256=hashlib.sha256(entry.encode()).hexdigest()), indent=2)+'\n')
print(target)
