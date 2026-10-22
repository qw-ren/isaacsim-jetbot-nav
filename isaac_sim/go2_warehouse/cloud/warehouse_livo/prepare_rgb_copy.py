"""New isolated RGB/noisy-IMU runtime; keep previous successful runtime intact."""
import ast
import hashlib
import json
from pathlib import Path
import shutil

root=Path('/home/admin/go2-warehouse')
source=root/'motion_lio_noise_20261021'
target=root/'motion_livo_rgb_20261022'
if target.exists():raise SystemExit('Refuse overwriting existing runtime')
entry=(source/'isaac_go2_ros2.py').read_text()
entry=entry.replace('sm.add_camera(cfg.freq)', 'sm.add_camera(10)')
entry=entry.replace('    started = time.monotonic()', '''    from rgb_trial_capture import RgbTrialCapture
    rgb_trial = RgbTrialCapture(cameras[0], dm, env, args_cli.artifact_dir)
    started = time.monotonic()''')
entry=entry.replace('            live_points.tick()', '            rgb_trial.tick()\n            live_points.tick()')
entry=entry.replace('    live_points.close()', '    rgb_trial.close()\n    live_points.close()')
assert entry.count('rgb_trial.tick()')==1
ast.parse(entry)
shutil.copytree(source,target,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
(target/'isaac_go2_ros2.py').write_text(entry)
p=target/'go2/go2_sensors.py';s=p.read_text();assert s.count('resolution=(640, 480)')==1
p.write_text(s.replace('resolution=(640, 480)','resolution=(320, 240)'))
shutil.copy2(Path(__file__).with_name('rgb_trial_capture.py'),target/'rgb_trial_capture.py')
# Native Camera stays enabled; suppress duplicate legacy RGB/depth writers.
p=target/'ros2/go2_ros2_bridge.py';t=p.read_text()
assert t.count('        self.create_camera_publisher()')==1
t=t.replace('        self.create_camera_publisher()', '        # RGB-only experiment owns /livo/image_raw; legacy writers disabled here.')
ast.parse(t);p.write_text(t)
(target/'RGB_COPY.json').write_text(json.dumps(dict(source=str(source),target=str(target),
 unchanged='scene, physics, controller, LiDAR, noisy IMU, guards; native Camera mount and focal length',
 camera_changes='native RGB 320x240 at requested10Hz, fixed mount/focal length, native intrinsics and timestamp',
 entry_sha256=hashlib.sha256(entry.encode()).hexdigest()),indent=2)+'\n')
print(target)
