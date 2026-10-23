"""Independent GPU-physics live-obstacle runtime; accepted copies stay untouched."""
from pathlib import Path
import ast,json,shutil,hashlib,argparse
root=Path('/home/admin/go2-warehouse')
parser=argparse.ArgumentParser();parser.add_argument('--source',default='motion_obstacles_20261018');parser.add_argument('--target',default='motion_live_obstacles_20261019');args=parser.parse_args()
src=(root/args.source).resolve();dst=(root/args.target).resolve()
if src.parent!=root or dst.parent!=root or not dst.name.startswith('motion_live_'): raise SystemExit('Expected isolated sibling motion_live_* runtime')
if dst.exists(): raise SystemExit('Runtime exists: inspect, do not overwrite')
shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
shutil.copy2(Path(__file__).with_name('live_rtx_points.py'),dst/'live_rtx_points.py')
p=dst/'go2/go2_sensors.py'; text=p.read_text()
a='lidar_annotators = []'; assert text.count(a)==1
text=text.replace(a,a+'\n        self.live_raw_annotators = []')
a='annotator.attach(hydra_texture.path)'; assert text.count(a)==1
text=text.replace(a,a+'''\n            raw = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacReadRTXLidarData")
            raw.initialize(keepOnlyPositiveDistance=False)
            raw.attach(hydra_texture.path)
            self.live_raw_annotators.append(raw)''')
ast.parse(text);p.write_text(text)
p=dst/'ros2/go2_ros2_bridge.py';text=p.read_text();a='from ros2.world_lidar_adapter import sensor_points';assert text.count(a)==1
text=text.replace(a,'from live_rtx_points import sensor_points');ast.parse(text);p.write_text(text)
p=dst/'isaac_go2_ros2.py';text=p.read_text();a='fixture = ObstacleFixture(args_cli.artifact_dir)';assert text.count(a)==1
text=text.replace(a,a+'\n    from live_rtx_points import LiveRtxPoints\n    live_points = LiveRtxPoints(sm.live_raw_annotators[0], lidar_annotators[0], env, args_cli.artifact_dir)')
a='            dm.pub_ros2_data()';assert text.count(a)==1
text=text.replace(a,'            live_points.tick()\n'+a)
a='go2_ctrl.base_vel_cmd_input.zero_()';assert text.count(a)==1
text=text.replace(a,'live_points.close()\n    '+a);ast.parse(text);p.write_text(text)
files=['isaac_go2_ros2.py','go2/go2_sensors.py','ros2/go2_ros2_bridge.py','live_rtx_points.py']
(dst/'LIVE_OBSTACLE_COPY.json').write_text(json.dumps(dict(source=str(src),target=str(dst),purpose='current five-render-packet navigation cloud and raw-vs-buffer comparison; no FAST-LIO changes',files_sha256={f:hashlib.sha256((dst/f).read_bytes()).hexdigest() for f in files}),indent=2)+'\n')
print(dst)
