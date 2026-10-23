"""Freeze an RGB-enabled LIO navigation capture for subsequent paired replay."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

parser=argparse.ArgumentParser()
parser.add_argument('--case',required=True)
parser.add_argument('--route',choices=['short','short-return','long','open-turn-long'],default='short')
parser.add_argument('--backend',choices=['lio','livo'],default='lio')
args=parser.parse_args()
if not args.case.replace('_','').isalnum():raise SystemExit('Invalid case identifier')
root=Path('/home/admin/go2-warehouse')
cloud=root/'nav_stage2/cloud'
case=root/'artifacts'/('livo_rgb_'+args.case)
case.mkdir(exist_ok=False)
(case/'simulation').mkdir()
for name in ['warehouse_lio','warehouse_stage2','warehouse_livo']:
    shutil.copytree(cloud/name,case/'source'/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
stage2=case/'source/warehouse_stage2'
for name,var in [('make_localization_params.py','pose'),('seed_amcl_pose.py','d')]:
    p=stage2/name;s=p.read_text();a=f"{var}['source'].startswith('SLAM_')"
    if a in s:s=s.replace(a,f"({a} or {var}['source'].startswith('CONFIGURED_CAD_'))")
    p.write_text(s)
p=stage2/'start_localization.sh';s=p.read_text()
a='"$go2_dir/seed_amcl_pose.py"';assert s.count(a)==1
s=s.replace(a,'"$go2_dir/../warehouse_lio/seed_lio_localization.py"')
a='"${go2_params%.yaml}_seed.json"';assert s.count(a)==1
s=s.replace(a,'"'+str(case/'localization_seed.json')+'"');p.write_text(s)
p=stage2/'start_velocity_guard.sh';s=p.read_text()
a='"$go2_script_dir/velocity_guard.py"';assert s.count(a)==1
s=s.replace(a,'"$go2_script_dir/../warehouse_lio/lio_map_velocity_guard.py"');p.write_text(s)
lio=case/'source/warehouse_lio'
p=lio/'record_lio_evaluation.py';s=p.read_text()
s=s.replace('from sensor_msgs.msg import Imu, PointCloud2, LaserScan','from sensor_msgs.msg import Imu, PointCloud2, LaserScan, Image, CameraInfo')
s=s.replace("TOPICS = {", "TOPICS = {\n    '/livo/image_raw':Image,\n    '/livo/camera_info':CameraInfo,")
s=s.replace("reliable_inputs = {", "reliable_inputs = {'/livo/image_raw','/livo/camera_info',")
s=s.replace('depth=20 if kind is PointCloud2 else 1000','depth=20 if kind in (PointCloud2, Image) else 1000')
p.write_text(s)
# Recompute native path periodically, including each new goal. Do not retain a
# previous leg's valid path merely because its cells remain collision-free.
(stage2/'navigate_valid_path.xml').write_text('''<root main_tree_to_execute="MainTree">
<BehaviorTree ID="MainTree"><PipelineSequence name="NavigateWithPeriodicReplanning">
<RateController hz="4.0"><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/></RateController>
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker"/>
</PipelineSequence></BehaviorTree></root>
''')
s=p.read_text()
s=s.replace('from nav_msgs.msg import Odometry, OccupancyGrid','from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath')
s=s.replace("TOPICS = {", "TOPICS = {\n    '/plan':NavPath,")
p.write_text(s)
p=lio/'run_nav_trial.sh';s=p.read_text().replace('sensor.enable_camera=False','sensor.enable_camera=True sensor.depth_image=False');p.write_text(s)
if args.backend=='livo':
    lines=s.splitlines()
    candidates=[i for i,line in enumerate(lines) if 'setsid env LD_LIBRARY_PATH=' in line and 'fastlio_mapping' in line]
    assert len(candidates)==1
    i=candidates[0]
    lines[i]='''  "$GO2_ISAAC_ROOT/python.sh" "$go2_lio_dir/../warehouse_livo/generate_livo_params.py" --contract "$go2_case/simulation/rgb/camera_contract.json" --output "$go2_case/livo_live_params.yaml" --imu-propagation
  setsid env LD_LIBRARY_PATH="$go2_root/nav_stage2/deps/fastlivo_trial_20261022/lib:$go2_root/nav_stage2/deps/fastlio_eval_20261019/lib:$LD_LIBRARY_PATH" "$go2_root/nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping" --ros-args --params-file "$go2_case/livo_live_params.yaml" -r __node:=fastlivo_navigation_trial -r /tf:=/livo/evaluation_tf -r /tf_static:=/livo/evaluation_tf_static -r /aft_mapped_to_init:=/livo/filter_updates -r /LIVO2/imu_propagate:=/lio/odometry > "$go2_case/fastlivo_live.log" 2>&1 &'''
    s='\n'.join(lines)+'\n'
    a='"$go2_lio_dir/lio_nav_odometry.py"';assert s.count(a)==1
    s=s.replace(a,'"$go2_lio_dir/../warehouse_livo/livo_nav_odometry.py"');p.write_text(s)
config=json.loads((root/'artifacts/lio_nav_20261021_noise_long_05/CASE.json').read_text())
# Deployment uses the native accelerometer/gyro; keep old noisy captures immutable.
config['imu_noise_enabled']=False
config['imu_mode']='native_accelerometer_gyro; artificial_noise_disabled'
config['runtime']='motion_livo_rgb_20261022'
if args.route in ['short','short-return']:
    config['route']=None
    config['map']=str(root/'nav_stage2/maps/warehouse_dev_v2/map.yaml')
    config.pop('planned_route_m',None)
    if args.route=='short-return':
        f=case/'return_route.json';f.write_text(json.dumps(dict(goals=[dict(xy=[5.,-1.]),dict(xy=[2.,-1.])]))+'\n')
        config['route']=str(f)
elif args.route=='open-turn-long':
    d=root/'nav_stage2/maps/warehouse_livo_open_turns_20261022'
    config['map']=str(d/'map.yaml');config['route']=str(d/'route.json')
    config['planned_route_m']=json.loads((d/'route.json').read_text())['planned_route_m']
config.update(rgb_enabled=True,online_estimator='FAST-LIO2',livo_used_for_control=False,
              native_replanning='periodic ComputePathToPose 4Hz; no recovery/clearing',
              purpose='common RGB/noisy-IMU/LiDAR dataset; LIVO replay evaluated separately')
if args.backend=='livo':
    config.update(online_estimator='FAST-LIVO2',livo_used_for_control=True,
        estimator_binary=str(root/'nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping'),
        estimator_binary_sha256=hashlib.sha256((root/'nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping').read_bytes()).hexdigest(),
        estimator_output='native LIVO IMU propagation with visual/LiDAR corrections; measurement stamps',
        purpose='isolated live LIVO navigation trial; old stable defaults preserved')
    config.pop('fastlio_binary',None);config.pop('fastlio_binary_sha256',None)
(case/'CASE.json').write_text(json.dumps(config,indent=2)+'\n')
(case/'initial_pose.json').write_text(json.dumps(dict(source='CONFIGURED_CAD_scan_projection_known_spawn',x=2.2,y=-1.,yaw=0.))+'\n')
runtime=root/config['runtime']
shutil.copytree(runtime,case/'runtime_source',ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
files=sorted(p for base in [case/'source',case/'runtime_source'] for p in base.rglob('*') if p.is_file())
(case/'source_files.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(case))+'\n' for p in files))
print(case)
