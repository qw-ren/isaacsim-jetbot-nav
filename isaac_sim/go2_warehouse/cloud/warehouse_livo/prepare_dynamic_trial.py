"""Freeze a selected-estimator, motion-insertion trial; previous cases untouched."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

parser=argparse.ArgumentParser()
parser.add_argument('--case',required=True)
parser.add_argument('--kind',choices=['A','B','C'],required=True)
parser.add_argument('--route',choices=['short','long'],default='short')
parser.add_argument('--selection',type=Path,required=True)
parser.add_argument('--map-directory',type=Path)
parser.add_argument('--profile',choices=['standard','fast-video'],default='standard')
args=parser.parse_args()
if not args.case.replace('_','').isalnum():raise SystemExit('Invalid case')
selection=json.loads(args.selection.read_text())
if not selection.get('live_navigation_verified'):raise SystemExit('Selected backend must first pass actual live navigation')
if selection['chosen'] not in ['FAST-LIO2','FAST-LIVO2']:raise SystemExit('Unsupported selected backend')
use_livo=selection['chosen']=='FAST-LIVO2'
if args.profile=='fast-video' and not use_livo:
    raise SystemExit('fast-video is implemented for FAST-LIVO2 only; standard FAST-LIO2 remains separate')
root=Path('/home/admin/go2-warehouse');cloud=root/'nav_stage2/cloud'
runtime=root/('motion_dynamic_livo_20261022' if use_livo else 'motion_dynamic_lio_20261022')
if not runtime.exists():
    shutil.copytree(root/('motion_livo_rgb_20261022' if use_livo else 'motion_lio_noise_20261021'),runtime,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
    p=runtime/'isaac_go2_ros2.py';s=p.read_text()
    a='    started = time.monotonic()';assert s.count(a)==1
    s=s.replace(a,'    from dynamic_fixture import DynamicFixture\n    dynamic_fixture = DynamicFixture(env, args_cli.artifact_dir)\n'+a)
    a='            dm.enforce_command_timeout()';assert s.count(a)==1
    s=s.replace(a,'            dynamic_fixture.tick()\n'+a);ast.parse(s);p.write_text(s)
    shutil.copy2(Path(__file__).with_name('dynamic_fixture.py'),runtime/'dynamic_fixture.py')
if args.profile=='fast-video':
    # A separate runtime and frozen source: standard .8m/s remains unchanged.
    fast_runtime=root/('motion_dynamic_livo_fastvideo_v3_20261022' if use_livo else 'motion_dynamic_lio_fastvideo_v3_20261022')
    if not fast_runtime.exists():
        shutil.copytree(runtime,fast_runtime,ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
        p=fast_runtime/'ros2/go2_ros2_bridge.py';s=p.read_text()
        a='np.clip(values[0], 0.0, 0.8)';assert s.count(a)==1
        p.write_text(s.replace(a,'np.clip(values[0], 0.0, 1.7)'))
        shutil.copy2(Path(__file__).with_name('follow_video_capture.py'),fast_runtime/'follow_video_capture.py')
        p=fast_runtime/'isaac_go2_ros2.py';s=p.read_text()
        a='    started = time.monotonic()';assert s.count(a)==1
        s=s.replace(a,'    from follow_video_capture import FollowVideoCapture\n    follow_video = FollowVideoCapture(env, args_cli.artifact_dir)\n'+a)
        a='            rgb_trial.tick()';assert s.count(a)==1
        s=s.replace(a,'            follow_video.tick()\n'+a)
        s=s.replace('    rgb_trial.close()','    follow_video.close()\n    rgb_trial.close()')
        ast.parse(s);p.write_text(s)
    runtime=fast_runtime
case=root/'artifacts'/('moving_'+args.case);case.mkdir(exist_ok=False)
for n in ['warehouse_lio','warehouse_livo','warehouse_stage2']:
    shutil.copytree(cloud/n,case/'source'/n,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
stage2=case/'source/warehouse_stage2';lio=case/'source/warehouse_lio';livo=case/'source/warehouse_livo'
mapdir=root/'nav_stage2/maps'/('moving_C_observable_20261022' if args.kind=='C' else 'moving_'+args.kind+'_20261022')
if args.map_directory is not None:
    mapdir=args.map_directory.resolve()
    if mapdir.parent!=root/'nav_stage2/maps':raise SystemExit('Map must be an existing stage2 maps child')
    if json.loads((mapdir/'dynamic_config.json').read_text())['kind']!=args.kind:raise SystemExit('Map kind mismatch')
if not mapdir.exists():raise SystemExit('Prepare topology proposal first')
(case/'simulation').mkdir()
shutil.copy2(mapdir/'dynamic_config.json',case/'simulation/dynamic_config.json')
shutil.copy2(args.selection,case/'estimator_selection.json')
for name,var in [('make_localization_params.py','pose'),('seed_amcl_pose.py','d')]:
    p=stage2/name;s=p.read_text();a=f"{var}['source'].startswith('SLAM_')";assert s.count(a)==1
    s=s.replace(a,f"{var}['source'].startswith(('SLAM_', 'CONFIGURED_CAD_'))");p.write_text(s)
p=stage2/'start_localization.sh';s=p.read_text()
a='"$go2_dir/seed_amcl_pose.py"';assert s.count(a)==1;s=s.replace(a,'"$go2_dir/../warehouse_lio/seed_lio_localization.py"')
a='"${go2_params%.yaml}_seed.json"';assert s.count(a)==1;s=s.replace(a,'"'+str(case/'localization_seed.json')+'"');p.write_text(s)
p=stage2/'start_velocity_guard.sh';s=p.read_text()
a='"$go2_script_dir/velocity_guard.py"';assert s.count(a)==1
s=s.replace(a,'"$go2_script_dir/../warehouse_livo/dynamic_velocity_guard.py" --fixture "'+str(case/'simulation/obstacle_state.json')+'"');p.write_text(s)
# Higher path-validity polling only; preserve all geometry, velocity and watchdog limits.
p=stage2/'navigate_valid_path.xml'
p.write_text('''<root main_tree_to_execute="MainTree">
<BehaviorTree ID="MainTree"><PipelineSequence name="NavigateWithPeriodicReplanning">
<RateController hz="4.0"><ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/></RateController>
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker"/>
</PipelineSequence></BehaviorTree></root>
''')
# Native continuous prelude avoids stop-and-reverse at every waypoint.
(stage2/'navigate_through_poses.xml').write_text('''<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">
<PipelineSequence name="ContinuousWarehousePrelude">
<RateController hz="4.0"><Sequence>
<RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="0.7" global_frame="map" robot_base_frame="unitree_go2/base_link"/>
<RetryUntilSuccessful num_attempts="3"><Fallback><ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased"/><Delay delay_msec="2500"><AlwaysFailure/></Delay></Fallback></RetryUntilSuccessful>
</Sequence></RateController>
<FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="goal_checker"/>
</PipelineSequence></BehaviorTree></root>
''')
nav=stage2/'nav_obstacle_region_arrival.yaml';text=nav.read_text()
assert 'plugin_lib_names: [' in text
text=text.replace('plugin_lib_names: [','plugin_lib_names: [nav2_remove_passed_goals_action_bt_node, ');nav.write_text(text)
p=lio/'run_nav_trial.sh';s=p.read_text()
s=s.replace('--spawn-x 2 --spawn-y -1','--spawn-x 1 --spawn-y 0')
s=s.replace('--anchor-xyz 2 -1 .4','--anchor-xyz 1 0 .4')
s=s.replace('if [[ -z "$go2_route" ]];then go2_record_args+=(--include-nav-cloud);fi', '# Dynamic trial records source LiDAR and scan; omit duplicate navigation XYZ cloud.')
s=s.replace('"$go2_root/artifacts/m4_open_a_20261018/height_clipped/static_boxes.json"','"'+str(mapdir/'safety_boxes.json')+'"')
s=s.replace('"$go2_lio_dir/check_lio_navigation.py"','"$go2_lio_dir/../warehouse_livo/check_moving_blockage.py"')
a='"${go2_target_args[@]}" --adapter-output'
assert a in s
routeflag=''
if args.route=='long':
    route_name='warehouse_livo_loop_20261022_v4' if args.profile=='fast-video' else 'warehouse_livo_loop_20261022_v3'
    shutil.copy2(root/'nav_stage2/maps'/route_name/'route.json',case/'prelude_route.json')
    routeflag=' --prelude "'+str(case/'prelude_route.json')+'"'

s=s.replace(a,'--fixture "'+str(case/'simulation')+'"'+routeflag+' --adapter-output')
if use_livo:
    s=s.replace('sensor.enable_camera=False','sensor.enable_camera=True sensor.depth_image=False')
    lines=s.splitlines();indices=[i for i,v in enumerate(lines) if 'setsid env LD_LIBRARY_PATH=' in v and 'fastlio_mapping' in v]
    assert len(indices)==1
    lines[indices[0]]='''  "$GO2_ISAAC_ROOT/python.sh" "$go2_lio_dir/../warehouse_livo/generate_livo_params.py" --contract "$go2_case/simulation/rgb/camera_contract.json" --output "$go2_case/livo_live_params.yaml" --imu-propagation
  setsid env LD_LIBRARY_PATH="$go2_root/nav_stage2/deps/fastlivo_trial_20261022/lib:$go2_root/nav_stage2/deps/fastlio_eval_20261019/lib:$LD_LIBRARY_PATH" "$go2_root/nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping" --ros-args --params-file "$go2_case/livo_live_params.yaml" -r __node:=fastlivo_navigation_trial -r /tf:=/livo/evaluation_tf -r /tf_static:=/livo/evaluation_tf_static -r /aft_mapped_to_init:=/livo/filter_updates -r /LIVO2/imu_propagate:=/lio/odometry > "$go2_case/fastlivo_live.log" 2>&1 &'''
    s='\n'.join(lines)+'\n'
    a='"$go2_lio_dir/lio_nav_odometry.py"';assert s.count(a)==1;s=s.replace(a,'"$go2_lio_dir/../warehouse_livo/livo_nav_odometry.py"')
    r=lio/'record_lio_evaluation.py';t=r.read_text()
    t=t.replace('from sensor_msgs.msg import Imu, PointCloud2, LaserScan','from sensor_msgs.msg import Imu, PointCloud2, LaserScan, Image, CameraInfo')
    t=t.replace('from nav_msgs.msg import Odometry, OccupancyGrid','from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath')
    t=t.replace('TOPICS = {',"TOPICS = {\n    '/plan':NavPath,")
    t=t.replace('TOPICS = {',"TOPICS = {\n    '/livo/filter_updates':Odometry,\n    '/livo/image_raw':Image,\n    '/livo/camera_info':CameraInfo,")
    t=t.replace('reliable_inputs = {',"reliable_inputs = {'/livo/image_raw','/livo/camera_info',")
    t=t.replace('depth=20 if kind is PointCloud2 else 1000','depth=20 if kind in (PointCloud2, Image) else 1000');r.write_text(t)
p.write_text(s)
if args.profile=='fast-video':
    p=stage2/'navigate_through_poses.xml';t=p.read_text();p.write_text(t.replace('radius="0.7"','radius="1.2"'))
    p=stage2/'velocity_guard.py';t=p.read_text();assert t.count('min(values[0], .8)')==1
    p.write_text(t.replace('min(values[0], .8)','min(values[0], 1.7)'))
    p=stage2/'nav_obstacle_region_arrival.yaml';t=p.read_text()
    t=t.replace('max_vel_x: 0.8','max_vel_x: 1.7').replace('max_speed_xy: 0.8','max_speed_xy: 1.7')
    t=t.replace('debug_trajectory_details: true','debug_trajectory_details: true\n      forward_prune_distance: 4.0')
    t=t.replace('width: 6','width: 8').replace('height: 6','height: 8')
    t=t.replace('sim_time: 2.0','sim_time: 1.5').replace('GoalDist.scale: 24.0','GoalDist.scale: 48.0')
    t=t.replace('BaseObstacle.scale: 0.20','BaseObstacle.scale: 0.50').replace('inflation_radius: 1.00','inflation_radius: 1.20');p.write_text(t)
    p=stage2/'nav_obstacle_plan_safe.yaml';t=p.read_text();p.write_text(t.replace('inflation_radius: 1.00','inflation_radius: 1.20'))
    p=livo/'check_moving_blockage.py';t=p.read_text()
    t=t.replace("pose['x']>=1.","pose['x']>=-3.6").replace("pose['x']>1.8","pose['x']>-2.2")
    t=t.replace("abs(pose['y'])<.8","abs(pose['y'])<1.7")
    t=t.replace("prelude_goals += [[-2.,0.],config['goal_xy']]","prelude_goals += [[-2.,-1.3],config['goal_xy']]")
    t=t.replace("c['vx']<=.80001","c['vx']<=1.70001");p.write_text(t)
config=json.loads((root/'artifacts/lio_nav_20261021_noise_long_05/CASE.json').read_text())
# Deployment uses the native accelerometer/gyro; keep old noisy captures immutable.
config['trial_profile']=args.profile
config['configured_max_vx_mps']=1.7 if args.profile=='fast-video' else .8
config['imu_noise_enabled']=False
config['imu_mode']='native_accelerometer_gyro; artificial_noise_disabled'
config.update(runtime=runtime.name,spawn=[1.,0.,0.],odom_anchor=[1.,0.,.4],map=str(mapdir/'map.yaml'),route=None,
              kind=args.kind,trial_route=args.route,goal=[6.,0.],scene='warehouse plus declared controlled prior/dynamic fixture',
              dynamic_obstacles='inserted only after estimated actual movement',chosen_estimator=selection['chosen'])
config.pop('planned_route_m',None)
if use_livo:
    config.pop('fastlio_binary',None);config.pop('fastlio_binary_sha256',None)
    config.update(rgb_enabled=True,livo_used_for_control=True,estimator_output='native LIVO IMU propagation with LiDAR/visual corrections',
                  estimator_binary=str(root/'nav_stage2/deps/fastlivo_trial_20261022/bin/fastlivo_mapping'))
(case/'CASE.json').write_text(json.dumps(config,indent=2)+'\n')
(case/'initial_pose.json').write_text(json.dumps(dict(source='CONFIGURED_CAD_scan_projection_known_spawn',x=1.2,y=0.,yaw=0.))+'\n')
shutil.copytree(runtime,case/'runtime_source',ignore=shutil.ignore_patterns('__pycache__','outputs','.git'))
files=sorted(p for base in [case/'source',case/'runtime_source'] for p in base.rglob('*') if p.is_file())
(case/'source_files.sha256').write_text(''.join(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(case))+'\n' for p in files))
print(case)
