"""Create a new, unpublished showcase case; frozen successful trials stay untouched."""
import argparse,ast,hashlib,json,shutil,subprocess,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--case',required=True);p.add_argument('--speed-cap',type=float,default=1.0);a=p.parse_args()
if not .99<=a.speed_cap<=1.7:raise SystemExit('Supported range 1.0..1.7')
r=Path('/home/admin/go2-warehouse');here=Path(__file__).resolve().parent
outer=[[6.7,10.],[6.5,12.],[2.,12.],[2.,15.],[-2.,15.],[-4.,13.],[-6.5,10.],[-6.5,-6.],[-4.,-10.],[2.,-11.]]
from validate_map_points import validate_points
preflight=validate_points(r/'nav_stage2/maps/moving_B_20261022/map.yaml',outer+[[-2.,-1.3],[4.,-11.]])
subprocess.run([sys.executable,str(here.parent/'warehouse_livo/prepare_dynamic_trial.py'),'--case',a.case,'--kind','B','--route','long','--selection',str(r/'artifacts/livo_selection_20261022_v2.json'),'--profile','fast-video'],check=True)
c=r/'artifacts'/('moving_'+a.case);s=c/'source';stage=s/'warehouse_stage2';livo=s/'warehouse_livo';lio=s/'warehouse_lio'
shutil.copytree(here,s/'warehouse_showcase',ignore=shutil.ignore_patterns('__pycache__'))
# Versioned runtime specific to this case; do not patch any prior runtime in place.
config=json.loads((c/'CASE.json').read_text());runtime=r/('motion_showcase_'+a.case)
shutil.copytree(r/config['runtime'],runtime,ignore=shutil.ignore_patterns('__pycache__','.git','outputs'))
for n in ['dynamic_fixture.py','follow_video_capture.py']:shutil.copy2(here/n,runtime/n)
p=runtime/'go2/go2_env.py';t=p.read_text();assert t.count('debug_vis=True')==1;p.write_text(t.replace('debug_vis=True','debug_vis=False'))
p=runtime/'ros2/go2_ros2_bridge.py';t=p.read_text();assert 'np.clip(values[0], 0.0, 1.7)' in t;p.write_text(t.replace('np.clip(values[0], 0.0, 1.7)',f'np.clip(values[0], 0.0, {a.speed_cap})'))
config.update(runtime=runtime.name,trial_profile='unpublished_showcase',configured_max_vx_mps=a.speed_cap,goal=[4.,-11.],showcase_events=['A_cart','A_pallet','B_rack'],route_scope='first warehouse loop plus long outer section after B; distinct final goal')
(c/'CASE.json').write_text(json.dumps(config,indent=2)+'\n')
# Restore the demonstrated complete-video turn settings, rather than the failed pruning experiments.
p=stage/'navigate_through_poses.xml';p.write_text(p.read_text().replace('radius="1.2"','radius="0.7"'))
p=stage/'nav_obstacle_region_arrival.yaml';t=p.read_text().replace('forward_prune_distance: 4.0','forward_prune_distance: 2.0');t=t.replace('max_vel_x: 1.7',f'max_vel_x: {a.speed_cap}').replace('max_speed_xy: 1.7',f'max_speed_xy: {a.speed_cap}');t=t.replace('min_vel_x: 0.0','min_vel_x: 0.30');t=t.replace('debug_trajectory_details: true','publish_local_plan: true\n      publish_global_plan: true\n      debug_trajectory_details: true');p.write_text(t)
p=stage/'velocity_guard.py';p.write_text(p.read_text().replace('min(values[0], 1.7)',f'min(values[0], {a.speed_cap})'))
from sim_scan_watchdog import patch_guard
tested=r/'artifacts/showcase_scan_watchdog_20261022_01'
if not (tested/'validation/RESULT.json').exists():tested=here/'validation/scan_watchdog'
proof=json.loads((tested/'validation/RESULT.json').read_text())
if not proof.get('passed'):raise SystemExit('Scan watchdog integration check required')
patched=patch_guard(p.read_text())
if hashlib.sha256(patched.encode()).hexdigest()!=hashlib.sha256((tested/'patched_guard.py').read_bytes()).hexdigest():raise SystemExit('Prepared guard differs from integration-tested source')
p.write_text(patched)
shutil.copytree(tested,c/'watchdog_validation')
config=json.loads((c/'CASE.json').read_text());config['scan_watchdog_profile']=dict(simulation_age_limit_s=.20,wall_fallback_limit_s=1.0,other_stream_deadlines_unchanged=True,scope='simulation only; no timestamp rewriting')
(c/'CASE.json').write_text(json.dumps(config,indent=2)+'\n')

route=json.loads((c/'prelude_route.json').read_text());lap=route['goals'];
route.update(goals=lap+[dict(xy=[-2.,-1.3]),dict(xy=[6.,0.])]+[dict(xy=g) for g in outer],execution='native single ThroughPoses; two laps with first-lap in-motion A/A/B',scope='unpublished demo; no navigation truth')
assert all(sum((v-z)**2 for v,z in zip(g['xy'],[4.,-11.]))>.8**2 for g in route['goals']), 'Final goal must not be an intermediate waypoint'
(c/'prelude_route.json').write_text(json.dumps(route,indent=2)+'\n')
(c/'ROUTE_PREFLIGHT.json').write_text(json.dumps(preflight,indent=2)+'\n')
dyn=json.loads((c/'simulation/dynamic_config.json').read_text())
dyn['goal_xy']=[4.,-11.]
dyn['obstacles']=[dict(center=[-4.,4.,.55],size=[.95,1.10,1.10],appearance='cart'),dict(center=[4.,5.5,.55],size=[1.,1.1,1.1],appearance='pallet'),dict(center=[4.3,1.4,.75],size=[.6,5.,1.5],appearance='rack')]
dyn['showcase_events']=[dict(name='A_cart',kind='A',active=[0],minimum_travel_m=15.,window=[-5.,-3.,0.,1.],velocity_axis='vy',velocity_sign=1),dict(name='A_pallet',kind='A',active=[0,1],minimum_travel_m=40.,window=[3.,5.,9.,11.],velocity_axis='vy',velocity_sign=-1),dict(name='B_rack',kind='B',active=[0,1,2],minimum_travel_m=60.,window=[-3.6,-2.2,-1.7,1.0],velocity_axis='vx',velocity_sign=1)]
(c/'simulation/dynamic_config.json').write_text(json.dumps(dyn,indent=2)+'\n')
# Freeze a derivative of the proven observer with an explicit multi-event trigger.
t=(here.parent/'warehouse_livo/check_moving_blockage.py').read_text()
t=t.replace('def request(fixture,identity,enabled):','def request(fixture,identity,enabled,active):').replace('dict(id=identity,enabled=enabled)','dict(id=identity,enabled=enabled,active=active)')
t=t.replace("prelude_goals += [[-2.,0.],config['goal_xy']]","prelude_goals += [config['goal_xy']]")
t=t.replace("start=time.monotonic();trigger_wall=None","start=time.monotonic();trigger_wall=None;event_index=0;pending_event=None;events=config['showcase_events'];result['events']=[]")
start=t.index('                if final and armed and not injected:');end=t.index("            leg['status']=future.result().status",start)
t=t[:start]+'''                if armed and event_index<len(events) and pending_event is None:
                    event=events[event_index];pose=p.map_pose();recent=p.rows[-10:]
                    x0,x1,y0,y1=event['window']
                    moving=np.mean([v['speed'] for v in recent])>.2
                    direction=(math.sin(pose['yaw']) if event['velocity_axis']=='vy' else math.cos(pose['yaw']))*event['velocity_sign']>.7
                    if travelled>=event['minimum_travel_m'] and moving and direction and x0<=pose['x']<=x1 and y0<=pose['y']<=y1:
                        wall=time.monotonic()
                        pending_event=dict(name=event['name'],kind=event['kind'],wall=wall,estimated_map_pose=pose,estimated_motion=recent,displacement_m=travelled,displacement_basis='cumulative estimated odom',active=event['active'])
                        request(args.fixture,event['name'],True,event['active']);injected=True
                        if trigger_wall is None:trigger_wall=wall;result['trigger']=pending_event.copy()
                if pending_event is not None:
                    state=json.loads((args.fixture/'obstacle_state.json').read_text())
                    if state['id']==pending_event['name'] and state['enabled']:
                        ack=state;pending_event['applied']=state;result['events'].append(pending_event);event_index+=1;pending_event=None
                        result['applied_obstacle']=ack
                        (args.output/'EVENTS.json').write_text(json.dumps(result['events'],indent=2)+'\\n')
                        (args.output/'APPLIED.json').write_text(json.dumps(ack,indent=2)+'\\n')
''' + t[end:]
t=t.replace("            if service.wait_for_service(timeout_sec=1) and p.wait(service.call_async(GetState.Request()),3).current_state.id==3:active=True;break", """            if service.wait_for_service(timeout_sec=1):
                try:
                    if p.wait(service.call_async(GetState.Request()),5).current_state.id==3:active=True;break
                except RuntimeError as exc:
                    if str(exc)!='Async request timeout':raise
            p.sample()""")
t=t.replace("0<=c['vx']<=.80001",f"0<=c['vx']<={a.speed_cap+.00001}")
t=t.replace("        result.update(checks=checks", "        checks['all_showcase_events_acknowledged']=event_index==len(events) and pending_event is None\n        checks['native_plan_after_each_event']=len(result['events'])==len(events) and all(any(v['wall']>e['wall'] for v in p.plans) for e in result['events'])\n        result.update(checks=checks")
ast.parse(t);(livo/'check_moving_blockage.py').write_text(t)
# Read-only path tap supervised with the existing child process cleanup.
# Display run: avoid a second reliable full-LiDAR subscriber doing bulk compression.
# Estimator/scan subscribers and all live sensor data remain unchanged.
r=lio/'record_lio_evaluation.py';t=r.read_text();needle='        topics = dict(TOPICS)';assert t.count(needle)==1
t=t.replace(needle,needle+"\n        topics = {topic:kind for topic,kind in topics.items() if kind is not PointCloud2}  # showcase lean disk capture")
t=t.replace('Evaluation-only sensor CDR capture for identical-input offline comparisons.','Showcase telemetry capture; bulk PointCloud2 excluded from disk, not from the estimator.')
r.write_text(t)
config=json.loads((c/'CASE.json').read_text());config['recording_scope']='native IMU/RGB/odom/scan/costmaps/plan; no duplicate bulk PointCloud2 recording; live estimator inputs unchanged';(c/'CASE.json').write_text(json.dumps(config,indent=2)+'\n')
p=lio/'run_nav_trial.sh';t=p.read_text();needle='  go2_record_args=()';assert t.count(needle)==1
t=t.replace(needle,'''  setsid "$GO2_ISAAC_ROOT/python.sh" "$go2_case/source/warehouse_showcase/live_composite_preview.py" --case "$go2_case" > "$go2_case/live_preview.log" 2>&1 &
  go2_children+=("$!")
  setsid "$GO2_ISAAC_ROOT/python.sh" "$go2_case/source/warehouse_showcase/record_display_paths.py" --output "$go2_case/simulation/navigation_paths.json" > "$go2_case/display_paths.log" 2>&1 &
  go2_children+=("$!")
'''+needle);p.write_text(t)
shutil.rmtree(c/'runtime_source');shutil.copytree(runtime,c/'runtime_source',ignore=shutil.ignore_patterns('__pycache__','.git','outputs'))
files=sorted(f for base in [s,c/'runtime_source'] for f in base.rglob('*') if f.is_file())
(c/'source_files.sha256').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(c))+'\n' for f in files))
print(c)
