"""Own bounded simulator/estimator groups; no cloud power operations."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();cfg=json.loads((a.case/'CASE.json').read_text());started=time.monotonic();children=[];handles=[];extension_children=[];error=None
python='/home/admin/isaacsim/python.sh';source=a.case/'source'
def spawn(args,log=None,env=None):
 handle=open(a.case/log,'w')if log else None
 if handle:handles.append(handle)
 process=subprocess.Popen(args,stdout=handle,stderr=subprocess.STDOUT if handle else None,start_new_session=True,env=env);children.append(process);return process
def stop(*_):
 for child in reversed(children):
  if child.poll()is None:os.killpg(child.pid,signal.SIGINT)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
sim_env=os.environ.copy();sim_env.update(cfg.get('visualization',{}).get('environment',{}))
sim=spawn([python,str(source/'run_sim.py'),'--case',str(a.case),'--duration',str(cfg['duration_sim_s'])],env=sim_env);backend=None;navigation_started=False;code=None;navigation_check=None
try:
 while sim.poll()is None:
  if (a.case/'STOP_REQUESTED.json').exists():code=130;break
  if time.monotonic()-started>float(cfg.get('runtime_wall_s',600))+60:raise RuntimeError('Bounded runtime wall timeout')
  if cfg.get('backend_enabled') and backend is None and (a.case/'camera_contract.json').exists() and (a.case/'points_0001.npz').exists():
   subprocess.run([python,str(source/'generate_params.py'),'--case',str(a.case)],check=True)
   environment=os.environ.copy();root=Path('/home/admin/go2-warehouse/nav_stage2/deps');environment['LD_LIBRARY_PATH']=str(root/'fastlivo_trial_20261022/lib')+':'+str(root/'fastlio_eval_20261019/lib')+':'+environment.get('LD_LIBRARY_PATH','')
   backend=spawn([str(root/'fastlivo_trial_20261022/bin/fastlivo_mapping'),'--ros-args','--params-file',str(a.case/'livo_params.yaml'),'-r','__node:=jetbot_fastlivo','-r','/tf:=/jetbot/livo/internal_tf','-r','/tf_static:=/jetbot/livo/internal_tf_static','-r','/aft_mapped_to_init:=/jetbot/livo/filter_updates','-r','/LIVO2/imu_propagate:=/jetbot/livo/imu_propagated'],'fastlivo.log',environment)
   spawn([python,str(source/'nav_odometry.py'),'--case',str(a.case)],'odometry.log')
   spawn([python,str(source/'audit_inputs.py'),'--case',str(a.case)],'input_audit.log')
   if cfg.get('motion_enabled'):spawn([python,str(source/'check_motion.py'),'--case',str(a.case)],'motion.log')
  if cfg.get('nav2_enabled') and not navigation_started and (a.case/'ODOM_ALIGNMENT.json').exists():
   params=str(source/'nav2.yaml')
   def ros(package,executable,name,extra=None):
    return spawn(['ros2','run',package,executable,'--ros-args','--params-file',params,'-r','__node:='+name,*(extra or [])],name+'.log')
   extensions=cfg.get('navigation',{}).get('extensions',{});mux_enabled=extensions.get('command_mux',False);assert type(mux_enabled)is bool
   if mux_enabled:
    assert extensions.get('control_source')in('nav2','ppo')
    extension_children.append(spawn([python,str(source/'command_mux.py'),'--case',str(a.case)],'command_mux.log'))
   if extensions.get('localization_probe',False):extension_children.append(spawn([python,str(source/'localization_probe.py'),'--case',str(a.case)],'localization_probe.log'))
   if extensions.get('probe',False):extension_children.append(spawn([python,str(source/'module_probe.py'),'--case',str(a.case)],'module_probe.log'))
   # The native bridge projects the same timed point packet into chassis scan.
   spawn([python,str(source/'velocity_gate.py'),'--case',str(a.case)],'velocity_gate.log')
   ros('nav2_map_server','map_server','map_server',['-p','yaml_filename:='+str(a.case/'map/map.yaml')])
   if cfg.get('navigation',{}).get('localization_prior_dir'):ros('nav2_map_server','map_server','localization_map_server')
   ros('nav2_amcl','amcl','amcl')
   ros('nav2_planner','planner_server','planner_server')
   ros('nav2_controller','controller_server','controller_server',['-r','/cmd_vel:='+('/jetbot/nav2_proposal'if mux_enabled else'/jetbot/nav_cmd_vel')])
   ros('nav2_bt_navigator','bt_navigator','bt_navigator',['-p','default_nav_to_pose_bt_xml:='+str(source/'navigate.xml'),'-p','default_nav_through_poses_bt_xml:='+str(source/'navigate_through.xml')])
   ros('nav2_lifecycle_manager','lifecycle_manager','jetbot_nav_lifecycle')
   navigation_check=spawn([python,str(source/('operator_session.py'if cfg.get('operator_enabled')else'check_navigation.py')),'--case',str(a.case)],'operator_session.log'if cfg.get('operator_enabled')else'navigation_check.log');navigation_started=True
  if cfg.get('nav2_enabled') and not cfg.get('operator_enabled') and (a.case/'NAVIGATION_RESULT.json').exists():
   result=json.loads((a.case/'NAVIGATION_RESULT.json').read_text());code=0 if result['passed'] else 1
   if navigation_check is not None:
    try:navigation_check.wait(timeout=1)
    except subprocess.TimeoutExpired:pass
   break
  if cfg.get('operator_enabled') and navigation_check is not None and navigation_check.poll()is not None:raise RuntimeError('Operator session exited '+str(navigation_check.returncode))
  for process in extension_children:
   if process.poll()is not None:raise RuntimeError('Extension interface process exited '+str(process.returncode))
  try:
   progress=json.loads((a.case/'PROGRESS.json').read_text())
   if progress.get('command_fault'):raise RuntimeError('Bridge command fault: '+progress['command_fault'])
  except (FileNotFoundError,json.JSONDecodeError):pass
  if backend is not None and backend.poll()is not None:raise RuntimeError('Estimator exited '+str(backend.returncode))
  time.sleep(.2)
 if code is None:code=sim.returncode
except Exception as exc:error=repr(exc);code=1
finally:
 stop()
 for child in reversed(children):
  try:child.wait(timeout=25)
  except subprocess.TimeoutExpired:
   os.killpg(child.pid,signal.SIGTERM)
   try:child.wait(timeout=10)
   except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
 for handle in handles:handle.close()
 sim_record=a.case/'SIM_RESULT.json'
 if sim_record.exists():
  sim_error=json.loads(sim_record.read_text()).get('error')
  if sim_error is not None:
   error='Simulator reported: '+str(sim_error)
   if code!=130:code=1
 elif code==0:error='Simulator exited without SIM_RESULT.json';code=1
 (a.case/'RUNNER_RESULT.json').write_text(json.dumps(dict(exit_code=code,error=error,elapsed_wall_s=time.monotonic()-started,children=[dict(pid=c.pid,exit_code=c.returncode)for c in children],power_operation=False),indent=2)+'\n')
raise SystemExit(code if code>=0 else 128-code)
