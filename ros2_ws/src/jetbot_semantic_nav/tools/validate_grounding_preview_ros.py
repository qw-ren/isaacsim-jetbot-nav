"""Real learned model + real mapper/manager, fake Nav2, domain52, preview only."""
import argparse,copy,hashlib,json,os,tempfile,threading,time
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='52':raise RuntimeError('Requires isolated domain52')
import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from validate_ros_failures import Fixture
from jetbot_semantic_nav.mapper import Mapper
from jetbot_semantic_nav.manager import Manager
p=argparse.ArgumentParser();p.add_argument('--min-confidence',type=float,default=.9);p.add_argument('--config',required=True,type=Path);p.add_argument('--model',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
cfg=json.loads(a.config.read_text());semantic=cfg['semantic'];semantic.pop('speed_policy',None);semantic['ready_after_sim_s']=1.;semantic['healthy_ready_wall_s']=.3
world=semantic['world_to_map'];semantic['object_grounding']=dict(backend='pretrained_chinese_encoder',checkpoint=str(a.model/'grounding.pt'),checkpoint_sha256=hashlib.sha256((a.model/'grounding.pt').read_bytes()).hexdigest(),min_confidence=a.min_confidence,min_margin=.2,bindings=dict(version=1,frame_id='map',scene_id=world['source_scene_sha256'],world_to_map=world,objects=[dict(object_id='fixture/box',goal=[3.,6.,0.],object_position=[5.,6.,.3],object_dimensions=[.7,.5,.6],object_orientation_xyzw=[0.,0.,0.,1.])]))
root=Path(tempfile.mkdtemp(prefix='grounding_preview_fixture_'));(root/'CASE.json').write_text(json.dumps(dict(simulation_only=True,navigation=cfg,motion_limits=dict(max_linear_m_s=cfg['max_linear_m_s'],scan_clearance_m=.4))))
(root/'GATE_PROGRESS.json').write_text(json.dumps(dict(fault=None,reason='DISARMED',output=[0.,0.],stream_wall_ages=dict(clock=.01,odom=.01,scan=.01))))
rclpy.init();fixture=Fixture(semantic);fixture.free=True;mapper=Mapper(root);manager=Manager(root);executor=MultiThreadedExecutor(num_threads=6)
for node in (fixture,mapper,manager):executor.add_node(node)
thread=threading.Thread(target=executor.spin);thread.start();checks=[]
def until(predicate,label):
 end=time.monotonic()+30
 while time.monotonic()<end:
  if predicate():return
  time.sleep(.03)
 raise AssertionError('Timeout '+label)
def command(task,dry=True):
 until(lambda:manager.active_id is None,'idle')
 fixture.commands.publish(String(data=json.dumps(dict(version=1,task_id=task,text='找到最近的纸箱',dry_run=dry))))
def state(task,label):return any(r.get('task_id')==task and r['state']==label for r in fixture.states)
try:
 until(lambda:manager.ready,'ready')
 command('live_refused',False);until(lambda:state('live_refused','MODEL_NOT_QUALIFIED'),'live refusal');assert not fixture.requests and not any(fixture.enables);checks.append('unqualified_model_cannot_navigate_or_enable')
 command('bound_preview');until(lambda:state('bound_preview','DRY_RUN_COMPLETE'),'bound preview');r=json.loads((root/'semantic_tasks/bound_preview.json').read_text());assert r['goal']==[3.,6.,0.] and r['selected_object']['object_id']=='fixture/box';assert not fixture.requests and not any(fixture.enables);checks.append('real_model_id_to_fixed_pose_preview_without_navigation')
 fixture.x=5.3;until(lambda:any(r['position'][0]==5.3 for r in manager.active_objects()),'moved snapshot');command('stale_binding');until(lambda:state('stale_binding','FAILED'),'stale refusal');assert 'BINDING_STALE_POSITION' in json.loads((root/'semantic_tasks/stale_binding.json').read_text())['error'];checks.append('stale_binding_rejected')
 fixture.x=5.;until(lambda:any(r['position'][0]==5. for r in manager.active_objects()),'restored snapshot');fixture.free=False;command('blocked_binding');until(lambda:state('blocked_binding','UNREACHABLE'),'blocked goal');assert not fixture.requests and not any(fixture.enables);checks.append('blocked_fixed_pose_no_fallback_or_navigation')
 result=dict(passed=True,checks=checks,ros_domain_id=52,case=str(root),native_navigation_goals=len(fixture.requests),gate_enable_count=sum(fixture.enables),states=fixture.states,not_Isaac_navigation_acceptance=True)
except Exception as exc:
 result=dict(passed=False,error=repr(exc),checks=checks,states=fixture.states,case=str(root));raise
finally:
 a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2))
 manager.stopping.set()
 for worker in (manager.startup,manager.worker):
  if worker:worker.join(timeout=8)
 executor.shutdown();thread.join(timeout=3)
 for node in (manager,mapper,fixture):node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
print(json.dumps(dict(passed=result['passed'],checks=checks)))
