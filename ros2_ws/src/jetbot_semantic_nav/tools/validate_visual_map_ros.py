"""Domain52 protocol/preview fixtures, not camera recognition or Isaac acceptance."""
import argparse,json,os,tempfile,threading,time,hashlib
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='52':raise RuntimeError('Isolated domain52 required')
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from validate_ros_failures import Fixture
from jetbot_semantic_nav.vision_mapper import VisionMapper
from jetbot_semantic_nav.manager import Manager
p=argparse.ArgumentParser();p.add_argument('--config',required=True,type=Path);p.add_argument('--model',required=True,type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
assert not a.output.exists()
c=json.loads(a.config.read_text());s=c['semantic'];s.update(mode='vision',object_geometry='measured_point',fixtures=[],registered_groups=[],registered_prims=[],vision_upstream_commit='a3bbe944a99b72e5b93e1ef6024125af4e586b73',ready_after_sim_s=1.,healthy_ready_wall_s=.1,object_ttl_s=3.,visual_map_policy=dict(active_s=.5,memory_sim_s=2.,memory_wall_s=2.))
s['topics']['observations']='/perception/objects_2d';s['topics']['objects']='/semantic/objects/map_2d';s.pop('speed_policy',None)
s['object_grounding']=dict(backend='pretrained_chinese_encoder',checkpoint=str(a.model/'grounding.pt'),checkpoint_sha256=hashlib.sha256((a.model/'grounding.pt').read_bytes()).hexdigest(),min_confidence=.8,min_margin=.2)
root=Path(tempfile.mkdtemp(prefix='visual_map_fixture_'));(root/'CASE.json').write_text(json.dumps(dict(simulation_only=True,operator_enabled=True,navigation=c,motion_limits=dict(max_linear_m_s=.8,scan_clearance_m=.4))))
(root/'GATE_PROGRESS.json').write_text(json.dumps(dict(fault=None,stream_wall_ages=dict(clock=.01,odom=.01,scan=.01))))
class VisualFixture(Fixture):
 def tick(self):
  self.stamp+=.05;msg=Clock();msg.clock.sec=int(self.stamp);msg.clock.nanosec=round((self.stamp-int(self.stamp))*1e9);self.clock.publish(msg)
  if self.publish_objects:
   stamp=round(self.stamp,6);obj=dict(object_id='vision:fixture:1',track_id=1,class_name='box',confidence=.95,position_xy=[5.,6.],position_reference='visible_surface',range_m=5.,localization_method='depth')
   self.objects.publish(String(data=json.dumps(dict(schema='atongmu.planar_observations',version=1,frame_id='map',source='vision',session_id='fixture',sequence=int(round(self.stamp*100)),stamp=stamp,objects=[obj]))))
rclpy.init();f=VisualFixture(s);f.free=True;mapper=VisionMapper(root);manager=Manager(root);ex=MultiThreadedExecutor(num_threads=6)
for n in (f,mapper,manager):ex.add_node(n)
t=threading.Thread(target=ex.spin);t.start();checks=[];result={}
def until(fn,label):
 end=time.monotonic()+35
 while time.monotonic()<end:
  if fn():return
  time.sleep(.03)
 raise AssertionError(label)
def run(task,text,expected,dry=True):
 until(lambda:manager.active_id is None,'idle')
 f.commands.publish(String(data=json.dumps(dict(version=1,task_id=task,text=text,dry_run=dry))))
 until(lambda:any(r.get('task_id')==task and r['state']==expected for r in f.states),task+': '+expected)
 return json.loads((root/'semantic_tasks'/(task+'.json')).read_text())
try:
 until(lambda:manager.ready,'READY')
 r=run('point_preview','找到最近的纸箱','DRY_RUN_COMPLETE');assert r['selected_object']['source']=='vision';assert 'dimensions'not in r['selected_object'];assert r['goal'][0]<5.;checks.append('vision_points_to_real_manager_preview_without_fabricated_box')
 run('missing_row','到第二排货架旁的托盘','REFERENCE_NOT_FOUND');checks.append('row_identity_not_invented_from_usd')
 run('execution_closed','找到最近的纸箱','MODEL_NOT_QUALIFIED',False);assert not f.requests and not any(f.enables);checks.append('preview_never_enables_gate_or_sends_navigation')
 f.publish_objects=False;until(lambda:not manager.active_objects(),'wall_and_sim_expiry');checks.append('stopped_visual_stream_expires_targets')
 until(lambda:(root/'SEMANTIC_MAP.json').exists() and any(o.get('status')=='stale'for o in json.loads((root/'SEMANTIC_MAP.json').read_text())['objects']),'historical map retains stale evidence')
 assert json.loads((root/'SEMANTIC_MAP.json').read_text())['navigation_input'] is False
 checks.append('historical_map_retained_without_reactivating_stale_target')
 result=dict(passed=True,checks=checks,case=str(root),ros_domain_id=52,native_navigation_goals=len(f.requests),gate_enables=sum(f.enables),camera_inference_tested=False,Isaac_navigation_tested=False)
except Exception as e:result=dict(passed=False,error=repr(e),checks=checks,case=str(root));raise
finally:
 a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2));manager.stopping.set()
 for w in (manager.startup,manager.worker):
  if w:w.join(timeout=8)
 ex.shutdown();t.join(timeout=3)
 mapper.log.close()
 for n in (manager,mapper,f):n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
print(json.dumps(result,ensure_ascii=False))
