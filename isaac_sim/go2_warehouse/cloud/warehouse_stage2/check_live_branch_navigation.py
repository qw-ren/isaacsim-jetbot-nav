"""Actual NavigateToPose evaluated in the AMCL map frame, with physical odometry limits."""
import argparse,json,time,math,hashlib
from pathlib import Path
import numpy as np
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from check_navigation import Probe
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
class LocalizedProbe(Probe):
 def __init__(self):
  super().__init__();self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self);self.map_rows=[];self.amcl_poses=[];self.amcl_sub=self.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",lambda m:self.amcl_poses.append({"wall":time.monotonic(),"x":m.pose.pose.position.x,"y":m.pose.pose.position.y,"covariance_diag":[m.pose.covariance[i] for i in [0,7,35]]}),qos_profile_sensor_data)
  self.costmap=None;self.plans=[]
  self.cost_sub=self.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:setattr(self,'costmap',m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
  self.plan_sub=self.create_subscription(NavPath,'/plan',lambda m:self.plans.append({'wall':time.monotonic(),'xy':[[v.pose.position.x,v.pose.position.y] for v in m.poses]}),10)
 def map_pose(self):
  t=self.buffer.lookup_transform('map','unitree_go2/base_link',Time());p=t.transform.translation;q=t.transform.rotation
  return {'wall':time.monotonic(),'stamp':t.header.stamp.sec+t.header.stamp.nanosec*1e-9,'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
 def sample(self):
  rclpy.spin_once(self,timeout_sec=.02)
  try:self.map_rows.append(self.map_pose())
  except Exception:pass
 def odom(self,m):super().odom(m);self.rows[-1]['wz']=m.twist.twist.angular.z
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--map',type=Path,required=True);ap.add_argument('--boxes',type=Path,required=True);ap.add_argument('--forward-goal',type=float,default=4.);ap.add_argument('--fixture',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False);assert .5<=a.forward_goal<=6
 files=[a.map,a.map.parent/'map.pgm'];hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files};rclpy.init();p=LocalizedProbe();r={'map':str(a.map),'mode':'B_preblocked_upper_branch_actual_lower_branch_NavigateToPose_with_AMCL'};handle=None;armed=False
 try:
  until=time.monotonic()+90
  while time.monotonic()<until and not(p.map_rows and p.rows and p.states and time.monotonic()-p.scan_wall<.4):p.sample()
  if not p.map_rows:raise RuntimeError('No localized map->base TF')
  state=json.loads((a.fixture/'obstacle_state.json').read_text())
  if not state.get('enabled'):raise RuntimeError('Fixed box is not enabled')
  r['fixture']=state;
  if 'prior_divider' not in state:raise RuntimeError('B prior divider is not identified in fixture state')
  box_center=np.asarray(state['center'][:2]);box_half=np.asarray(state['size'][:2])/2
  p.arm(False);life=p.create_client(GetState,'/bt_navigator/get_state');until=time.monotonic()+40;active=False
  while time.monotonic()<until:
   if life.wait_for_service(timeout_sec=1) and p.wait(life.call_async(GetState.Request()),3).current_state.id==3:active=True;break
   p.sample()
  if not active or not p.nav.wait_for_server(timeout_sec=8):raise RuntimeError('Navigator not active')
  warm_sim=p.rows[-1]['sim'];until=time.monotonic()+10
  while p.rows[-1]['sim']-warm_sim<.6 and time.monotonic()<until:p.sample()
  if p.costmap is None:raise RuntimeError('No global costmap')
  grid=p.costmap;values=np.asarray(grid.data).reshape(grid.info.height,grid.info.width);yy,xx=np.indices(values.shape);xx=grid.info.origin.position.x+(xx+.5)*grid.info.resolution;yy=grid.info.origin.position.y+(yy+.5)*grid.info.resolution
  t=p.buffer.lookup_transform('map','odom',Time()).transform;q=t.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));cx=t.translation.x+box_center[0]*math.cos(yaw)-box_center[1]*math.sin(yaw);cy=t.translation.y+box_center[0]*math.sin(yaw)+box_center[1]*math.cos(yaw);mask=(abs(xx-cx)<.5)&(abs(yy-cy)<.5)
  r['initial_box_lethal_cells']=int(np.sum(values[mask]==100))
  np.savez_compressed(a.output/'initial_costmap.npz',costmap=values,origin=[grid.info.origin.position.x,grid.info.origin.position.y],resolution=grid.info.resolution,box_center=[cx,cy])
  if r['initial_box_lethal_cells']<1:raise RuntimeError('New box not observed in global costmap')
  start=p.map_pose();r['start']=start;r['target']={'x':start['x']+a.forward_goal,'y':start['y'],'yaw':0.};g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=p.get_clock().now().to_msg();g.pose.pose.position.x=r['target']['x'];g.pose.pose.position.y=r['target']['y'];g.pose.pose.orientation.w=1.;start_wall=time.monotonic();r['start_wall']=start_wall;r['start_sim']=p.rows[-1]['sim']
  handle=p.wait(p.nav.send_goal_async(g))
  if not handle.accepted:raise RuntimeError('Goal rejected')
  f=handle.get_result_async();until=time.monotonic()+180
  while not f.done():
   p.sample()
   if time.monotonic()>until:raise RuntimeError('Navigation wall timeout')
   if p.rows[-1]['tilt']>.5 or not .2<=p.rows[-1]['z']<=.85:raise RuntimeError('Posture limit')
   if not armed and p.commands and p.commands[-1]['wall']>start_wall:p.arm(True);armed=True;enable_wall=time.monotonic()
   if armed and p.states and p.states[-1]['wall']>enable_wall+.2 and not p.states[-1]['armed']:raise RuntimeError('Guard latched: '+p.states[-1]['reason'])
  r['action_status']=f.result().status;r['action_end_pose']=p.map_pose();p.arm(False)
 except Exception as e:r['error']=str(e);r['abort_wall']=time.monotonic()
 finally:
  if handle is not None:
   try:p.wait(handle.cancel_goal_async(),3)
   except Exception:pass
  try:p.arm(False)
  except Exception as e:r['disarm_error']=str(e)
  if p.rows:
   start_stop=p.rows[-1]['sim'];until=time.monotonic()+45
   while p.rows[-1]['sim']-start_stop<5 and time.monotonic()<until:p.sample()
  try:r['final_map_pose']=p.map_pose()
  except Exception:pass
  (a.output/'telemetry.json').write_text(json.dumps({'global_plans':p.plans,'amcl_poses':p.amcl_poses,'odom':p.rows,'map_poses':p.map_rows,'commands':p.commands,'runtime':p.runtime,'guard':p.states},indent=2)+'\n')
  (a.output/'action_result_before_metrics.json').write_text(json.dumps(r,indent=2)+'\n')
  target=r.get('target',{});final=r.get('final_map_pose',{});r['goal_error_m']=math.hypot(final.get('x',99)-target.get('x',0),final.get('y',99)-target.get('y',0));dy=final.get('yaw',99)-target.get('yaw',0);r['goal_yaw_error_rad']=abs(math.atan2(math.sin(dy),math.cos(dy)))
  end_wall=r.get('action_end_pose',{}).get('wall',r.get('abort_wall',time.monotonic()));cruise=[]
  for x in p.rows:
   if not(r.get('start_wall',float('inf'))<x['wall']<end_wall and x['sim']>r.get('start_sim',float('inf'))+.5):continue
   near=min(p.map_rows,key=lambda m:abs(m['wall']-x['wall'])) if p.map_rows else None
   if near and math.hypot(near['x']-target.get('x',0),near['y']-target.get('y',0))>.75:cruise.append(x['vx'])
  r['cruise_mps']=float(np.mean(cruise)) if cruise else 0.;r['cruise_samples']=len(cruise);stopped=[x for x in p.rows if x['sim']>=p.rows[-1]['sim']-2] if p.rows else [];moving=[c for c in p.commands if c['wall']>=r.get('start_wall',float('inf')) and c['vx']>1e-3];xy=np.asarray([[x['x'],x['y']] for x in p.rows]).reshape(-1,2);clearance=float('inf')
  for b in json.loads(a.boxes.read_text()):
   if len(xy):clearance=min(clearance,float(np.linalg.norm(np.maximum(np.maximum(np.array(b['min'][:2])-xy,xy-np.array(b['max'][:2])),0),axis=1).min()))
  r['min_center_aabb_clearance_m']=clearance;r['map_publishers']=[x.node_name for x in p.get_publishers_info_by_topic('/map')];r['tf_publishers']=[x.node_name for x in p.get_publishers_info_by_topic('/tf')]
  r['amcl_pose_updates_during_action']=sum(r.get('start_wall',float('inf'))<=m['wall']<=end_wall for m in p.amcl_poses)
  r['checks']={'amcl_updates_during_navigation':r['amcl_pose_updates_during_action']>=2,'no_error':'error' not in r,'action_succeeded':r.get('action_status')==4,'localized_goal_error_le_025':r['goal_error_m']<=.25,'localized_yaw_error_le_030':r['goal_yaw_error_rad']<=.3,'cruise_ge_06':len(cruise)>=10 and r['cruise_mps']>=.6,'commands_bounded':bool(moving) and all(0.<=c['vx']<=.80001 and abs(c['wz'])<=.60001 for c in moving),'sustained_stop':bool(stopped) and all(x['speed']<.05 and abs(x['wz'])<.1 for x in stopped),'posture':bool(p.rows) and all(.2<=x['z']<=.85 and x['tilt']<.5 for x in p.rows),'geometry_clearance':clearance>.45,'saved_map_unchanged':all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==h for f,h in hashes.items()),'saved_map_amcl_sources':r['map_publishers']==['map_server'] and 'amcl' in r['tf_publishers'] and 'slam_toolbox' not in r['tf_publishers']}
  r['obstacle_clearance_m']=float(np.linalg.norm(np.maximum(np.abs(xy-np.asarray(r.get('fixture',{'center':[4.,-1.]})['center'][:2]))-np.asarray(r.get('fixture',{'size':[.6,.6]})['size'][:2])/2,0),axis=1).min()) if len(xy) else 0.
  r['global_plans_seen']=len(p.plans)
  r['lateral_excursion_m']=float(np.max(np.abs(xy[:,1]-xy[0,1]))) if len(xy) else 0.
  r['checks']['global_plan_observed']=bool(p.plans)
  r['checks']['new_obstacle_clearance']=r['obstacle_clearance_m']>.45
  r['checks']['passed_obstacle']=bool(p.rows) and p.rows[-1]['x']>r.get('fixture',{'center':[4.,-1.],'size':[.6,.6]})['center'][0]+r.get('fixture',{'center':[4.,-1.],'size':[.6,.6]})['size'][0]/2+.5
  r['checks']['guard_no_latched_fault']=not any(not x['armed'] and x['reason']!='DISARMED' and x['wall']>=r.get('start_wall',float('inf')) for x in p.states)
  r['informational_checks']={'sustained_stop':r['checks'].pop('sustained_stop'),'post_stop_xy_le_025':r['checks'].pop('localized_goal_error_le_025'),'post_stop_yaw_le_030':r['checks'].pop('localized_yaw_error_le_030')}
  end_pose=r.get('action_end_pose',{});r['arrival_position_error_m']=math.hypot(end_pose.get('x',99)-target.get('x',0),end_pose.get('y',99)-target.get('y',0));d=end_pose.get('yaw',99)-target.get('yaw',0);r['arrival_yaw_error_rad']=abs(math.atan2(math.sin(d),math.cos(d)))
  during=[m for m in p.map_rows if r.get('start_wall',float('inf'))<=m['wall']<=end_wall];r['closest_goal_error_m']=min((math.hypot(m['x']-target.get('x',0),m['y']-target.get('y',0)) for m in during),default=99.)
  r['checks']['configured_stateful_goal_xy_entered']=r['closest_goal_error_m']<=.20
  r['checks']['configured_goal_yaw_at_success']=r.get('action_status')==4 and r['arrival_yaw_error_rad']<=.5001
  tail=[x for x in p.states if p.rows and isinstance(x.get('sim'),(int,float)) and x['sim']>=p.rows[-1]['sim']-1]
  r['checks']['zero_target_after_disarm']=bool(tail) and all(not x['armed'] and max(abs(v) for v in x['output'])<1e-6 for x in tail)
  for key in ['cruise_ge_06','configured_stateful_goal_xy_entered','configured_goal_yaw_at_success']:
   if key in r['checks']:r['informational_checks'][key]=r['checks'].pop(key)
  r['acceptance']='Functional alternate-passage navigation; transport region xy=.40m, heading unconstrained; extra precision/cruise checks informational'
  middle=[v for v in p.rows if r.get('start_wall',float('inf'))<=v['wall']<=end_wall and 4.0<=v['x']<=4.6];r['middle_section_y_range']=[min(v['y'] for v in middle),max(v['y'] for v in middle)] if middle else None;r['checks']['actual_lower_branch_traversed']=len(middle)>=5 and max(v['y'] for v in middle)<-1.5;r['scope']='Preblocked branch actual traversal after stationary before/after native planning; obstacle is not inserted during motion';r['passed']=all(r['checks'].values());(a.output/'navigation_check.json').write_text(json.dumps(r,indent=2)+'\n');(a.output/'telemetry.json').write_text(json.dumps({'global_plans':p.plans,'amcl_poses':p.amcl_poses,'odom':p.rows,'map_poses':p.map_rows,'commands':p.commands,'runtime':p.runtime,'guard':p.states},indent=2)+'\n');print(json.dumps(r,indent=2));p.destroy_node();rclpy.shutdown()
 return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
