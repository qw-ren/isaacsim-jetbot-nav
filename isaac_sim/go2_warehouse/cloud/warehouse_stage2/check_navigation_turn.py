#!/usr/bin/env python3
"""Bounded actual NavigateToPose test. Publishes no velocity."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import Twist,TwistStamped
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

class Probe(Node):
 def __init__(self):
  super().__init__('go2_navigation_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.rows=[];self.commands=[];self.runtime=[];self.states=[];self.scan_wall=0.0
  self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
  self.enable=self.create_client(SetBool,'/nav_m1/enable')
  self.subs=[self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data),self.create_subscription(Odometry,'/unitree_go2/odom',self.odom,qos_profile_sensor_data),self.create_subscription(Twist,'/nav_m1/cmd_vel',self.cmd,10),self.create_subscription(TwistStamped,'/nav_m1/runtime_cmd',self.rt,10),self.create_subscription(String,'/nav_m1/guard_state',self.state,10)]
 def scan(self,m):self.scan_wall=time.monotonic()
 def odom(self,m):
  p,q,v=m.pose.pose.position,m.pose.pose.orientation,m.twist.twist
  self.rows.append(dict(wall=time.monotonic(),sim=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,x=p.x,y=p.y,z=p.z,vx=v.linear.x,speed=math.hypot(v.linear.x,v.linear.y),wz=v.angular.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),tilt=math.acos(float(np.clip(1-2*(q.x*q.x+q.y*q.y),-1,1)))))
 def cmd(self,m):self.commands.append(dict(wall=time.monotonic(),vx=m.linear.x,wz=m.angular.z))
 def rt(self,m):self.runtime.append(dict(wall=time.monotonic(),vx=m.twist.linear.x,wz=m.twist.angular.z))
 def state(self,m):self.states.append(json.loads(m.data))
 def wait(self,f,seconds=10):
  until=time.monotonic()+seconds
  while not f.done() and time.monotonic()<until:rclpy.spin_once(self,timeout_sec=.02)
  if not f.done():raise RuntimeError('Async request timeout')
  return f.result()
 def arm(self,value):
  if not self.enable.wait_for_service(timeout_sec=3):raise RuntimeError('Guard unavailable')
  r=SetBool.Request();r.data=value
  response=self.wait(self.enable.call_async(r))
  if not response.success:raise RuntimeError(response.message)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--boxes',type=Path,required=True);ap.add_argument('--goal-x',type=float,default=6.0);ap.add_argument('--goal-y',type=float,default=-1.0);ap.add_argument('--goal-yaw',type=float,default=0.0);ap.add_argument('--allow-turning',action='store_true');args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
 rclpy.init();p=Probe();result={};handle=None;armed=False
 try:
  until=time.monotonic()+80
  while time.monotonic()<until and not(p.rows and p.states and p.rows[-1]["sim"]>2 and time.monotonic()-p.scan_wall<.4):rclpy.spin_once(p,timeout_sec=.1)
  if not p.rows or time.monotonic()-p.scan_wall>=.4:raise RuntimeError('No fresh odometry/scan')
  p.arm(False)
  lifecycle=p.create_client(GetState,'/bt_navigator/get_state')
  until=time.monotonic()+30
  active=False
  while time.monotonic()<until:
   if lifecycle.wait_for_service(timeout_sec=1):
    if p.wait(lifecycle.call_async(GetState.Request()),3).current_state.id==3:active=True;break
   rclpy.spin_once(p,timeout_sec=.1)
  if not active:raise RuntimeError('BT navigator not active')
  if not p.nav.wait_for_server(timeout_sec=10):raise RuntimeError('NavigateToPose unavailable')
  goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=p.get_clock().now().to_msg();goal.pose.pose.position.x=args.goal_x;goal.pose.pose.position.y=args.goal_y;goal.pose.pose.orientation.w=math.cos(args.goal_yaw/2);goal.pose.pose.orientation.z=math.sin(args.goal_yaw/2)
  result['goal']={'x':args.goal_x,'y':args.goal_y,'yaw':args.goal_yaw};result['start']=dict(p.rows[-1]);start=time.monotonic();handle=p.wait(p.nav.send_goal_async(goal))
  if not handle.accepted:raise RuntimeError('Goal rejected')
  future=handle.get_result_async();until=time.monotonic()+180
  while not future.done():
   rclpy.spin_once(p,timeout_sec=.02)
   if time.monotonic()>until:raise RuntimeError('Navigation wall timeout')
   if p.rows[-1]['tilt']>.5:raise RuntimeError('Posture limit')
   if not armed and p.commands and p.commands[-1]['wall']>start:
    p.arm(True);armed=True;enable_wall=time.monotonic()
   if armed and p.states and p.states[-1]['wall']>enable_wall+.2 and not p.states[-1]['armed']:raise RuntimeError('Guard latched: '+p.states[-1]['reason'])
  result['action_status']=future.result().status
  result['action_end']=dict(p.rows[-1]);p.arm(False)
 except Exception as exc:result['error']=str(exc)
 finally:
  if handle is not None:
   try:p.wait(handle.cancel_goal_async(),3)
   except Exception:pass
  try:p.arm(False)
  except Exception as exc:result['disarm_error']=str(exc)
  if p.rows:
   end=p.rows[-1]['sim'];until=time.monotonic()+45
   while p.rows[-1]['sim']-end<5 and time.monotonic()<until:rclpy.spin_once(p,timeout_sec=.02)
   result['final']=p.rows[-1];result['final_goal_error_m']=math.hypot(p.rows[-1]['x']-args.goal_x,p.rows[-1]['y']-args.goal_y)
  xy=np.array([[r['x'],r['y']] for r in p.rows]).reshape(-1,2);clearance=float('inf')
  for box in json.loads(args.boxes.read_text()):
   if len(xy):clearance=min(clearance,float(np.linalg.norm(np.maximum(np.maximum(np.array(box['min'][:2])-xy,xy-np.array(box['max'][:2])),0),axis=1).min()))
  result['minimum_center_aabb_clearance_m']=clearance
  # Cruise is measured away from startup and final approach; never count stopping as cruising.
  start_sim=result.get('start',{}).get('sim',0)
  origin=result.get('start',{'x':0,'y':0})
  def interior(r):return r['sim']>start_sim+2 and math.hypot(r['x']-origin['x'],r['y']-origin['y'])>1 and math.hypot(r['x']-args.goal_x,r['y']-args.goal_y)>1
  cruise=[r['vx'] for r in p.rows if interior(r) and (not args.allow_turning or abs(r['wz'])<.2)]
  result['cruise_samples']=len(cruise)
  result['final_yaw_error_rad']=abs(math.atan2(math.sin(p.rows[-1]['yaw']-args.goal_yaw),math.cos(p.rows[-1]['yaw']-args.goal_yaw))) if p.rows else 99
  result['actual_cruise_mean_mps']=float(np.mean(cruise)) if cruise else 0
  stopped=[r for r in p.rows if r['sim']>=p.rows[-1]['sim']-2] if p.rows else []
  moving=[v['vx'] for v in p.commands if v['vx']>1e-3]
  result['checks']={'no_error':'error' not in result,'action_succeeded':result.get('action_status')==4,'goal_error_le_025':result.get('final_goal_error_m',99)<=.25,'actual_cruise_ge_06':result['actual_cruise_mean_mps']>=.6,'nonzero_controller_commands':bool(moving),'final_yaw_bounded':result['final_yaw_error_rad']<=.3 if args.allow_turning else True,'commands_bounded':bool(p.commands) and all(0<=v['vx']<=.80001 and abs(v['wz'])<=.60001 for v in p.commands),'sustained_stop_2sim_seconds':bool(stopped) and max(r['speed'] for r in stopped)<.05,'sustained_angular_stop_2sim_seconds':bool(stopped) and max(abs(r['wz']) for r in stopped)<.1,'posture_bounded':bool(p.rows) and max(r['tilt'] for r in p.rows)<.5 and all(.2 <= r['z'] <= .85 for r in p.rows),'geometry_clearance':clearance>.45}
  result['passed']=all(result['checks'].values());(args.output/'navigation_check.json').write_text(json.dumps(result,indent=2)+'\n');(args.output/'telemetry.json').write_text(json.dumps(dict(odom=p.rows,commands=p.commands,runtime=p.runtime,guard=p.states),indent=2)+'\n');print(json.dumps(result,indent=2));p.destroy_node();rclpy.shutdown()
 return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
