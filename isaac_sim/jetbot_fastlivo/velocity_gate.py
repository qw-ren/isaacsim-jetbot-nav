"""JetBot simulation velocity gate; estimated pose and real scan only."""
import argparse,json,math,time,signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import Parameter
from rclpy.clock import Clock as RosClock,ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args()
class Gate(Node):
 def __init__(self):
  super().__init__('jetbot_velocity_gate',parameter_overrides=[Parameter('use_sim_time',value=True)]);self.sim=None;self.odom=None;self.points=None;self.times={};self.latest=[0.,0.];self.armed=False;self.fault=None;self.reason='DISARMED';self.stream=(a.case/'gate_states.jsonl').open('x');self.scan_log=(a.case/'scan_delivery.jsonl').open('x')
  self.motion_limits=json.loads((a.case/'CASE.json').read_text()).get('motion_limits',{})if(a.case/'CASE.json').exists()else {};self.max_linear=float(self.motion_limits.get('max_linear_m_s',.2));self.max_angular=float(self.motion_limits.get('max_angular_rad_s',.6));self.wheel_max=float(self.motion_limits.get('max_wheel_rad_s',1e20));self.wheel_radius=float(self.motion_limits.get('wheel_radius_m',.03));self.wheel_base=float(self.motion_limits.get('wheel_base_m',.1125))
  self.pub=self.create_publisher(Twist,'/jetbot/cmd_vel',1)
  self.create_subscription(Clock,'/clock',self.clock,10);self.create_subscription(Odometry,'/jetbot/odom',self.pose,qos_profile_sensor_data);self.create_subscription(LaserScan,'/jetbot/scan',self.scan,qos_profile_sensor_data);self.create_subscription(Twist,'/jetbot/nav_cmd_vel',self.command,1)
  self.create_service(SetBool,'/jetbot/arm',self.arm);self.timer=self.create_timer(.04,self.tick,clock=RosClock(clock_type=ClockType.STEADY_TIME))
 def clock(self,m):self.sim=m.clock.sec+m.clock.nanosec*1e-9;self.times['clock']=time.monotonic()
 def pose(self,m):self.odom=m;self.times['odom']=time.monotonic()
 def scan(self,m):
  ranges=np.asarray(m.ranges);angles=m.angle_min+np.arange(len(ranges))*m.angle_increment;valid=np.isfinite(ranges)&(ranges>=m.range_min)&(ranges<=m.range_max);self.points=np.c_[ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid])];self.scan_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;self.times['scan']=time.monotonic();self.scan_log.write(json.dumps(dict(wall=self.times['scan'],stamp=self.scan_stamp,sim=self.sim,finite_ranges=len(self.points)))+'\n');self.scan_log.flush()
 def command(self,m):self.latest=[m.linear.x,m.angular.z];self.times['input']=time.monotonic()
 def sensor_problem(self):
  wall=time.monotonic()
  for name,limit in [('clock',.5),('odom',.5),('scan',.8)]:
   if name not in self.times or wall-self.times[name]>limit:return name.upper()+'_TIMEOUT'
  ts=self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9
  if not -.025<=self.sim-ts<=.30:return 'ODOM_SIM_AGE'
  if not 0<=self.sim-self.scan_stamp<=.30:return 'SCAN_SIM_AGE'
  if self.points is None or len(self.points)<5:return 'SCAN_EMPTY'
  q=self.odom.pose.pose.orientation;tilt=math.acos(float(np.clip(1-2*(q.x*q.x+q.y*q.y),-1,1)))
  if tilt>.6:return 'POSTURE'
  if len(self.get_publishers_info_by_topic('/jetbot/cmd_vel'))>1 or len(self.get_publishers_info_by_topic('/jetbot/nav_cmd_vel'))>1:return 'MULTIPLE_COMMAND_SOURCES'
  return None
 def arm(self,request,response):
  if not request.data:self.armed=False;self.reason='DISARMED';response.success=True;return response
  problem=self.sensor_problem()
  if not problem and ('input'not in self.times or time.monotonic()-self.times['input']>.35):problem='WAITING_FOR_CONTROLLER_INPUT'
  if self.fault or problem:response.success=False;response.message=str(self.fault or problem);return response
  self.armed=True;self.reason='ENABLED';response.success=True;response.message='JetBot simulation only';return response
 def tick(self):
  v,w=0.,0.;clearance=None
  if self.armed:
   problem=self.sensor_problem()
   if not problem and ('input'not in self.times or time.monotonic()-self.times['input']>.35):problem='INPUT_TIMEOUT'
   if not np.isfinite(self.latest).all():problem='NONFINITE_COMMAND'
   if not problem:
    w=float(np.clip(self.latest[1],-self.max_angular,self.max_angular));linear_cap=min(self.max_linear,max(0.,self.wheel_max*self.wheel_radius-abs(w)*self.wheel_base/2));v=float(np.clip(self.latest[0],-linear_cap,linear_cap));t=np.linspace(0,1.5,16)
    path=np.c_[v*t,np.zeros_like(t)]if abs(w)<1e-6 else np.c_[v/w*np.sin(w*t),v/w*(1-np.cos(w*t))]
    clearance=float(np.linalg.norm(self.points[:,None,:]-path[None,:,:],axis=2).min())
    if clearance<float(self.motion_limits.get('scan_clearance_m',.20)):problem='PREDICTED_SCAN_CLEARANCE'
   if problem:self.fault=problem;self.armed=False;self.reason=problem;v=w=0.
   else:self.reason='ENABLED'
  m=Twist();m.linear.x=v;m.angular.z=w;self.pub.publish(m)
  row=dict(wall=time.monotonic(),sim=self.sim,armed=self.armed,reason=self.reason,fault=self.fault,output=[v,w],predicted_clearance_m=clearance,scan_stamp=getattr(self,'scan_stamp',None),stream_wall_ages={k:time.monotonic()-v for k,v in self.times.items()});self.stream.write(json.dumps(row)+'\n');self.stream.flush();temporary=a.case/'GATE_PROGRESS.tmp';temporary.write_text(json.dumps(row)+'\n');temporary.replace(a.case/'GATE_PROGRESS.json')
stop_requested=False;runtime_error=None
def request_stop(*_):
 global stop_requested;stop_requested=True
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Gate();signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
try:
 while rclpy.ok()and not stop_requested:rclpy.spin_once(node,timeout_sec=.05)
except Exception as exc:
 runtime_error=repr(exc);node.fault=node.fault or 'GATE_CALLBACK_ERROR';raise
finally:
 zero_published=False
 try:node.pub.publish(Twist());zero_published=True
 except Exception:pass
 node.stream.close();node.scan_log.close();(a.case/'GATE_RESULT.json').write_text(json.dumps(dict(fault=node.fault,last_reason=node.reason,runtime_error=runtime_error,shutdown_zero_published=zero_published,automatic_rearm=False,simulation_only=True),indent=2)+'\n');node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
