"""Small estimated-odom motion bootstrap, NOT Nav2 or truth-guided control."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock as RosClock,ClockType
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args()
class Bootstrap(Node):
 def __init__(self):
  super().__init__('jetbot_motion_bootstrap',parameter_overrides=[Parameter('use_sim_time',value=True)]);self.phase='WAIT';self.sim=None;self.clock_wall=None;self.odom=None;self.odom_wall=None;self.cloud_wall=None;self.cloud_sim=None;self.front=None;self.samples=0;self.error=None;self.poses=[];self.events=[];self.start=None;self.stop_sim=None
  self.pub=self.create_publisher(Twist,'/jetbot/cmd_vel',10)
  self.create_subscription(Clock,'/clock',self.clock,10);self.create_subscription(Odometry,'/jetbot/odom',self.pose,qos_profile_sensor_data);self.create_subscription(PointCloud2,'/jetbot/livo/points',self.cloud,qos_profile_sensor_data)
  self.timer=self.create_timer(.04,self.tick,clock=RosClock(clock_type=ClockType.STEADY_TIME));self.stream=(a.case/'motion_commands.jsonl').open('x')
 def clock(self,m):self.sim=m.clock.sec+m.clock.nanosec*1e-9;self.clock_wall=time.monotonic()
 def pose(self,m):
  ts=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;p=m.pose.pose.position;q=m.pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));self.odom=(ts,p.x,p.y,yaw);self.odom_wall=time.monotonic();self.samples+=1
 def cloud(self,m):
  if m.point_step!=24 or len(m.data)!=m.row_step*m.height:self.error='CLOUD_LAYOUT';return
  dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[0,4,8],'itemsize':24});v=np.frombuffer(m.data,dtype=dtype);sel=(v['x']>.05)&(np.abs(v['y'])<.18)&(v['z']>-.04)&(v['z']<.5);self.front=float(v['x'][sel].min())if sel.any()else 100.;self.cloud_wall=time.monotonic();self.cloud_sim=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 def event(self,name):self.events.append(dict(name=name,sim=self.sim,odom=self.odom));self.phase=name
 def tick(self):
  wall=time.monotonic();fresh=(self.clock_wall is not None and wall-self.clock_wall<.5 and self.odom_wall is not None and wall-self.odom_wall<.5 and self.cloud_wall is not None and wall-self.cloud_wall<.8 and -.025<=self.sim-self.odom[0]<.30 and 0<=self.sim-self.cloud_sim<.30)
  v,w=0.,0.
  if self.phase=='WAIT' and fresh and self.samples>80:self.start=self.odom;self.event('FORWARD')
  if self.phase in ['FORWARD','TURN']:
   if not fresh:
    self.error='SENSOR_OR_ESTIMATE_TIMEOUT'
    (a.case/'MOTION_FRESHNESS_FAULT.json').write_text(json.dumps(dict(sim=self.sim,odom=self.odom,clock_wall_age=None if self.clock_wall is None else wall-self.clock_wall,odom_wall_age=None if self.odom_wall is None else wall-self.odom_wall,cloud_wall_age=None if self.cloud_wall is None else wall-self.cloud_wall,cloud_sim=self.cloud_sim),indent=2)+'\n')
   if self.front is not None and self.front<.45:self.error='FRONT_CLEARANCE'
   if self.error:self.event('FAULT')
   elif self.phase=='FORWARD':
    if math.hypot(self.odom[1]-self.start[1],self.odom[2]-self.start[2])>=.7:self.turn_start=self.odom[3];self.event('TURN')
    else:v=.15
   if self.phase=='TURN':
    delta=math.atan2(math.sin(self.odom[3]-self.turn_start),math.cos(self.odom[3]-self.turn_start))
    if delta>=.5:self.stop_sim=self.sim;self.event('STOP')
    else:w=.35
  if self.phase=='STOP' and self.sim-self.stop_sim>=2:self.event('DONE')
  msg=Twist();msg.linear.x=v;msg.angular.z=w;self.pub.publish(msg);self.stream.write(json.dumps(dict(wall=wall,sim=self.sim,phase=self.phase,command=[v,w],front=self.front,fresh=fresh))+'\n');self.stream.flush()
  (a.case/'MOTION_PROGRESS.json').write_text(json.dumps(dict(phase=self.phase,error=self.error,estimated_samples=self.samples,events=self.events))+'\n')
rclpy.init();node=Bootstrap()
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.stream.close();(a.case/'MOTION_RESULT.json').write_text(json.dumps(dict(passed=node.phase=='DONE'and not node.error,phase=node.phase,error=node.error,events=node.events,truth_input=False,scope='bounded differential-drive and FAST-LIVO estimated odom bootstrap; no Nav2'),indent=2)+'\n');node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
