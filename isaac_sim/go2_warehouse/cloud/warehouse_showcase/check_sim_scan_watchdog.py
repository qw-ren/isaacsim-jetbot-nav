"""Real ROS guard integration in isolated domain48, no simulator/robot present."""
import argparse,importlib.util,json,os,time
from pathlib import Path
assert os.environ.get('ROS_DOMAIN_ID')=='48','Isolated ROS domain48 required'
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import SetBool
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False)
spec=importlib.util.spec_from_file_location('tested_guard',a.source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
rclpy.init();guard=module.VelocityGuard([dict(min=[100.,100.,0.],max=[101.,101.,1.])]);node=Node('showcase_watchdog_test_source');executor=SingleThreadedExecutor();executor.add_node(guard);executor.add_node(node)
pubs={k:node.create_publisher(cls,topic,qos_profile_sensor_data if k!='input' else 10) for k,cls,topic in [('odom',Odometry,'/unitree_go2/odom'),('scan',LaserScan,'/scan'),('clock',Clock,'/clock'),('input',Twist,'/nav_m1/cmd_vel')]}
states=[];outputs=[];node.create_subscription(String,'/nav_m1/guard_state',lambda m:states.append(json.loads(m.data)),10);node.create_subscription(Twist,'/unitree_go2/cmd_vel',lambda m:outputs.append((time.monotonic(),m.linear.x,m.angular.z)),10)
client=node.create_client(SetBool,'/nav_m1/enable');start=time.monotonic();last_publish=0.;facts=[]
def pump(duration,scan=True,input=True,clock=True):
 global last_publish
 until=time.monotonic()+duration
 while time.monotonic()<until:
  now=time.monotonic();sim=10.+(now-start)*.2
  if now-last_publish>=.02:
   sec=int(sim);ns=round((sim-sec)*1e9)
   if clock:
    m=Clock();m.clock.sec=sec;m.clock.nanosec=ns;pubs['clock'].publish(m)
   m=Odometry();m.header.frame_id='odom';m.child_frame_id='unitree_go2/base_link';m.header.stamp.sec=sec;m.header.stamp.nanosec=ns;m.pose.pose.position.z=.4;m.pose.pose.orientation.w=1.;pubs['odom'].publish(m)
   if scan:
    m=LaserScan();m.header.frame_id='unitree_go2/scan_ground';m.header.stamp.sec=sec;m.header.stamp.nanosec=ns;m.angle_min=-.5;m.angle_increment=.02;m.range_min=.01;m.range_max=20.;m.ranges=[10.]*51;pubs['scan'].publish(m)
   if input:
    m=Twist();m.linear.x=.8;pubs['input'].publish(m)
   last_publish=now
  for _ in range(12):executor.spin_once(timeout_sec=0)
  time.sleep(.004)
def arm():
 request=SetBool.Request();request.data=True;f=client.call_async(request)
 until=time.monotonic()+2
 while not f.done() and time.monotonic()<until:pump(.03)
 assert f.done() and f.result().success, f.result() if f.done() else 'service timeout'
 pump(.1);assert guard.armed
try:
 pump(1.5);arm();pump(.66,scan=False)
 assert guard.armed and outputs[-1][1]>.5,'Slow simulation gap incorrectly stopped'
 facts.append(dict(case='observed slow-sim scan gap',wall_gap_s=.66,sim_gap_s=.132,armed=True,output=outputs[-1][1]))
 pump(.25);phase=time.monotonic();pump(1.25,scan=False)
 assert not guard.armed and guard.reason=='SCAN_TIMEOUT' and outputs[-1][1]==0.,'Real scan loss not stopped'
 fault=next(s for s in states if s['wall']>=phase and s['reason']=='SCAN_TIMEOUT');facts.append(dict(case='independent scan loss',stop_after_wall_s=fault['wall']-phase,state=fault))
 pump(.3);assert not guard.armed and outputs[-1][1]==0.;facts.append(dict(case='scan resumed',automatic_rearm=False))
 arm();pump(.65,clock=False);assert not guard.armed and guard.reason=='CLOCK_TIMEOUT' and outputs[-1][1]==0.;facts.append(dict(case='clock loss',reason=guard.reason))
 pump(.25);arm();pump(.5,input=False);assert not guard.armed and guard.reason=='INPUT_TIMEOUT' and outputs[-1][1]==0.;facts.append(dict(case='command loss',reason=guard.reason))
 result=dict(passed=True,domain=48,simulator_or_robot_connected=False,simulated_rtf=.2,facts=facts)
except Exception as e:
 result=dict(passed=False,error=str(e),facts=facts);raise
finally:
 (a.output/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');(a.output/'states.json').write_text(json.dumps(states)+'\n');print(json.dumps(result,indent=2));executor.remove_node(guard);executor.remove_node(node);guard.destroy_node();node.destroy_node();rclpy.shutdown()
