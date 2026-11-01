"""Isolated ROS domain 52: actual gate input loss and no automatic restart."""
import argparse,hashlib,json,os,signal,subprocess,time
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='52':raise RuntimeError('Must use isolated ROS_DOMAIN_ID=52, never active simulation domain')
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
p=argparse.ArgumentParser();p.add_argument('--gate',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False)
(a.output/'tested_gate.py').write_bytes(a.gate.read_bytes());log=(a.output/'gate.log').open('w');child=subprocess.Popen(['/home/admin/isaacsim/python.sh',str(a.output/'tested_gate.py'),'--case',str(a.output)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('jetbot_gate_validation');initial=time.monotonic();send_commands=True;last_output=None
publishers={t:node.create_publisher(k,t,10)for t,k in [('/clock',Clock),('/jetbot/odom',Odometry),('/jetbot/scan',LaserScan),('/jetbot/nav_cmd_vel',Twist)]}
def timestamp(value):
 from builtin_interfaces.msg import Time
 ns=round(value*1e9);return Time(sec=ns//10**9,nanosec=ns%10**9)
def publish():
 ts=10.+(time.monotonic()-initial)*.2;c=Clock();c.clock=timestamp(ts);publishers['/clock'].publish(c)
 o=Odometry();o.header.frame_id='odom';o.header.stamp=timestamp(ts+.015);o.child_frame_id='chassis';o.pose.pose.orientation.w=1.;publishers['/jetbot/odom'].publish(o)
 s=LaserScan();s.header.frame_id='chassis';s.header.stamp=timestamp(ts-.1);s.angle_min=-3.141592653589793;s.angle_increment=3.141592653589793/180;s.angle_max=s.angle_min+359*s.angle_increment;s.range_min=.15;s.range_max=20.;s.ranges=[3.]*360;publishers['/jetbot/scan'].publish(s)
 if send_commands:
  m=Twist();m.linear.x=.15;publishers['/jetbot/nav_cmd_vel'].publish(m)
def output(m):
 global last_output;last_output=[m.linear.x,m.angular.z]
node.create_timer(.02,publish);node.create_subscription(Twist,'/jetbot/cmd_vel',output,10);arm=node.create_client(SetBool,'/jetbot/arm');facts=[];passed=False;error=None

def spin(duration):
 end=time.monotonic()+duration
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def enable():
 request=SetBool.Request();request.data=True;future=arm.call_async(request);end=time.monotonic()+3
 while not future.done()and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
 if not future.done():raise TimeoutError('Arm response')
 return future.result()
try:
 end=time.monotonic()+15
 while not arm.wait_for_service(timeout_sec=.1):
  spin(.05)
  if time.monotonic()>end:raise TimeoutError('Gate service')
 spin(.8);reply=enable();assert reply.success,reply.message;spin(.3);assert last_output and abs(last_output[0]-.15)<1e-6,last_output;facts.append(dict(case='fresh native-shaped inputs',output=last_output))
 send_commands=False;lost=time.monotonic();end=lost+.8
 while time.monotonic()<end:
  spin(.02)
  if last_output==[0.,0.]:break
 assert last_output==[0.,0.],last_output
 facts.append(dict(case='command loss while odom/clock/scan keep updating',stop_after_wall_s=time.monotonic()-lost,output=last_output));spin(.1)
 send_commands=True;spin(.35);assert last_output==[0.,0.];reply=enable();assert not reply.success and reply.message=='INPUT_TIMEOUT',reply.message;facts.append(dict(case='commands resume and enable requested',output=last_output,enable_accepted=reply.success,reason=reply.message));passed=True
except Exception as exc:error=repr(exc)
finally:
 os.killpg(child.pid,signal.SIGINT)
 try:child.wait(timeout=12)
 except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=8)
 log.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 result=dict(passed=passed,error=error,domain=52,simulation_or_robot_connected=False,source_sha256=hashlib.sha256(a.gate.read_bytes()).hexdigest(),facts=facts,child_exit_code=child.returncode)
 (a.output/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
 if not passed:raise SystemExit(1)
