"""ROS52-only synthetic PPO proposal -> mux -> actual 3x velocity gate contract.
No policy model, simulator, reference pose or physical robot is connected.
"""
import argparse,json,math,os,signal,subprocess,time,hashlib
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist,TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--source-case',type=Path,required=True);a=p.parse_args();assert os.environ.get('ROS_DOMAIN_ID')=='52';a.output.mkdir(parents=True,exist_ok=False);source=Path(__file__).parent;cfg=json.loads((a.source_case/'CASE.json').read_text());assert cfg['robot_variant']['scale']==3.;cfg['navigation']['extensions']={'control_source':'ppo'};(a.output/'CASE.json').write_text(json.dumps(cfg,indent=2));children=[];handles=[];error=None;checks={};sim=10.;command=[5.,2.];received=[];forwarded=[]
rclpy.init();node=rclpy.create_node('scaled_policy_contract_test');clock=node.create_publisher(Clock,'/clock',10);scan=node.create_publisher(LaserScan,'/jetbot/scan',10);odom=node.create_publisher(Odometry,'/jetbot/odom',10);policy=node.create_publisher(TwistStamped,'/jetbot/policy_proposal',10);nav=node.create_publisher(Twist,'/jetbot/nav2_proposal',10);node.create_subscription(Twist,'/jetbot/cmd_vel',lambda m:received.append((time.monotonic(),m.linear.x,m.angular.z)),10);node.create_subscription(Twist,'/jetbot/nav_cmd_vel',lambda m:forwarded.append((time.monotonic(),m.linear.x,m.angular.z)),10);arm=node.create_client(SetBool,'/jetbot/arm')
def tick(send=True):
 global sim
 sim+=.01;message=Clock();ns=round(sim*1e9);message.clock.sec,message.clock.nanosec=divmod(ns,10**9);clock.publish(message)
 m=Odometry();m.header.stamp=message.clock;m.header.frame_id='odom';m.child_frame_id='chassis';m.pose.pose.orientation.w=1.;odom.publish(m)
 m=LaserScan();m.header.stamp=message.clock;m.header.frame_id='chassis';m.range_min=.15;m.range_max=20.;m.angle_min=-math.pi;m.angle_increment=2*math.pi/360;m.angle_max=math.pi-m.angle_increment;m.ranges=[10.]*360;scan.publish(m)
 m=Twist();m.linear.x=.12;m.angular.z=-.19;nav.publish(m)
 if send:
  m=TwistStamped();m.header.stamp=message.clock;m.header.frame_id='chassis';m.twist.linear.x,m.twist.angular.z=command;policy.publish(m)
 rclpy.spin_once(node,timeout_sec=.005);time.sleep(.015)
def run(seconds,send=True):
 end=time.monotonic()+seconds
 while time.monotonic()<end:tick(send)
try:
 for name in ['velocity_gate','command_mux']:
  handle=(a.output/(name+'.log')).open('w');handles.append(handle);children.append(subprocess.Popen(['/home/admin/isaacsim/python.sh',str(source/(name+'.py')),'--case',str(a.output)],stdout=handle,stderr=subprocess.STDOUT,start_new_session=True))
 run(5.);assert arm.wait_for_service(timeout_sec=1.);request=SetBool.Request();request.data=True;future=arm.call_async(request);deadline=time.monotonic()+3
 while not future.done()and time.monotonic()<deadline:tick()
 assert future.done()and future.result().success,'Gate did not arm on fresh synthetic inputs'
 for sign in [1.,-1.]:
  command=[sign*5.,sign*2.];received.clear();forwarded.clear();run(1.2);limits=cfg['motion_limits'];expected=[sign*limits['max_linear_m_s'],sign*limits['max_angular_rad_s']];assert any(abs(v-expected[0])<1e-8 and abs(w-expected[1])<1e-8 for _,v,w in received),'Actual gate limits differ from active CASE';assert any(abs(v-command[0])<1e-8 and abs(w-command[1])<1e-8 for _,v,w in forwarded),'Selected physical policy proposal not forwarded'
  maximum=max(max(abs((v-w*limits['wheel_base_m']/2)/limits['wheel_radius_m']),abs((v+w*limits['wheel_base_m']/2)/limits['wheel_radius_m']))for _,v,w in received);assert maximum<=limits['max_wheel_rad_s']+1e-8;checks['positive_caps'if sign>0 else'negative_caps']=dict(v_m_s=expected[0],w_rad_s=expected[1],actual_implied_max_wheel_rad_s=maximum)
 run(.9,False);mux=json.loads((a.output/'MUX_PROGRESS.json').read_text());assert mux['fault']=='SELECTED_SOURCE_TIMEOUT';assert received[-1][1:]==(0.,0.);checks['unselected_nav_source_not_fallback']=True;boundary=time.monotonic();run(.6,True);assert not any(v!=0 or w!=0 for wall,v,w in forwarded if wall>boundary);checks['no_automatic_resume_after_source_loss']=True
except Exception as exc:error=repr(exc)
finally:
 for child in reversed(children):
  if child.poll()is None:os.killpg(child.pid,signal.SIGINT)
  child.wait(timeout=10)
 for handle in handles:handle.close()
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 result=dict(passed=error is None and all(c.returncode==0 for c in children),checks=checks,error=error,child_exit_codes=[c.returncode for c in children],domain=52,real_robot_or_simulator_connected=False,model_implemented=False,geometry_source_case=str(a.source_case),motion_limits=cfg['motion_limits'],source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(a.output/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if not result['passed']:raise SystemExit(1)
