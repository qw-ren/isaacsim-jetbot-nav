"""Isolated ROS-domain interface test using synthetic proposals, not a PPO model."""
import os,json,time,signal,subprocess,argparse
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist,TwistStamped
from rosgraph_msgs.msg import Clock
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert os.environ.get('ROS_DOMAIN_ID')=='52';a.output.mkdir(exist_ok=False)
rclpy.init();node=rclpy.create_node('mux_contract_test');clock=node.create_publisher(Clock,'/clock',10);nav=node.create_publisher(Twist,'/jetbot/nav2_proposal',1);policy=node.create_publisher(TwistStamped,'/jetbot/policy_proposal',1);received=[];node.create_subscription(Twist,'/jetbot/nav_cmd_vel',lambda m:received.append((time.monotonic(),m.linear.x)),10);sim=10.;results=[]
def tick(mode,send):
 global sim
 sim+=.01;c=Clock();n=round(sim*1e9);c.clock.sec=n//10**9;c.clock.nanosec=n%10**9;clock.publish(c)
 v=Twist();v.linear.x=.12 if mode=='nav2'else .19
 if send or mode!='nav2':nav.publish(v)
 m=TwistStamped();m.header.stamp=c.clock;m.header.frame_id='chassis';m.twist.linear.x=.12 if mode=='ppo'else .19
 if send or mode!='ppo':policy.publish(m)
 rclpy.spin_once(node,timeout_sec=.01);time.sleep(.01)
try:
 for mode in ('nav2','ppo'):
  case=a.output/mode;case.mkdir();(case/'CASE.json').write_text(json.dumps({'navigation':{'extensions':{'control_source':mode}}}));log=(case/'process.log').open('w');child=subprocess.Popen(['/home/admin/isaacsim/python.sh',str(Path(__file__).with_name('command_mux.py')),'--case',str(case)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True);received.clear()
  (case/'GATE_PROGRESS.json').write_text(json.dumps({'armed':False}))
  try:
   end=time.monotonic()+5
   while time.monotonic()<end:tick(mode,True)
   assert received and any(abs(x-.12)<1e-8 for _,x in received),'Selected source not forwarded'
   assert all(abs(x-.19)>1e-8 for _,x in received),'Unselected source leaked'
   (case/'GATE_PROGRESS.json').write_text(json.dumps({'armed':False}));end=time.monotonic()+.7
   while time.monotonic()<end:tick(mode,False)
   assert json.loads((case/'MUX_PROGRESS.json').read_text())['fault'] is None,'Normal disarmed idle must not lock the next mission'
   end=time.monotonic()+.3
   while time.monotonic()<end:tick(mode,True)
   (case/'GATE_PROGRESS.json').write_text(json.dumps({'armed':True}));end=time.monotonic()+.7
   while time.monotonic()<end:tick(mode,False)
   fault=json.loads((case/'MUX_PROGRESS.json').read_text())['fault'];assert fault=='SELECTED_SOURCE_TIMEOUT';assert received[-1][1]==0
   boundary=time.monotonic();end=boundary+.7
   while time.monotonic()<end:tick(mode,True)
   assert not any(x!=0 for stamp,x in received if stamp>boundary),'Auto restart after source loss'
   results.append({'mode':mode,'selected_forwarded':True,'unselected_ignored':True,'stale_zero_and_latched':True,'automatic_resume':False})
  finally:
   os.killpg(child.pid,signal.SIGINT);child.wait(timeout=10);log.close();assert child.returncode==0
 (a.output/'RESULT.json').write_text(json.dumps({'passed':True,'domain':52,'real_robot_or_simulator_connected':False,'model_implemented':False,'results':results},indent=2)+'\n');print((a.output/'RESULT.json').read_text())
finally:node.destroy_node();rclpy.shutdown()
