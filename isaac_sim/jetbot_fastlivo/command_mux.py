"""One frozen control source -> existing safety gate. Stale selected source locks the lease."""
import argparse,json,time,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock,ClockType
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist,TwistStamped
from rosgraph_msgs.msg import Clock as ClockMessage
from extension_core import CommandLease
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args()
source=json.loads((a.case/'CASE.json').read_text())['navigation']['extensions']['control_source'];assert source in ('nav2','ppo')
stop=False
def terminate(*_):
 global stop;stop=True
class Mux(Node):
 def __init__(self):
  super().__init__('jetbot_command_mux',parameter_overrides=[Parameter('use_sim_time',value=True)]);self.sim=None;self.lease=CommandLease();self.sent=0;self.zero_sent=False
  self.topic='/jetbot/nav2_proposal'if source=='nav2'else'/jetbot/policy_proposal'
  self.pub=self.create_publisher(Twist,'/jetbot/nav_cmd_vel',1);self.create_subscription(ClockMessage,'/clock',self.clock,10);self.create_subscription(Twist if source=='nav2'else TwistStamped,self.topic,self.proposal,1);self.create_timer(.04,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
 def clock(self,m):self.sim=m.clock.sec+m.clock.nanosec*1e-9
 def proposal(self,m):
  if self.sim is None:return
  value=m if source=='nav2'else m.twist
  stamp=self.sim if source=='nav2'else m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  if source=='ppo'and m.header.frame_id!='chassis':self.lease.fault='POLICY_FRAME';return
  self.lease.accept([value.linear.x,value.angular.z],stamp,time.monotonic(),self.sim)
 def tick(self):
  if len(self.get_publishers_info_by_topic(self.topic))>1:self.lease.fault='DUPLICATE_SELECTED_SOURCE'
  if self.sim is None:return
  # Normal completed missions intentionally stop controller output while disarmed.
  armed=True
  try:armed=bool(json.loads((a.case/'GATE_PROGRESS.json').read_text())['armed'])
  except (FileNotFoundError,json.JSONDecodeError,KeyError):pass
  if not armed and self.lease.fault is None and self.lease.last_wall is not None and time.monotonic()-self.lease.last_wall>.25:self.lease=CommandLease()
  value=self.lease.output(time.monotonic(),self.sim)
  if value is not None:
   m=Twist();m.linear.x,m.angular.z=value;self.pub.publish(m);self.sent+=1
  elif self.lease.fault and not self.zero_sent:
   self.pub.publish(Twist());self.zero_sent=True
   # Stop refreshing gate input after zero; existing gate latches INPUT_TIMEOUT.
  row=dict(source=source,selected_topic=self.topic,fault=self.lease.fault,published=self.sent,sim=self.sim,automatic_fallback=False,truth_input=False)
  tmp=a.case/'MUX_PROGRESS.tmp';tmp.write_text(json.dumps(row)+'\n');tmp.replace(a.case/'MUX_PROGRESS.json')
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Mux();signal.signal(signal.SIGINT,terminate);signal.signal(signal.SIGTERM,terminate)
try:
 while rclpy.ok()and not stop:rclpy.spin_once(node,timeout_sec=.05)
finally:
 node.pub.publish(Twist());(a.case/'MUX_RESULT.json').write_text(json.dumps(dict(source=source,fault=node.lease.fault,published=node.sent,shutdown_zero=True,automatic_fallback=False),indent=2)+'\n');node.destroy_node();rclpy.shutdown()
