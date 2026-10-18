#!/usr/bin/env python3
"""Do not configure navigation during the initial wall-clock to /clock switch."""
import time
import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
rclpy.init()
n=rclpy.create_node('go2_navigation_readiness',parameter_overrides=[Parameter('use_sim_time',value=True)])
seen={'clock':0,'odom':0}
def got(k):seen[k]+=1
subs=[n.create_subscription(Clock,'/clock',lambda m:got('clock'),qos_profile_sensor_data),n.create_subscription(Odometry,'/unitree_go2/odom',lambda m:got('odom'),qos_profile_sensor_data)]
c=n.create_client(GetState,'/planner_server/get_state')
deadline=time.monotonic()+120
ready=False
while time.monotonic()<deadline:
 rclpy.spin_once(n,timeout_sec=.1)
 if min(seen.values())>=10 and c.service_is_ready():
  f=c.call_async(GetState.Request())
  until=time.monotonic()+2
  while not f.done() and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.05)
  if f.done() and f.result().current_state.id==3:ready=True;break
n.destroy_node();rclpy.shutdown()
if not ready:raise SystemExit('Planner and live sim time not ready; navigation not started')
print('Planner active and live simulation time ready',flush=True)
