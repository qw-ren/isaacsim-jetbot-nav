#!/usr/bin/env python3
"""Wait for established nonzero /clock before constructing TF buffers."""
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
rclpy.init();n=rclpy.create_node('go2_wait_live_sim',parameter_overrides=[Parameter('use_sim_time',value=True)])
seen=[]
def cb(m):seen.append(m.clock.sec+m.clock.nanosec*1e-9)
s=n.create_subscription(Clock,'/clock',cb,qos_profile_sensor_data)
until=time.monotonic()+120
while time.monotonic()<until and not(len(seen)>=10 and seen[-1]>.5 and seen[-1]>seen[0]):rclpy.spin_once(n,timeout_sec=.1)
ready=len(seen)>=10 and seen[-1]>.5 and seen[-1]>seen[0]
n.destroy_node();rclpy.shutdown()
if not ready:raise SystemExit('No advancing simulation clock; nodes not started')
