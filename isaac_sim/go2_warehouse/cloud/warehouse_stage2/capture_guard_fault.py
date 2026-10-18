#!/usr/bin/env python3
"""Read-only source cloud and scan snapshot at a guard fault."""
import json,time,argparse
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
rclpy.init();n=rclpy.create_node('go2_guard_fault_capture');latest={};fault=None
def state(m):
 global fault
 d=json.loads(m.data)
 if d['reason'] not in ('DISARMED','ENABLED'):fault=d
subs=[n.create_subscription(LaserScan,'/scan',lambda m:latest.update(scan=m),qos_profile_sensor_data),n.create_subscription(PointCloud2,'/unitree_go2/lidar/point_cloud',lambda m:latest.update(cloud=m),qos_profile_sensor_data),n.create_subscription(Odometry,'/unitree_go2/odom',lambda m:latest.update(odom=m),qos_profile_sensor_data),n.create_subscription(String,'/nav_m1/guard_state',state,10)]
until=time.monotonic()+150
while time.monotonic()<until and not(fault and all(k in latest for k in ('scan','cloud','odom'))):rclpy.spin_once(n,timeout_sec=.05)
if fault:
 s,c,o=latest['scan'],latest['cloud'],latest['odom'];q=o.pose.pose.orientation;v=o.pose.pose.position
 xyz=point_cloud2.read_points_numpy(c,field_names=('x','y','z'),skip_nans=True)
 np.savez_compressed(args.output/'fault.npz',xyz=xyz,ranges=np.array(s.ranges),position=np.array([v.x,v.y,v.z]),quaternion_xyzw=np.array([q.x,q.y,q.z,q.w]))
 (args.output/'fault.json').write_text(json.dumps(dict(guard=fault,angle_min=s.angle_min,angle_increment=s.angle_increment,scan_stamp=[s.header.stamp.sec,s.header.stamp.nanosec],cloud_stamp=[c.header.stamp.sec,c.header.stamp.nanosec],odom_stamp=[o.header.stamp.sec,o.header.stamp.nanosec]),indent=2)+'\n')
 print('Captured actual fault source')
else:print('No guard fault observed')
n.destroy_node();rclpy.shutdown()
