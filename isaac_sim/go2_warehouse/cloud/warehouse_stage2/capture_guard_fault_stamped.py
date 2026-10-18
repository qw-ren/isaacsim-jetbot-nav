#!/usr/bin/env python3
"""Capture exactly stamped source cloud, scan and odometry for a latched fault."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan,PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=150);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
rclpy.init();n=rclpy.create_node('go2_stamped_fault_capture');buf={x:{} for x in ['scan','cloud','odom']};fault=None
st=lambda m:round(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,6)
def cb(k,m):
 buf[k][st(m)]=m
 if len(buf[k])>200:buf[k].pop(next(iter(buf[k])))
def status(m):
 global fault
 d=json.loads(m.data)
 if fault is None and d['reason'] not in ['DISARMED','ENABLED']:fault=d
subs=[n.create_subscription(LaserScan,'/scan',lambda m:cb('scan',m),qos_profile_sensor_data),n.create_subscription(PointCloud2,'/unitree_go2/lidar/point_cloud',lambda m:cb('cloud',m),qos_profile_sensor_data),n.create_subscription(Odometry,'/unitree_go2/odom',lambda m:cb('odom',m),qos_profile_sensor_data),n.create_subscription(String,'/nav_m1/guard_state',status,10)]
until=time.monotonic()+a.seconds;target=None
while time.monotonic()<until:
 rclpy.spin_once(n,timeout_sec=.03)
 if fault is not None and fault.get('scan_stamp') is not None:
  target=round(fault['scan_stamp'],6)
  if all(target in b for b in buf.values()):break
result={'fault':fault,'target_stamp':target,'matched':target is not None and all(target in b for b in buf.values())}
if result['matched']:
 s,c,o=(buf[k][target] for k in ['scan','cloud','odom']);q=o.pose.pose.orientation;v=o.pose.pose.position
 xyz=point_cloud2.read_points_numpy(c,field_names=('x','y','z'),skip_nans=True)
 np.savez_compressed(a.output/'fault.npz',xyz=xyz,ranges=np.array(s.ranges),position=np.array([v.x,v.y,v.z]),quaternion_xyzw=np.array([q.x,q.y,q.z,q.w]))
 result.update(angle_min=s.angle_min,angle_increment=s.angle_increment)
(a.output/'capture.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
