#!/usr/bin/env python3
"""Capture one stamped cloud/odom pair without publishing commands."""
import argparse, json, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
p=argparse.ArgumentParser();p.add_argument("--output",required=True);args=p.parse_args()
out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
rclpy.init();n=rclpy.create_node("go2_cloud_capture");odoms={};clouds={};result=None
def stamp(m):return (m.header.stamp.sec,m.header.stamp.nanosec)
def match():
 global result
 if result is not None:return
 for s in clouds.keys() & odoms.keys():
  c=clouds[s];o=odoms[s];a=point_cloud2.read_points_numpy(c,field_names=("x","y","z"),skip_nans=True)
  pose=o.pose.pose
  result={"stamp":list(s),"cloud_frame":c.header.frame_id,"odom_frame":o.header.frame_id,"base_frame":o.child_frame_id,"position":[pose.position.x,pose.position.y,pose.position.z],"quaternion_xyzw":[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w],"linear_velocity":[o.twist.twist.linear.x,o.twist.twist.linear.y,o.twist.twist.linear.z],"yaw_rate":o.twist.twist.angular.z,"point_count":len(a)}
  np.savez_compressed(out/"cloud_odom_pair.npz",xyz=a,position=np.array(result["position"]),quaternion_xyzw=np.array(result["quaternion_xyzw"]))
  (out/"cloud_odom_pair.json").write_text(json.dumps(result,indent=2)+"\n");break
def odom(m):
 odoms[stamp(m)]=m
 if len(odoms)>30:odoms.pop(next(iter(odoms)))
 match()
def cloud(m):
 clouds[stamp(m)]=m
 if len(clouds)>10:clouds.pop(next(iter(clouds)))
 match()
subs=[n.create_subscription(Odometry,"/unitree_go2/odom",odom,qos_profile_sensor_data),n.create_subscription(PointCloud2,"/unitree_go2/lidar/point_cloud",cloud,qos_profile_sensor_data)]
start=time.monotonic()
while result is None and time.monotonic()-start<20:rclpy.spin_once(n,timeout_sec=.2)
n.destroy_node();rclpy.shutdown()
if result is None:raise SystemExit("No exactly stamped cloud/odom pair within 20 seconds")
print(json.dumps(result,indent=2))
