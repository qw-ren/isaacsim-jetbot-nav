import json, time, math, subprocess, os
from collections import defaultdict
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from sensor_msgs_py import point_cloud2
import numpy as np
rclpy.init()
n = rclpy.create_node('go2_nav_m0_observer')
counts=defaultdict(int); samples={}; edges={}; stamps={}; backwards=defaultdict(int)
def record(key,msg):
 counts[key]+=1
 if hasattr(msg,'header'):
  s=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
  if key in stamps and s<stamps[key]: backwards[key]+=1
  stamps[key]=s
 if key=='odom':
  p=msg.pose.pose.position;q=msg.pose.pose.orientation
  samples[key]={'frame':msg.header.frame_id,'child':msg.child_frame_id,'position':[p.x,p.y,p.z], 'orientation':[q.x,q.y,q.z,q.w], 'linear':[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z], 'yaw_rate':msg.twist.twist.angular.z,'stamp':s}
 elif key=='cloud' and key not in samples:
  a=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)
  a=a[np.isfinite(a).all(axis=1)]
  samples[key]={'frame':msg.header.frame_id,'fields':[f.name for f in msg.fields], 'points':msg.width*msg.height,'finite_points':len(a),'xyz_min':a.min(axis=0).tolist() if len(a) else None,'xyz_max':a.max(axis=0).tolist() if len(a) else None,'stamp':s}
 elif key=='clock': samples[key]={'sim_time':msg.clock.sec+msg.clock.nanosec*1e-9}
 elif key in ('tf','tf_static'):
  for t in msg.transforms:
   p=t.transform.translation;q=t.transform.rotation
   edges[t.header.frame_id+' -> '+t.child_frame_id]={'topic':key,'translation':[p.x,p.y,p.z],'rotation':[q.x,q.y,q.z,q.w]}
subs=[]
for key,typ,topic,qos in [('odom',Odometry,'/unitree_go2/odom',qos_profile_sensor_data),('cloud',PointCloud2,'/unitree_go2/lidar/point_cloud',qos_profile_sensor_data),('clock',Clock,'/clock',qos_profile_sensor_data),('tf',TFMessage,'/tf',10),('tf_static',TFMessage,'/tf_static',QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
 subs.append(n.create_subscription(typ,topic,lambda m,k=key:record(k,m),qos))
start=time.monotonic()
while time.monotonic()-start<12: rclpy.spin_once(n,timeout_sec=.2)
graph={topic:{'publishers':[p.node_name for p in n.get_publishers_info_by_topic(topic)],'subscribers':[p.node_name for p in n.get_subscriptions_info_by_topic(topic)]} for topic in ['/tf','/tf_static','/map','/scan','/cmd_vel','/unitree_go2/cmd_vel','/unitree_go2/odom']}
libs={}
root='/home/admin/go2-warehouse/deps/ros_humble'
for x in ['nav2_controller/controller_server','nav2_planner/planner_server','nav2_map_server/map_server','nav2_amcl/amcl','slam_toolbox/async_slam_toolbox_node','pointcloud_to_laserscan/pointcloud_to_laserscan_node']:
 p=root+'/lib/'+x
 r=subprocess.run(['ldd',p],capture_output=True,text=True)
 libs[x]={'exists':os.path.isfile(p),'ldd_returncode':r.returncode,'missing':sorted(set(line.strip() for line in r.stdout.splitlines() if 'not found' in line))}
report={'audit_date':'2026-10-14','wall_observation_seconds':time.monotonic()-start,'mode':'read_only_no_velocity_publishers_created','counts':dict(counts),'timestamp_backwards':dict(backwards),'samples':samples,'tf_edges':edges,'graph':graph,'nodes':n.get_node_names(),'shared_libraries_in_actual_isaac_ros_environment':libs}
n.destroy_node();rclpy.shutdown()
print('M0_JSON_BEGIN')
print(json.dumps(report,indent=2))
