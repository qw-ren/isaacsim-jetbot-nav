import argparse,json,time,math
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();rclpy.init();n=rclpy.create_node('go2_slam_pose_snapshot',parameter_overrides=[Parameter('use_sim_time',value=True)]);b=Buffer(node=n);listener=TransformListener(b,n);end=time.monotonic()+12;result=None
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05)
 try:
  t=b.lookup_transform('map','unitree_go2/scan_ground',Time());p=t.transform.translation;q=t.transform.rotation;result={'source':'SLAM_map_to_odom_plus_odometry_scan_ground_TF','x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'stamp':t.header.stamp.sec+t.header.stamp.nanosec*1e-9};break
 except Exception:pass
if result is None:raise RuntimeError('SLAM pose snapshot unavailable')
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result));n.destroy_node();rclpy.shutdown()
