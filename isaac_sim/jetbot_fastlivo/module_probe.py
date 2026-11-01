"""Read-only teammate interface example; no model, velocity publisher or truth subscription."""
import argparse,json,time,signal
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock,ClockType
from sensor_msgs.msg import Image,CameraInfo,LaserScan
from nav_msgs.msg import Odometry,Path as NavPath
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();stop=False
class Probe(Node):
 def __init__(self):
  super().__init__('jetbot_module_interface_probe');self.counts={};self.latest={};self.saved=0;self.log=(a.case/'module_observations.jsonl').open('x')
  for topic,kind,name in [('/jetbot/depth/image_rect_raw',Image,'depth'),('/jetbot/image_raw',Image,'rgb'),('/jetbot/camera_info',CameraInfo,'intrinsics'),('/jetbot/scan',LaserScan,'scan'),('/jetbot/odom',Odometry,'odometry'),('/plan',NavPath,'global_plan')]:
   self.create_subscription(kind,topic,lambda m,n=name:self.receive(n,m),qos_profile_sensor_data)
  self.create_timer(.5,self.sample,clock=Clock(clock_type=ClockType.STEADY_TIME))
 def receive(self,name,m):self.counts[name]=self.counts.get(name,0)+1;self.latest[name]=(m,time.monotonic())
 def sample(self):
  if not all(n in self.latest for n in ('rgb','depth','intrinsics','scan','odometry','global_plan')):return
  image=self.latest['rgb'][0];info=self.latest['intrinsics'][0];scan=self.latest['scan'][0];odom=self.latest['odometry'][0];plan=self.latest['global_plan'][0]
  depth=self.latest['depth'][0]
  if image.header!=depth.header or image.header!=info.header:return
  depth_array=np.frombuffer(depth.data,dtype='<f4');valid_depth=depth.encoding=='32FC1'and depth.width==image.width and depth.height==image.height and depth.step==image.width*4 and np.count_nonzero(np.isfinite(depth_array)&(depth_array>0))>100
  same_header=image.header==info.header
  valid=image.encoding=='rgb8'and image.width==info.width and image.height==info.height and len(image.data)==image.height*image.step and info.k[0]>0 and info.k[4]>0
  row=dict(rgb_stamp=image.header.stamp.sec+image.header.stamp.nanosec*1e-9,rgb_frame=image.header.frame_id,rgb_shape=[image.height,image.width,3],scan_frame=scan.header.frame_id,scan_bins=len(scan.ranges),odometry_frame=odom.header.frame_id,body_frame=odom.child_frame_id,plan_frame=plan.header.frame_id,plan_points=len(plan.poses),image_intrinsics_same_header=same_header,rgb_valid=bool(valid),depth_valid=bool(valid_depth),depth_encoding=depth.encoding,depth_units='m optical Z',depth_finite_pixels=int(np.isfinite(depth_array).sum()),wall_ages_s={k:time.monotonic()-v[1]for k,v in self.latest.items()},truth_input=False,velocity_publisher=False)
  self.log.write(json.dumps(row)+'\n');self.log.flush();self.saved+=1
  summary=dict(samples=self.saved,counts=self.counts,last=row,fresh_for_control=all(time.monotonic()-v[1]<(2. if k=='global_plan'else .8)for k,v in self.latest.items()),format_checks_passed=bool(same_header and valid and valid_depth and image.header.frame_id=='jetbot/camera_optical'and odom.header.frame_id=='odom'and odom.child_frame_id=='chassis'and plan.header.frame_id=='map'))
  tmp=a.case/'MODULE_READINESS.tmp';tmp.write_text(json.dumps(summary,indent=2)+'\n');tmp.replace(a.case/'MODULE_READINESS.json')
def terminate(*_):
 global stop;stop=True
rclpy.init();node=Probe();signal.signal(signal.SIGINT,terminate);signal.signal(signal.SIGTERM,terminate)
try:
 while rclpy.ok()and not stop:rclpy.spin_once(node,timeout_sec=.05)
finally:
 node.log.close();node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
