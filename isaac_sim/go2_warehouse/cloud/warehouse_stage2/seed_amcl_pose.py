"""Provide an explicit uncertain initial guess; never publish simulated truth continuously."""
import argparse,json,time,math
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
p=argparse.ArgumentParser();p.add_argument('--pose',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args();d=json.loads(a.pose.read_text());assert d['source'].startswith('SLAM_');rclpy.init();n=rclpy.create_node('go2_amcl_initial_guess',parameter_overrides=[Parameter('use_sim_time',value=True)]);c=n.create_client(GetState,'/amcl/get_state');pub=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10);until=time.monotonic()+60;active=False
while time.monotonic()<until:
 rclpy.spin_once(n,timeout_sec=.1)
 if not c.service_is_ready():continue
 f=c.call_async(GetState.Request());end=time.monotonic()+2
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if f.done() and f.result().current_state.id==3 and pub.get_subscription_count()>0 and 0<n.get_clock().now().nanoseconds<10**15:active=True;break
if not active:raise RuntimeError('AMCL not ready for initial pose')
m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.pose.position.x=d['x'];m.pose.pose.position.y=d['y'];m.pose.pose.orientation.z=math.sin(d['yaw']/2);m.pose.pose.orientation.w=math.cos(d['yaw']/2);m.pose.covariance[0]=m.pose.covariance[7]=.25**2;m.pose.covariance[35]=.15**2;pub.publish(m);end=time.monotonic()+1
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
a.report.parent.mkdir(parents=True,exist_ok=True)
with a.report.open('x') as f:json.dump({'source':d,'covariance_diag':[.25**2,.25**2,.15**2],'frame':'map','continuous_truth_injection':False},f,indent=2)
print(json.dumps({'initial_pose_sent':d,'covariance_diag':[.25**2,.25**2,.15**2]}));n.destroy_node();rclpy.shutdown()
