"""Read-only AMCL and map/odom diagnostics; no reference/command subscriptions."""
import argparse,json,signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer,TransformListener
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Node('jetbot_localization_probe',parameter_overrides=[Parameter('use_sim_time',value=True)]);buffer=Buffer();listener=TransformListener(buffer,n);stream=(a.case/'localization_probe.jsonl').open('x');last=[None];count=[0];stop=[False]
def pose(m):
 p=m.pose.pose.position;q=m.pose.pose.orientation;last[0]=dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,xy=[p.x,p.y],xyzw=[q.x,q.y,q.z,q.w],covariance=list(m.pose.covariance))
n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',pose,10)
def tick():
 row=dict(stamp=n.get_clock().now().nanoseconds/1e9,amcl=last[0],transforms={})
 for parent,child in [('map','odom'),('map','chassis'),('odom','chassis')]:
  try:
   m=buffer.lookup_transform(parent,child,rclpy.time.Time());v=m.transform.translation;q=m.transform.rotation;row['transforms'][parent+'->'+child]=dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,xyz=[v.x,v.y,v.z],xyzw=[q.x,q.y,q.z,q.w])
  except Exception:pass
 stream.write(json.dumps(row)+'\n');stream.flush();count[0]+=1
n.create_timer(.2,tick)
def finish(*_):stop[0]=True
signal.signal(signal.SIGINT,finish);signal.signal(signal.SIGTERM,finish);fault=None
try:
 while rclpy.ok()and not stop[0]:rclpy.spin_once(n,timeout_sec=.1)
except Exception as e:fault=repr(e);raise
finally:
 stream.close();(a.case/'LOCALIZATION_PROBE_RESULT.json').write_text(json.dumps(dict(samples=count[0],fault=fault,truth_input=False,command_publishers=0)));n.destroy_node()
 if rclpy.ok():rclpy.shutdown()
