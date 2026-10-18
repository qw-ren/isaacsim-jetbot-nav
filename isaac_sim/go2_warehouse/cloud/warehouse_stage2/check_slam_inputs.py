"""Reject truth map TF or an existing map publisher before starting SLAM."""
import time,json
import rclpy
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
rclpy.init();n=rclpy.create_node('go2_slam_input_audit');edges=set()
def cb(m):
 for t in m.transforms:edges.add((t.header.frame_id,t.child_frame_id))
q=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);sub=n.create_subscription(TFMessage,'/tf_static',cb,q);end=time.monotonic()+3
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
checks={'no_static_map_odom':('map','odom') not in edges,'no_existing_map_publisher':not n.get_publishers_info_by_topic('/map'),'one_scan_publisher':len(n.get_publishers_info_by_topic('/scan'))==1,'expected_static_lidar_extrinsic':('unitree_go2/base_link','unitree_go2/lidar_frame') in edges}
print(json.dumps({'checks':checks,'static_edges':sorted(edges)},indent=2));n.destroy_node();rclpy.shutdown();raise SystemExit(0 if all(checks.values()) else 1)
