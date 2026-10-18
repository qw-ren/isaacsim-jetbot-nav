#!/usr/bin/env python3
"""Publish one yaw-only virtual scan frame at the LiDAR ground projection."""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def rotation(q):
    """Rotation matrix for a ROS xyzw quaternion; reject invalid input."""
    x, y, z, w = q
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class ScanGroundFrame(Node):
    def __init__(self):
        super().__init__("go2_scan_ground_frame")
        self.base_frame = self.declare_parameter("base_frame", "unitree_go2/base_link").value
        self.lidar_frame = self.declare_parameter("lidar_frame", "unitree_go2/lidar_frame").value
        self.scan_frame = self.declare_parameter("scan_frame", "unitree_go2/scan_ground").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.ground_z = self.declare_parameter("ground_z", 0.0).value
        if self.scan_frame in (self.base_frame, self.lidar_frame, self.odom_frame, "map"):
            raise ValueError("scan_frame must be a new TF child")
        self.extrinsic = None
        self.broadcaster = TransformBroadcaster(self)
        static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_sub = self.create_subscription(TFMessage, "/tf_static", self.static_cb, static_qos)
        self.odom_sub = self.create_subscription(Odometry, "/unitree_go2/odom", self.odom_cb,
                                                 qos_profile_sensor_data)

    def static_cb(self, message):
        for t in message.transforms:
            if t.header.frame_id == self.base_frame and t.child_frame_id == self.lidar_frame:
                p, q = t.transform.translation, t.transform.rotation
                self.extrinsic = (np.array([p.x, p.y, p.z]), rotation([q.x, q.y, q.z, q.w]))
                self.get_logger().info("Using Bridge base-to-LiDAR extrinsic; ground_z=" + str(self.ground_z))

    def odom_cb(self, message):
        if self.extrinsic is None:
            return
        if message.header.frame_id != self.odom_frame or message.child_frame_id != self.base_frame:
            self.get_logger().error("Unexpected odometry frames; no virtual TF published")
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        try:
            rb = rotation([q.x, q.y, q.z, q.w])
        except ValueError:
            return
        offset, rl = self.extrinsic
        origin = np.array([p.x, p.y, p.z]) + rb @ offset
        r = rb @ rl
        yaw = math.atan2(r[1, 0], r[0, 0])
        if not np.isfinite(origin).all() or not math.isfinite(yaw):
            return
        t = TransformStamped()
        t.header = message.header
        t.child_frame_id = self.scan_frame
        t.transform.translation.x = float(origin[0])
        t.transform.translation.y = float(origin[1])
        t.transform.translation.z = float(self.ground_z)
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ScanGroundFrame()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
