#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import sys


class PoseToOdomRepublisher(Node):
    def __init__(self, pub_hz: float = 25.0):
        super().__init__('pose_to_odom_republisher')
        self.pub_hz = pub_hz
        self.last_pose = None
        self.sub = self.create_subscription(PoseStamped, '/unitree_go2/pose', self.pose_cb, 10)
        self.pub = self.create_publisher(Odometry, '/unitree_go2/odom', 10)
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / float(self.pub_hz), self.timer_cb)

    def pose_cb(self, msg: PoseStamped):
        self.last_pose = msg

    def timer_cb(self):
        if self.last_pose is None:
            return
        now = self.get_clock().now().to_msg()

        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'odom'
        od.child_frame_id = 'unitree_go2/base_link'
        od.pose.pose = self.last_pose.pose
        od.pose.covariance = [0.05 if i % 7 == 0 else 0.0 for i in range(36)]
        self.pub.publish(od)

        t = od.pose.pose
        tf = TransformBroadcaster.TransformStamped()
        # Note: TransformBroadcaster.TransformStamped may not exist; build manually
        from geometry_msgs.msg import TransformStamped
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'unitree_go2/base_link'
        tf.transform.translation.x = od.pose.pose.position.x
        tf.transform.translation.y = od.pose.pose.position.y
        tf.transform.translation.z = od.pose.pose.position.z
        tf.transform.rotation = od.pose.pose.orientation
        self.br.sendTransform(tf)


def main(argv=sys.argv[1:]):
    rclpy.init()
    hz = 25.0
    if len(argv) >= 1:
        try:
            hz = float(argv[0])
        except Exception:
            pass
    node = PoseToOdomRepublisher(pub_hz=hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
