"""Restore the accepted LIO startup ordering: sensors settle before IMU init.

Uses native IMU/point data only, while no controller is armed. No truth input.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu,PointCloud2
from rclpy.qos import QoSProfile,ReliabilityPolicy


class Warmup(Node):
    def __init__(self):
        super().__init__('lio_sensor_warmup')
        self.rows = deque(maxlen=200)
        self.cloud_stamp = None
        qos = QoSProfile(depth=200,reliability=ReliabilityPolicy.RELIABLE)
        self.imu = self.create_subscription(Imu,'/lio/imu',self.sample,qos)
        self.cloud = self.create_subscription(PointCloud2,'/lio/points_frame20ms',self.points,
                                             QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE))

    def sample(self,m):
        a,g = m.linear_acceleration,m.angular_velocity
        self.rows.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,[a.x,a.y,a.z],[g.x,g.y,g.z]))

    def points(self,m):
        if m.width > 0 and len(m.data) == m.width*m.height*m.point_step:
            self.cloud_stamp = m.header.stamp.sec+m.header.stamp.nanosec*1e-9


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[])
    node = Warmup()
    deadline = time.monotonic()+140
    result = None
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if len(node.rows)<150 or node.cloud_stamp is None:
                continue
            stamp = node.rows[-1][0]
            gyro = np.linalg.norm([x[2] for x in node.rows],axis=1)
            acceleration = np.linalg.norm([x[1] for x in node.rows],axis=1)
            if stamp>=3.0 and node.cloud_stamp>=2.8 and float(np.percentile(gyro,95))<.08 and 8.<float(acceleration.mean())<12.:
                result = dict(imu_stamp_s=stamp,lidar_stamp_s=node.cloud_stamp,
                    stationary_gyro_p95_radps=float(np.percentile(gyro,95)),acceleration_norm_mean_mps2=float(acceleration.mean()),
                    samples=len(node.rows),truth_used=False,controller_armed=False,
                    purpose='settled sensor initialization, not an odometry-accuracy criterion')
                (args.output/'READY.json').write_text(json.dumps(result,indent=2)+'\n')
                print(json.dumps(result),flush=True)
                break
        if result is None:
            raise RuntimeError('Sensors failed to settle for LIO initialization')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
