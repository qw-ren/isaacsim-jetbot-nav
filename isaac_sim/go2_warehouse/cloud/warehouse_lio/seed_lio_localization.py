"""Bounded acknowledged initial guess, before any controller is armed."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


class Seed(Node):
    def __init__(self):
        super().__init__('lio_localization_initial_guess',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.odom = None
        self.pose = None
        self.pub = self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.odom_sub = self.create_subscription(Odometry,'/unitree_go2/odom',self.odometry,qos_profile_sensor_data)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.localized,qos_profile_sensor_data)
        self.life = self.create_client(GetState,'/amcl/get_state')

    def odometry(self,m):
        self.odom = m

    def localized(self,m):
        self.pose = m

    def wait(self,seconds,predicate):
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            if predicate():
                return True
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pose',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    args = parser.parse_args()
    configured = json.loads(args.pose.read_text())
    if not configured['source'].startswith(('SLAM_','CONFIGURED_CAD_')):
        raise SystemExit('Initial guess source is not explicitly declared')
    rclpy.init(args=[])
    n = Seed()
    try:
        if not n.life.wait_for_service(timeout_sec=30):
            raise RuntimeError('AMCL service unavailable')
        active=False
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            future=n.life.call_async(GetState.Request())
            if n.wait(min(5.,max(0.,deadline-time.monotonic())),lambda:future.done()):
                if future.result().current_state.id==3:
                    active=True;break
            else:
                n.life.remove_pending_request(future)
                future.cancel()
            n.wait(.2,lambda:False)
        if not active:
            raise RuntimeError('AMCL active state not acknowledged within 30s; controller remains disarmed')
        if not n.wait(20,lambda:n.odom is not None and n.pub.get_subscription_count()>0):
            raise RuntimeError('Estimated odom or initial-pose subscriber missing')
        for attempt in range(1,4):
            n.pose = None
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            # The latest estimated odom timestamp already has a corresponding
            # TF sample; don't ask AMCL to transform a future wall-clock guess.
            message.header.stamp = n.odom.header.stamp
            message.pose.pose.position.x = configured['x']
            message.pose.pose.position.y = configured['y']
            message.pose.pose.orientation.z = math.sin(configured['yaw']/2)
            message.pose.pose.orientation.w = math.cos(configured['yaw']/2)
            message.pose.covariance[0] = message.pose.covariance[7] = .25**2
            message.pose.covariance[35] = .15**2
            n.pub.publish(message)
            if n.wait(5,lambda:n.pose is not None):
                pose = n.pose.pose.pose
                report = dict(source=configured,attempts=attempt,acknowledged=True,
                    accepted_amcl_pose=dict(x=pose.position.x,y=pose.position.y),
                    covariance_diag=[.25**2,.25**2,.15**2],continuous_truth_injection=False,
                    timestamp_source='latest estimated odom',controller_armed=False)
                args.report.parent.mkdir(parents=True,exist_ok=True)
                with args.report.open('x') as f:json.dump(report,f,indent=2)
                print(json.dumps(report),flush=True)
                return
        raise RuntimeError('AMCL did not acknowledge the configured initial guess')
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
