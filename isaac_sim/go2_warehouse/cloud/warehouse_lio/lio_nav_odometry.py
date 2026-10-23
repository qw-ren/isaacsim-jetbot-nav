#!/usr/bin/env python3
"""Exclusive estimated odom/TF for an isolated navigation trial.

Inputs: /lio/odometry and stationary IMU gravity during initialization.
No ground-truth or wheel odometry subscription.
No extrapolation, repeated stale pose, fabricated timestamp or automatic reset.
The unchanged velocity gate stops if this output stops updating.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage

from lio_nav_math import LioNavConverter


class LioNavOdometry(Node):
    def __init__(self, args):
        super().__init__('go2_lio_nav_odometry', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.converter = LioNavConverter(args.anchor_xyz, args.anchor_yaw)
        self.anchor_xyz, self.anchor_yaw = args.anchor_xyz, args.anchor_yaw
        self.gravity_rows = []
        self.latest_lio = None
        self.gravity_ready = False
        self.output = Path(args.output)
        self.output.mkdir(parents=True, exist_ok=False)
        self.stream = (self.output/'odometry.jsonl').open('x')
        self.fault = None
        self.samples = 0
        self.pub = self.create_publisher(Odometry, '/unitree_go2/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_audit_sub = self.create_subscription(TFMessage, '/tf', self.audit_tf, 100)
        self.sub = self.create_subscription(Odometry, '/lio/odometry', self.callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(Imu, '/lio/imu', self.imu_callback, qos_profile_sensor_data)
        (self.output/'contract.json').write_text(json.dumps(dict(
            input=['/lio/odometry', '/lio/imu (stationary gravity alignment only)'], output='/unitree_go2/odom',
            parent='odom', child='unitree_go2/base_link', imu_to_base='identity mount verified in native_imu_adapter.py',
            anchor_xyz=args.anchor_xyz, anchor_yaw=args.anchor_yaw,
            anchor_source='fixed xyz/yaw case configuration plus initial sensor-only gravity alignment; no truth-based alignment',
            twist='consecutive estimated poses, expressed in body frame',
            source_timestamps_preserved=True, extrapolation=False,
            input_timing='explicit approximate 20ms packets; strict per-ray timing unverified',
            scope='independent LIO navigation experiment; stable default is unchanged'), indent=2)+'\n')

    def audit_tf(self, message):
        if self.fault is not None:
            return
        # This cloud rclpy executor passes only the message, without MessageInfo.
        # Audit graph sources together with the frozen Bridge remap; do not
        # pretend that an individual TFMessage contains publisher authority.
        allowed = {self.get_name(), 'go2_scan_ground_frame', 'amcl'}
        unknown = {x.node_name for x in self.get_publishers_info_by_topic('/tf')}-allowed-{'_NODE_NAME_UNKNOWN_'}
        if unknown:
            self.stop_output('Unexpected navigation TF publisher: '+repr(sorted(unknown)))
        if any(t.child_frame_id == 'unitree_go2/base_link' and t.header.frame_id != 'odom'
               for t in message.transforms):
            self.stop_output('Unexpected base TF parent')

    def imu_callback(self, message):
        if self.gravity_ready or self.latest_lio is None or self.fault is not None:
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        when, rotation = self.latest_lio
        if not 0. <= stamp-when <= .20:
            return
        a = message.linear_acceleration
        value = np.array([a.x, a.y, a.z])
        if not np.isfinite(value).all() or not 7. < np.linalg.norm(value) < 13.:
            return
        self.gravity_rows.append((stamp, rotation.apply(value)))
        if len(self.gravity_rows) >= 200 and stamp-self.gravity_rows[0][0] >= 1.:
            gravity = np.mean([x[1] for x in self.gravity_rows], axis=0)
            self.converter = LioNavConverter(self.anchor_xyz, self.anchor_yaw, gravity)
            self.gravity_ready = True
            (self.output/'GRAVITY_ANCHOR.json').write_text(json.dumps(dict(
                source='stationary native IMU transformed by concurrent LIO estimates',
                samples=len(self.gravity_rows), gravity_in_lio=gravity.tolist(),
                lio_to_odom_rotation_xyzw=self.converter.rotation.as_quat().tolist(),
                frozen_after_initialization=True, truth_used=False), indent=2)+'\n')

    def stop_output(self, reason):
        self.fault = reason
        (self.output/'FAULT.json').write_text(json.dumps(dict(error=self.fault, output_stopped=True), indent=2)+'\n')
        self.get_logger().error('LIO navigation output stopped: '+self.fault)

    def callback(self, message):
        if self.fault is not None:
            return
        try:
            if message.header.frame_id != 'camera_init' or message.child_frame_id != 'body':
                raise ValueError('Unexpected FAST-LIO output frames')
            publishers = self.get_publishers_info_by_topic('/unitree_go2/odom')
            others = [x.node_name for x in publishers
                      if x.node_name != self.get_name()]
            if others or len(publishers) > 1:
                raise ValueError('Conflicting odometry publisher: '+repr(others))
            p, q = message.pose.pose.position, message.pose.pose.orientation
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            self.latest_lio = (stamp, Rotation.from_quat([q.x,q.y,q.z,q.w]))
            if not self.gravity_ready:
                return
            result = self.converter.update(stamp, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            if result is None:
                return
            output = Odometry()
            output.header.stamp = message.header.stamp
            output.header.frame_id = 'odom'
            output.child_frame_id = 'unitree_go2/base_link'
            output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = map(float, result['xyz'])
            output.pose.pose.orientation.x, output.pose.pose.orientation.y, output.pose.pose.orientation.z, output.pose.pose.orientation.w = map(float, result['xyzw'])
            output.twist.twist.linear.x, output.twist.twist.linear.y, output.twist.twist.linear.z = map(float, result['linear_body'])
            output.twist.twist.angular.x, output.twist.twist.angular.y, output.twist.twist.angular.z = map(float, result['angular_body'])
            covariance = np.asarray(message.pose.covariance).reshape(6, 6)
            if not np.isfinite(covariance).all():
                raise ValueError('Nonfinite LIO covariance')
            transform = np.zeros((6, 6))
            transform[:3, :3] = transform[3:, 3:] = self.converter.rotation.as_matrix()
            output.pose.covariance = (transform@covariance@transform.T).ravel().tolist()
            # Zero twist covariance means unknown in nav_msgs; no calibrated
            # velocity uncertainty is claimed by this finite-difference adapter.
            self.pub.publish(output)
            t = TransformStamped()
            t.header = output.header
            t.child_frame_id = output.child_frame_id
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, result['xyz'])
            t.transform.rotation = output.pose.pose.orientation
            self.tf.sendTransform(t)
            self.samples += 1
            row = {k:(v.tolist() if isinstance(v, np.ndarray) else v) for k,v in result.items()}
            row['wall'] = time.monotonic()
            self.stream.write(json.dumps(row)+'\n')
            self.stream.flush()
        except Exception as exc:
            self.stop_output(str(exc))

    def close(self):
        self.stream.close()
        (self.output/'SUMMARY.json').write_text(json.dumps(dict(samples=self.samples, fault=self.fault), indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--anchor-xyz', nargs=3, type=float, required=True)
    parser.add_argument('--anchor-yaw', type=float, default=0.)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = LioNavOdometry(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        node.stop_output('Unhandled adapter error: '+str(exc))
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if node.fault else 0


if __name__ == '__main__':
    raise SystemExit(main())
