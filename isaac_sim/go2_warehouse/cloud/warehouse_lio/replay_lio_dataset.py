#!/usr/bin/env python3
"""Publish identical recorded LiDAR with ideal/noisy IMU in a separate domain.

Only sensor rows are read. Truth and online navigation outputs are not replayed.
FAST-LIO results are captured for subsequent offline evaluation, never control.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import struct
import time
import zlib

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu,PointCloud2
from rcl_interfaces.srv import GetParameters


class Replay(Node):
    def __init__(self):
        super().__init__('lio_sensor_replay',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.imu = self.create_publisher(Imu,'/lio/imu',1000)
        self.points = self.create_publisher(PointCloud2,'/lio/points_frame20ms',100)
        self.clock = self.create_publisher(Clock,'/clock',10)
        self.poses = []
        self.sub = self.create_subscription(Odometry,'/lio/odometry',self.odometry,qos_profile_sensor_data)
        self.parameters = self.create_client(GetParameters,'/fastlio_recorded_replay/get_parameters')

    def odometry(self,m):
        p,q = m.pose.pose.position,m.pose.pose.orientation
        self.poses.append(dict(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                               xyz=[p.x,p.y,p.z],xyzw=[q.x,q.y,q.z,q.w]))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--database',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--imu-topic',choices=['/lio/imu','/evaluation/imu_ideal'],required=True)
    parser.add_argument('--rate',type=float,default=2.)
    parser.add_argument('--begin',type=float,default=3.)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') == '41' or not os.environ.get('ROS_DOMAIN_ID'):
        raise SystemExit('Replay requires an explicit separate domain, never live domain41')
    if not 0.1 <= args.rate <= 4.:
        raise SystemExit('Replay rate outside supported range')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[])
    node = Replay()
    db = sqlite3.connect(f'file:{args.database}?mode=ro',uri=True)
    stats = dict(imu_topic=args.imu_topic,rate=args.rate,domain=os.environ['ROS_DOMAIN_ID'],
                 truth_replayed=False,counts={},source_database=str(args.database),requested_begin_s=args.begin)
    error = None
    try:
        deadline = time.monotonic()+25
        while time.monotonic()<deadline:
            if node.get_subscriptions_info_by_topic('/lio/imu') and node.get_subscriptions_info_by_topic('/lio/points_frame20ms'):
                break
            rclpy.spin_once(node,timeout_sec=.1)
        else:
            raise RuntimeError('FAST-LIO input subscribers not ready')
        if not node.parameters.wait_for_service(timeout_sec=10):
            raise RuntimeError('Replay estimator parameter service not ready')
        request = GetParameters.Request()
        request.names = ['reliable_lidar']
        future = node.parameters.call_async(request)
        deadline = time.monotonic()+10
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        if not future.done() or not future.result().values[0].bool_value:
            raise RuntimeError('Reliable LiDAR transport is not confirmed')
        stats['reliable_lidar_confirmed'] = True
        ends = [db.execute('SELECT MAX(stamp_ns) FROM records WHERE topic=?',(topic,)).fetchone()[0]
                for topic in ['/lio/imu','/evaluation/imu_ideal']]
        common_end = min(x for x in ends if x is not None)
        stats['common_sensor_end_s'] = common_end/1e9
        rows = db.execute('SELECT topic,stamp_ns,payload FROM records WHERE topic IN (?,?) AND stamp_ns>=? AND stamp_ns<=? ORDER BY stamp_ns,id',
                          (args.imu_topic,'/lio/points_frame20ms',round(args.begin*1e9),common_end))
        first = None
        started = time.monotonic()
        last_stamp = 0
        last_emitted = None
        last_source = None
        points_hash = hashlib.sha256()
        imu_stamps_hash = hashlib.sha256()
        for topic,stamp,payload in rows:
            if first is None:
                first = stamp
                started = time.monotonic()
            # Never catch up a disk/deserialize delay by bursting IMU messages
            # into the estimator's finite subscription queue.
            due = time.monotonic() if last_emitted is None else last_emitted+(stamp-last_source)/1e9/args.rate
            while time.monotonic()<due:
                rclpy.spin_once(node,timeout_sec=min(.01,max(0.,due-time.monotonic())))
            clock = Clock()
            clock.clock.sec,clock.clock.nanosec = divmod(stamp,1000000000)
            node.clock.publish(clock)
            message = deserialize_message(zlib.decompress(payload),Imu if topic==args.imu_topic else PointCloud2)
            (node.imu if topic==args.imu_topic else node.points).publish(message)
            if topic==args.imu_topic:
                imu_stamps_hash.update(struct.pack('<q',stamp))
            else:
                points_hash.update(struct.pack('<q',stamp))
                points_hash.update(payload)
            stats['counts'][topic] = stats['counts'].get(topic,0)+1
            last_stamp = stamp
            last_source,last_emitted = stamp,time.monotonic()
            rclpy.spin_once(node,timeout_sec=0.)
        stats['last_input_stamp_s'] = last_stamp/1e9
        # Let the algorithm finish the final real sensor rows; no invented IMU.
        deadline = time.monotonic()+10
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        stats['wall_seconds'] = time.monotonic()-started
        stats['point_input_sha256'] = points_hash.hexdigest()
        stats['imu_timestamps_sha256'] = imu_stamps_hash.hexdigest()
        if not stats['counts'].get(args.imu_topic) or not stats['counts'].get('/lio/points_frame20ms') or len(node.poses)<2:
            raise RuntimeError('Missing recorded sensors or replayed LIO output')
    except Exception as exc:
        error = str(exc)
    finally:
        stats.update(error=error,output_poses=len(node.poses),last_output_stamp_s=node.poses[-1]['stamp'] if node.poses else None)
        (args.output/'odometry.json').write_text(json.dumps(dict(poses=node.poses,statistics=stats))+'\n')
        (args.output/'SUMMARY.json').write_text(json.dumps(stats,indent=2)+'\n')
        print(json.dumps(stats,indent=2),flush=True)
        db.close()
        node.destroy_node()
        rclpy.shutdown()
    return 1 if error else 0


if __name__ == '__main__':
    raise SystemExit(main())
