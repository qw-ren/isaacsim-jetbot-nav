#!/usr/bin/env python3
"""Evaluation-only sensor CDR capture for identical-input offline comparisons.

SQLite stores zlib-compressed ROS serialization. No publisher or command
service exists in this process; truth is never fed back to an estimator.
"""
import argparse
from functools import partial
import json
from pathlib import Path
import signal
import sqlite3
import time
import zlib

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, PointCloud2, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock

TOPICS = {
    '/lio/imu':Imu,
    '/evaluation/imu_ideal':Imu,
    '/lio/points_frame20ms':PointCloud2,
    '/lio/odometry':Odometry,
    '/truth/odom':Odometry,
    '/unitree_go2/odom':Odometry,
    '/clock':Clock,
    '/scan':LaserScan,
    '/global_costmap/costmap':OccupancyGrid,
    '/local_costmap/costmap':OccupancyGrid,
}


class Recorder(Node):
    def __init__(self, output, include_nav_cloud=False):
        super().__init__('lio_evaluation_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.root = output
        self.root.mkdir(parents=True, exist_ok=False)
        self.db = sqlite3.connect(self.root/'sensors.sqlite3')
        self.db.execute('PRAGMA journal_mode=WAL')
        self.db.execute('CREATE TABLE records(id INTEGER PRIMARY KEY,topic TEXT,stamp_ns INTEGER,wall REAL,payload BLOB)')
        self.db.execute('CREATE INDEX by_topic_stamp ON records(topic,stamp_ns)')
        self.counts = {}
        self.last_commit = time.monotonic()
        topics = dict(TOPICS)
        if include_nav_cloud:
            topics['/unitree_go2/lidar/point_cloud'] = PointCloud2
        reliable_inputs = {'/lio/imu','/evaluation/imu_ideal','/lio/points_frame20ms',
                           '/lio/odometry','/truth/odom','/unitree_go2/odom'}
        self.subs = [self.create_subscription(kind, topic, partial(self.capture, topic),
                     QoSProfile(depth=20 if kind is PointCloud2 else 1000,reliability=ReliabilityPolicy.RELIABLE)
                     if topic in reliable_inputs else qos_profile_sensor_data)
                     for topic,kind in topics.items()]
        self.fault = None

    def capture(self, topic, message):
        stamp = message.clock if topic == '/clock' else message.header.stamp
        ns = stamp.sec*1000000000+stamp.nanosec
        data = zlib.compress(serialize_message(message), level=1)
        self.db.execute('INSERT INTO records(topic,stamp_ns,wall,payload) VALUES(?,?,?,?)',
                        (topic, ns, time.monotonic(), data))
        self.counts[topic] = self.counts.get(topic, 0)+1
        if time.monotonic()-self.last_commit > 1.:
            self.db.commit()
            self.last_commit = time.monotonic()
            (self.root/'PROGRESS.json').write_text(json.dumps(dict(counts=self.counts,latest_stamp_ns=ns))+'\n')
            size = sum(p.stat().st_size for p in self.root.glob('sensors.sqlite3*'))
            if size > 15*1024**3:
                raise RuntimeError('Evaluation capture reached 15GiB limit')

    def close(self):
        self.db.commit()
        self.db.execute('PRAGMA wal_checkpoint(TRUNCATE)')
        self.db.close()
        (self.root/'SUMMARY.json').write_text(json.dumps(dict(counts=self.counts, fault=self.fault,
            serialization='ROS2 CDR with zlib level1 in SQLite',truth_usage='evaluation only; no publishers'), indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--include-nav-cloud', action='store_true')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Recorder(args.output, args.include_nav_cloud)
    stopped = [False]
    signal.signal(signal.SIGINT, lambda *_:stopped.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_:stopped.__setitem__(0, True))
    try:
        while rclpy.ok() and not stopped[0]:
            rclpy.spin_once(node, timeout_sec=.1)
    except Exception as exc:
        node.fault = str(exc)
        raise
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
