#!/usr/bin/env python3
"""Simulation-only exclusive velocity gate with wall-time fail-stop."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool


class VelocityGuard(Node):
    def __init__(self, boxes):
        super().__init__("go2_sim_velocity_guard", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.boxes = np.array([[b["min"][:2], b["max"][:2]] for b in boxes])
        self.armed = False
        self.reason = "DISARMED"
        self.command = np.zeros(2)
        self.last = {key: 0.0 for key in ("input", "odom", "scan", "clock")}
        self.pose = None
        self.scan = None
        self.scan_stamp = self.odom_stamp = self.sim = None
        self.front_min = None
        self.front_angle = None
        self.publisher = self.create_publisher(Twist, "/unitree_go2/cmd_vel", 10)
        self.status = self.create_publisher(String, "/nav_m1/guard_state", 10)
        self.input = self.create_subscription(Twist, "/nav_m1/cmd_vel", self.command_cb, 10)
        self.odom = self.create_subscription(Odometry, "/unitree_go2/odom", self.odom_cb, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.clock_sub = self.create_subscription(ClockMessage, "/clock", self.clock_cb, qos_profile_sensor_data)
        self.enable = self.create_service(SetBool, "/nav_m1/enable", self.enable_cb)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self.tick, clock=self.wall_clock)

    def command_cb(self, message):
        values = np.array([message.linear.x, message.linear.y, message.angular.z])
        if not np.isfinite(values).all() or abs(values[1]) > 1e-6 or values[0] < 0:
            self.armed, self.reason = False, "INVALID_COMMAND"
            self.command[:] = 0
            return
        self.command[:] = [min(values[0], 1.0), float(np.clip(values[2], -.6, .6))]
        self.last["input"] = time.monotonic()

    def odom_cb(self, message):
        if message.header.frame_id != "odom" or message.child_frame_id != "unitree_go2/base_link":
            self.armed, self.reason = False, "ODOM_FRAME_ERROR"
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        tilt = math.acos(float(np.clip(1-2*(q.x*q.x+q.y*q.y), -1, 1)))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        values = [p.x, p.y, p.z, yaw, tilt]
        if not np.isfinite(values).all():
            return
        self.odom_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.pose = values
        self.last["odom"] = time.monotonic()

    def scan_cb(self, message):
        if message.header.frame_id != "unitree_go2/scan_ground":
            return
        self.scan_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.scan = message
        self.last["scan"] = time.monotonic()

    def clock_cb(self, message):
        self.sim = message.clock.sec + message.clock.nanosec * 1e-9
        self.last["clock"] = time.monotonic()

    def fault(self, now):
        for key, timeout in (("odom", .5), ("scan", 1.0), ("clock", .5), ("input", .35)):
            if key == "scan":
                # .20 sim seconds bounds travelled distance; 1.0 wall second is a hard cap.
                # Clock/odom/input retain their independent .5/.5/.35 wall deadlines.
                if self.sim is None or self.scan_stamp is None or now-self.last[key] > timeout or self.sim-self.scan_stamp > .20:
                    return "SCAN_TIMEOUT"
            elif now - self.last[key] > timeout:
                return key.upper() + "_TIMEOUT"
        if self.pose is None or self.pose[2] < .20 or self.pose[2] > .85 or self.pose[4] > .5:
            return "POSTURE_LIMIT"
        if [p.node_name for p in self.get_publishers_info_by_topic("/unitree_go2/cmd_vel")] != [self.get_name()]:
            return "OUTPUT_PUBLISHER_CONFLICT"
        if self.get_publishers_info_by_topic("/cmd_vel") or len(self.get_publishers_info_by_topic("/nav_m1/cmd_vel")) != 1:
            return "INPUT_PUBLISHER_CONFLICT"
        return None

    def geometry_clearance(self, v, w):
        x, y, _, yaw, _ = self.pose
        t = np.linspace(0, 1.5, 16)
        if abs(w) < 1e-6:
            points = np.c_[x+v*t*math.cos(yaw), y+v*t*math.sin(yaw)]
        else:
            points = np.c_[x+v/w*(np.sin(yaw+w*t)-math.sin(yaw)),
                           y-v/w*(np.cos(yaw+w*t)-math.cos(yaw))]
        delta = np.maximum(np.maximum(self.boxes[:, 0, None, :] - points,
                                      points-self.boxes[:, 1, None, :]), 0)
        return float(np.linalg.norm(delta, axis=-1).min())

    def enable_cb(self, request, response):
        if not request.data:
            self.armed, self.reason = False, "DISARMED"
            self.command[:] = 0
            self.publisher.publish(Twist())
            response.success, response.message = True, "Zero velocity; explicitly disarmed"
            return response
        fault = self.fault(time.monotonic())
        self.armed = fault is None
        self.reason = fault or "ENABLED"
        response.success, response.message = self.armed, self.reason
        return response

    def tick(self):
        clearance = None
        self.front_min = self.front_angle = None
        if self.armed:
            problem = self.fault(time.monotonic())
            if problem is None:
                clearance = self.geometry_clearance(*self.command)
                if clearance < .55:
                    problem = "PREDICTED_STATIC_CLEARANCE"
                elif self.command[0] > 0 and self.scan is not None:
                    angles = self.scan.angle_min + np.arange(len(self.scan.ranges))*self.scan.angle_increment
                    r = np.array(self.scan.ranges)
                    mask = (np.abs(angles) < .35) & np.isfinite(r)
                    front = r[mask]
                    if len(front):
                        i = int(np.argmin(front))
                        self.front_min = float(front[i])
                        self.front_angle = float(angles[mask][i])
                    if len(front) and front.min() < .55 + .5*self.command[0]:
                        problem = "FRONT_SCAN_CLEARANCE"
            if problem:
                self.armed, self.reason = False, problem
        output = self.command if self.armed else np.zeros(2)
        message = Twist()
        message.linear.x, message.angular.z = map(float, output)
        self.publisher.publish(message)
        status = String()
        status.data = json.dumps({"wall": time.monotonic(), "armed": self.armed, "reason": self.reason,
                                  "output": output.tolist(), "predicted_center_clearance_m": clearance, "pose": self.pose,
                                  "scan_wall_age_s": time.monotonic()-self.last["scan"], "scan_sim_age_s": self.sim-self.scan_stamp if self.sim is not None and self.scan_stamp is not None else None, "scan_wall_limit_s": 1.0, "scan_sim_limit_s": .20,
                                  "scan_stamp": self.scan_stamp, "odom_stamp": self.odom_stamp, "sim": self.sim,
                                  "front_min_range_m": self.front_min, "front_angle_rad": self.front_angle,
                                  "input_command": self.command.tolist()})
        self.status.publish(status)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--boxes", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = VelocityGuard(json.loads(args.boxes.read_text()))
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
