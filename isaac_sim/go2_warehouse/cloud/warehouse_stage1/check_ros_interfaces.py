#!/usr/bin/env python3
"""Measure warehouse simulation interfaces; motion mode sends bounded test commands."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage


class InterfaceProbe(Node):
    def __init__(self, output, command_topic):
        super().__init__("warehouse_interface_probe")
        self.output = output
        self.streams = {}
        self.edges = set()
        self.odom = None
        self.odom_samples = []
        self.clock = 0.0
        self.publisher = self.create_publisher(Twist, command_topic, 10)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        specs = {
            "/clock": Clock,
            "/unitree_go2/odom": Odometry,
            "/unitree_go2/pose": PoseStamped,
            "/unitree_go2/lidar/point_cloud": PointCloud2,
            "/unitree_go2/front_cam/color_image": Image,
            "/unitree_go2/front_cam/depth_image": Image,
            "/unitree_go2/front_cam/info": CameraInfo,
            "/tf": TFMessage,
        }
        self.subscriptions_keepalive = [
            self.create_subscription(kind, topic, lambda msg, t=topic: self.receive(t, msg), qos)
            for topic, kind in specs.items()
        ]
        static_qos = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_keepalive.append(self.create_subscription(
            TFMessage, "/tf_static", lambda msg: self.receive("/tf_static", msg), static_qos))

    def receive(self, topic, msg):
        now = time.monotonic()
        row = self.streams.setdefault(topic, {"count": 0, "first_arrival": now, "last_arrival": now})
        row["count"] += 1
        row["last_arrival"] = now
        if hasattr(msg, "header"):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            row["frame"] = msg.header.frame_id
            if "last_stamp" in row and stamp < row["last_stamp"]:
                row["stamp_regressions"] = row.get("stamp_regressions", 0) + 1
            row["last_stamp"] = stamp
        if isinstance(msg, Clock):
            self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9
            row["last_sim_time"] = self.clock
        elif isinstance(msg, Odometry):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            tilt = math.acos(float(np.clip(1 - 2 * (q.x * q.x + q.y * q.y), -1, 1)))
            v = msg.twist.twist
            self.odom = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw,
                         "tilt": tilt, "speed": math.hypot(v.linear.x, v.linear.y),
                         "yaw_rate": v.angular.z, "stamp": row["last_stamp"]}
            self.odom_samples.append(dict(self.odom, arrival=now))
            row["child_frame"] = msg.child_frame_id
        elif isinstance(msg, TFMessage):
            self.edges.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms)
        elif isinstance(msg, CameraInfo):
            row.update(width=msg.width, height=msg.height, focal_x=msg.k[0], focal_y=msg.k[4])
        elif isinstance(msg, PointCloud2) and row["count"] % 10 == 1:
            points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            row.update(points=int(len(points)), finite_points=int(np.isfinite(points).all(axis=1).sum()))
            if len(points):
                row["xyz_min"] = np.min(points, axis=0).astype(float).tolist()
                row["xyz_max"] = np.max(points, axis=0).astype(float).tolist()
        elif isinstance(msg, Image) and row["count"] == 1:
            row.update(width=msg.width, height=msg.height, encoding=msg.encoding, bytes=len(msg.data))
            if msg.encoding in ("rgb8", "rgba8", "bgr8", "bgra8"):
                channels = 4 if "a8" in msg.encoding else 3
                pixels = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
                pixels = pixels[:, :msg.width * channels].reshape(msg.height, msg.width, channels)[..., :3]
                if msg.encoding.startswith("bgr"):
                    pixels = pixels[..., ::-1]
                row["pixel_std"] = float(pixels.std())
                PILImage.fromarray(pixels).save(self.output / "front_rgb.png")
            elif msg.encoding == "32FC1":
                dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
                depth = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.step // 4)[:, :msg.width]
                good = depth[np.isfinite(depth) & (depth > 0)]
                row["valid_depth_pixels"] = int(good.size)
                if good.size:
                    row["depth_range_m"] = [float(good.min()), float(good.max())]
                    preview = np.nan_to_num(depth, nan=0, posinf=0, neginf=0)
                    preview = np.clip(preview / min(float(good.max()), 20.0) * 255, 0, 255).astype(np.uint8)
                    PILImage.fromarray(preview).save(self.output / "front_depth.png")

    def phase(self, seconds, linear=0.0, angular=0.0):
        sim_start = self.clock
        deadline = time.monotonic() + max(30, seconds * 20)
        next_command = 0.0
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        while self.clock - sim_start < seconds:
            now = time.monotonic()
            if now >= deadline:
                raise RuntimeError("Simulation time stalled during test phase")
            if self.odom and (self.odom["z"] < 0.15 or self.odom["tilt"] > 0.7):
                raise RuntimeError("Robot posture failed; aborting motion test")
            if now >= next_command:
                self.publisher.publish(message)
                next_command = now + 0.1
            rclpy.spin_once(self, timeout_sec=0.05)
        return dict(self.odom) if self.odom else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--motion", action="store_true")
    parser.add_argument("--cmd-topic", default="/unitree_go2/cmd_vel")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    probe = InterfaceProbe(args.output, args.cmd_topic)
    result = {"motion_requested": args.motion, "command_topic": args.cmd_topic}
    try:
        required = {"/clock", "/unitree_go2/odom", "/unitree_go2/lidar/point_cloud",
                    "/unitree_go2/front_cam/color_image", "/unitree_go2/front_cam/depth_image"}
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and not required.issubset(probe.streams):
            rclpy.spin_once(probe, timeout_sec=0.2)
        if not required.issubset(probe.streams):
            raise RuntimeError("Missing streams: " + str(sorted(required - probe.streams.keys())))
        start = probe.phase(4)
        result["standing"] = start
        if args.motion:
            forward = probe.phase(4, linear=0.25)
            result["forward_end"] = forward
            result["forward_distance_m"] = math.hypot(forward["x"] - start["x"], forward["y"] - start["y"])
            turn = probe.phase(3, angular=0.35)
            result["turn_end"] = turn
            result["turn_angle_rad"] = math.atan2(math.sin(turn["yaw"] - forward["yaw"]),
                                                   math.cos(turn["yaw"] - forward["yaw"]))
        result["stopped"] = probe.phase(4)
    except Exception as exc:
        result["error"] = str(exc)
    finally:
        stop = Twist()
        for _ in range(20):
            probe.publisher.publish(stop)
            rclpy.spin_once(probe, timeout_sec=0.05)
        for row in probe.streams.values():
            span = row["last_arrival"] - row["first_arrival"]
            row["wall_rate_hz"] = (row["count"] - 1) / span if span > 0 else 0
        result["topics"] = probe.streams
        result["tf_edges"] = sorted([list(edge) for edge in probe.edges])
        rgb = probe.streams.get("/unitree_go2/front_cam/color_image", {})
        depth = probe.streams.get("/unitree_go2/front_cam/depth_image", {})
        info = probe.streams.get("/unitree_go2/front_cam/info", {})
        cloud = probe.streams.get("/unitree_go2/lidar/point_cloud", {})
        expected_tf = {("map", "odom"), ("odom", "unitree_go2/base_link"),
                       ("unitree_go2/base_link", "unitree_go2/lidar_frame"),
                       ("unitree_go2/base_link", "unitree_go2/front_cam")}
        result["checks"] = {
            "no_runtime_error": "error" not in result,
            "rgb_nonblank": rgb.get("pixel_std", 0) > 5,
            "depth_valid": depth.get("valid_depth_pixels", 0) > 1000,
            "pointcloud_valid": cloud.get("finite_points", 0) > 100,
            "camera_info_matches_image_frame": bool(info) and info.get("frame") == rgb.get("frame"),
            "camera_intrinsics_valid": info.get("focal_x", 0) > 0 and info.get("focal_y", 0) > 0,
            "tf_tree_present": expected_tf.issubset(probe.edges),
            "sim_time_advances": probe.clock > 0,
            "timestamps_monotonic": all(r.get("stamp_regressions", 0) == 0 for r in probe.streams.values()),
        }
        if args.motion:
            result["checks"].update(forward_motion=result.get("forward_distance_m", 0) > 0.3,
                                    turning_motion=abs(result.get("turn_angle_rad", 0)) > 0.2,
                                    final_speed_low=result.get("stopped", {}).get("speed", 99) < 0.1)
        result["checks"] = {key: bool(value) for key, value in result["checks"].items()}
        result["passed"] = all(result["checks"].values())
        (args.output / "interfaces.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps({k: v for k, v in result.items() if k != "topics"}, indent=2), flush=True)
        probe.destroy_node()
        rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
