"""ROS2 感知节点：把 perception 核心库接到 Isaac Sim 的 ROS2 Bridge 上。

订阅：
    <image_topic>         sensor_msgs/Image      彩色图（Isaac Sim: /rgb 或 /camera/color/image_raw）
    <depth_topic>         sensor_msgs/Image      深度图（可关闭）
    <camera_info_topic>   sensor_msgs/CameraInfo 内参
    <scan_topic>          sensor_msgs/LaserScan  单线激光
    /tf /tf_static        TF2                    base_link / 相机光学系 / 激光系 / 地图

发布：
    /perception/detections    std_msgs/String  2D 检测（JSON）
    /perception/objects_2d    std_msgs/String  类别 + 平面 XY（JSON）
    /perception/object_poses  std_msgs/String  目标类别 + 世界坐标（JSON）
    /perception/markers       visualization_msgs/MarkerArray  给 RViz 看

为什么用 std_msgs/String(JSON) 而不是自定义消息：
团队里导航/RL 两条线可以零依赖先接上，避免自定义 msg 包编译顺序和版本问题；
接口稳定后再平移成 atongmu_msgs 自定义消息（字段与此处 JSON 一一对应）。
"""
from __future__ import annotations

import json
import time
import uuid
from collections import deque

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from atongmu_perception import PerceptionConfig, PerceptionPipeline
from atongmu_perception.detector import YoloDetector
from atongmu_perception.geometry import CameraIntrinsics, make_transform
from atongmu_perception.planar import json_finite, planar_observations


def transform_to_matrix(msg: TransformStamped) -> np.ndarray:
    t = msg.transform.translation
    q = msg.transform.rotation
    return make_transform([t.x, t.y, t.z], quaternion_xyzw=[q.x, q.y, q.z, q.w])


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("atongmu_perception")

        # ---------------- 参数 ----------------
        self.declare_parameter("config_file", "")            # perception/configs/perception.yaml 的路径
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("use_depth", True)
        self.declare_parameter("use_scan", True)
        self.declare_parameter("use_tf_extrinsics", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")  # 必须是光学系（z 朝前）
        self.declare_parameter("laser_frame", "laser_frame")
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("sync_slop", 0.06)
        self.declare_parameter("scan_slop", 0.06)
        self.declare_parameter("require_map", False)
        self.declare_parameter("publish_legacy", True)

        p = self.get_parameter
        self.image_topic = p("image_topic").value
        self.depth_topic = p("depth_topic").value
        self.camera_info_topic = p("camera_info_topic").value
        self.scan_topic = p("scan_topic").value
        self.use_depth = bool(p("use_depth").value)
        self.use_scan = bool(p("use_scan").value)
        self.use_tf_extrinsics = bool(p("use_tf_extrinsics").value)
        self.base_frame = p("base_frame").value
        self.map_frame = p("map_frame").value
        self.camera_frame = p("camera_frame").value
        self.laser_frame = p("laser_frame").value
        self.sync_slop = float(p("sync_slop").value)
        self.scan_slop = float(p("scan_slop").value)
        self.require_map = bool(p("require_map").value)
        self.publish_legacy = bool(p("publish_legacy").value)
        self.session_id = uuid.uuid4().hex
        self.sequence = 0
        self.last_stamp = None
        self.pending_frames = deque(maxlen=5)
        self.tf_wait_sec = .25

        # ---------------- 感知核心 ----------------
        cfg_path = p("config_file").value
        if cfg_path:
            self.cfg = PerceptionConfig.from_yaml(cfg_path)
            self.get_logger().info("已加载配置: %s" % cfg_path)
        else:
            self.cfg = PerceptionConfig()
            self.get_logger().warn("未指定 config_file，使用内置默认参数")
        self.pipeline = PerceptionPipeline(self.cfg, detector=YoloDetector.from_config(self.cfg.detector))
        self.model_identity = self.pipeline.detector.identity

        # ---------------- ROS 设施 ----------------
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_scan = None
        self.latest_info = None
        self._warned = set()
        # Use wall/steady time so queued frames expire even if simulation pauses.
        self.retry_timer = self.create_timer(
            .02, self._retry_pending, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.det_pub = self.create_publisher(String, "/perception/detections", 10)
        self.pose_pub = self.create_publisher(String, "/perception/object_poses", 10)
        self.planar_pub = self.create_publisher(String, "/perception/objects_2d", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/markers", 10) if p("publish_markers").value else None

        from message_filters import ApproximateTimeSynchronizer, Subscriber

        image_sub = Subscriber(self, Image, self.image_topic, qos_profile=qos_profile_sensor_data)
        if self.use_depth:
            depth_sub = Subscriber(self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data)
            self.sync = ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=5, slop=self.sync_slop)
            self.sync.registerCallback(self.image_depth_cb)
        else:
            image_sub.registerCallback(lambda msg: self.image_depth_cb(msg, None))

        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, qos_profile_sensor_data)
        if self.use_scan:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)

        self.get_logger().info(
            "感知节点已启动: image=%s depth=%s scan=%s device=%s"
            % (self.image_topic, self.depth_topic if self.use_depth else "off",
               self.scan_topic if self.use_scan else "off", self.cfg.detector.device)
        )

    # ------------------------------------------------------------------ #
    def camera_info_cb(self, msg: CameraInfo) -> None:
        if (msg.header.frame_id != self.camera_frame or not np.all(np.isfinite(msg.k))
                or msg.k[0] <= 0 or msg.k[4] <= 0 or msg.width <= 0 or msg.height <= 0):
            return
        if self.latest_info is None or (list(msg.k), msg.width, msg.height) != (list(self.latest_info.k), self.latest_info.width, self.latest_info.height):
            self.pipeline.set_intrinsics(CameraIntrinsics.from_camera_info(msg))
            self.get_logger().info("内参已更新: fx=%.1f fy=%.1f cx=%.1f cy=%.1f  %dx%d"
                                   % (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height))
        self.latest_info = msg

    def scan_cb(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        # Isaac Sim 无命中时给 0 或 inf，两种都当无效处理
        ranges = np.where((ranges > msg.range_min) & (ranges < msg.range_max), ranges, np.inf)
        self.latest_scan = (ranges, float(msg.angle_min), float(msg.angle_increment),
                            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            msg.header.stamp, msg.header.frame_id)

    # ------------------------------------------------------------------ #
    def _lookup(self, target: str, source: str, stamp):
        try:
            return transform_to_matrix(self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp)))
        except TransformException as exc:
            if (target, source) not in self._warned:
                self._warned.add((target, source))
                self.get_logger().warn("采集时间 TF 不可用 %s <- %s: %s" % (target, source, exc))
            return None

    def _update_transforms(self, stamp) -> bool:
        if not self.use_tf_extrinsics:
            return True
        T_base_cam = self._lookup(self.base_frame, self.camera_frame, stamp)
        if T_base_cam is None:
            return False
        self.pipeline.set_transforms(T_base_camera=T_base_cam)
        return True

    # ------------------------------------------------------------------ #
    def image_depth_cb(self, image_msg: Image, depth_msg) -> None:
        stamp = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        if stamp == 0 and self.last_stamp is None:
            return  # simulation has not started; Time(0) would mean latest TF
        if stamp <= 0 or (self.last_stamp is not None and stamp < self.last_stamp):
            self.get_logger().error("仿真时间无效或回退，请重启感知节点", throttle_duration_sec=5.0)
            self.last_stamp = float("inf")  # latch until node restart after simulation reset
            self.pending_frames.clear()
            return
        if self.last_stamp == stamp:
            return
        self.last_stamp = stamp
        self.pending_frames.append((image_msg, depth_msg, time.monotonic()))
        self._retry_pending()

    def _retry_pending(self) -> None:
        """Give timestamped TF a bounded chance to arrive; never use latest TF."""
        while self.pending_frames:
            image_msg, depth_msg, received = self.pending_frames[0]
            if time.monotonic() - received > self.tf_wait_sec:
                self.pending_frames.popleft()
                continue
            if self._process_frame(image_msg, depth_msg) is False:
                return
            self.pending_frames.popleft()

    def _process_frame(self, image_msg: Image, depth_msg):
        stamp = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        if (self.latest_info is None or image_msg.header.frame_id != self.camera_frame
                or (image_msg.width, image_msg.height) != (self.latest_info.width, self.latest_info.height)):
            self.get_logger().warn("等待匹配的 CameraInfo / optical frame", throttle_duration_sec=5.0)
            return False
        if depth_msg is not None and (
                depth_msg.header.frame_id != image_msg.header.frame_id
                or (depth_msg.width, depth_msg.height) != (image_msg.width, image_msg.height)
                or depth_msg.header.stamp != image_msg.header.stamp):
            self.get_logger().warn("拒绝非同帧 RGB-D", throttle_duration_sec=5.0)
            return
        if not self._update_transforms(image_msg.header.stamp):
            return False
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("彩色图转换失败: %s" % exc)
            return

        depth = None
        depth_scale = None
        if depth_msg is not None:
            try:
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error("深度图转换失败: %s" % exc)
            else:
                if depth_msg.encoding == "16UC1":     # 毫米
                    depth_scale = 0.001
                elif depth_msg.encoding == "32FC1":   # 米
                    depth_scale = 1.0
                else:
                    self.get_logger().warn("拒绝未知深度编码 %s" % depth_msg.encoding, throttle_duration_sec=10.0)
                    return
                if depth is not None and depth.ndim == 3:
                    depth = depth[:, :, 0]

        T_map_base = self._lookup(self.map_frame, self.base_frame, image_msg.header.stamp)
        if T_map_base is None and self.require_map:
            return False

        scan = None
        if self.use_scan and self.latest_scan is not None:
            ranges, amin, ainc, scan_stamp, scan_time, scan_frame = self.latest_scan
            if abs(stamp - scan_stamp) <= self.scan_slop and scan_frame == self.laser_frame:
                try:
                    tf = self.tf_buffer.lookup_transform_full(
                        self.camera_frame, Time.from_msg(image_msg.header.stamp),
                        scan_frame, Time.from_msg(scan_time), "odom")
                    self.pipeline.set_transforms(
                        T_base_camera=self.pipeline.localizer.T_base_camera,
                        T_camera_laser=transform_to_matrix(tf))
                    scan = (ranges, amin, ainc)
                except TransformException:
                    pass  # no time-aligned laser transform: use depth/ground only

        result = self.pipeline.process(
            image=image,
            depth=depth,
            depth_scale=depth_scale,
            scan=scan,
            T_map_base=T_map_base,
            stamp=stamp,
            base_frame=self.base_frame,
            output_frame=self.map_frame,
            estimate_sizes=False,
        )

        det_payload = {
            "stamp": result.stamp,
            "frame_id": image_msg.header.frame_id,
            "image_size": list(result.image_size) if result.image_size else None,
            "latency_ms": round(result.latency_ms, 2),
            "detections": [d.to_dict() for d in result.detections],
        }
        pose_payload = {
            "stamp": result.stamp,
            "frame_id": result.frame_id,
            "latency_ms": round(result.latency_ms, 2),
            "objects": [o.to_dict() for o in result.objects],
        }
        self.det_pub.publish(String(data=json.dumps(json_finite(det_payload), ensure_ascii=False, allow_nan=False)))
        if self.publish_legacy:
            self.pose_pub.publish(String(data=json.dumps(json_finite(pose_payload), ensure_ascii=False, allow_nan=False)))
        self.sequence += 1
        planar_payload = planar_observations(result, self.session_id, self.sequence)
        planar_payload["model"] = getattr(self, "model_identity", None)
        planar_payload["time_domain"] = "ros_sim" if self.get_parameter("use_sim_time").value else "ros_system"
        self.planar_pub.publish(String(data=json.dumps(
            planar_payload, ensure_ascii=False, allow_nan=False)))
        if self.marker_pub is not None:
            self.marker_pub.publish(self._markers(result, stamp))

    # ------------------------------------------------------------------ #
    def _markers(self, result, stamp: float) -> MarkerArray:
        array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, obj in enumerate(result.objects):
            m = Marker()
            m.header.frame_id = result.frame_id
            m.header.stamp = now
            m.ns = "perception"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = obj.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.18
            m.color.a = 0.85
            m.color.r, m.color.g, m.color.b = 0.95, 0.35, 0.35
            array.markers.append(m)

            text = Marker()
            text.header = m.header
            text.ns = "perception_labels"
            text.id = 1000 + i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = obj.position[0], obj.position[1]
            text.pose.position.z = obj.position[2] + 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color.a = 1.0
            text.color.r = text.color.g = text.color.b = 1.0
            rng = obj.localization.range_m if obj.localization else float("nan")
            text.text = "%s %.2fm" % (obj.class_name, rng)
            array.markers.append(text)
        return array


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
