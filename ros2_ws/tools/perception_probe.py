#!/usr/bin/env python3
"""上云第一件事：探测 Isaac Sim 发出来的 ROS2 话题、深度编码、内参和 TF frame。

目的：把「猜话题名、猜坐标系名、猜深度单位」这三件最容易卡住联调的事，
在一次运行里自动问清楚，并直接给出应该用哪条 launch 命令。

用法（云电脑上，先 source /opt/ros/humble/setup.bash 并启动 Isaac Sim）：
    python3 ros2_ws/tools/perception_probe.py
    python3 ros2_ws/tools/perception_probe.py --timeout 15

不需要 colcon build，是独立脚本，只依赖 rclpy。
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_msgs.msg import TFMessage


DTYPES = {
    "32fc1": (np.float32, 1),
    "16uc1": (np.uint16, 1),
    "8uc1": (np.uint8, 1),
    "bgr8": (np.uint8, 3),
    "rgb8": (np.uint8, 3),
    "mono8": (np.uint8, 1),
}


def decode_2d(msg: Image) -> np.ndarray | None:
    """按 encoding 把 Image 解成二维数组（第一通道），不做色彩转换。"""
    key = msg.encoding.lower()
    if key not in DTYPES:
        return None
    dtype, channels = DTYPES[key]
    dtype = np.dtype(dtype).newbyteorder(">" if getattr(msg, "is_bigendian", False) else "<")
    itemsize = dtype.itemsize
    if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * channels * itemsize:
        return None
    if msg.step % itemsize:
        return None
    cols = msg.step // itemsize
    try:
        arr = np.frombuffer(msg.data, dtype=dtype)
    except ValueError:
        return None
    if arr.size != cols * msg.height:
        return None
    arr = arr.reshape(msg.height, cols)
    if channels > 1:
        arr = arr[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    else:
        arr = arr[:, : msg.width]
    return arr


def short_topic(name: str) -> str:
    return name.split("/")[-1]


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("atongmu_perception_probe")
        # 传感器多为 BEST_EFFORT，用 best_effort 订阅才不会「连上但收不到」
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.qos = qos
        self.images: dict[str, dict] = {}
        self.infos: dict[str, CameraInfo] = {}
        self.scans: dict[str, LaserScan] = {}
        self.tf_frames: dict[str, set] = {}
        self.subs = []

    # ------------------------------------------------------------------ #
    def discover(self) -> dict[str, list[str]]:
        """按消息类型归类当前存在的传感器话题。"""
        groups: dict[str, list[str]] = {"image": [], "camera_info": [], "scan": [], "odom": [], "other": []}
        for name, types in self.get_topic_names_and_types():
            for t in types:
                if t.endswith("/Image"):
                    groups["image"].append(name)
                elif t.endswith("/CameraInfo"):
                    groups["camera_info"].append(name)
                elif t.endswith("/LaserScan"):
                    groups["scan"].append(name)
                elif any(t.endswith("/" + x) for x in ("Odometry", "PoseStamped", "TwistStamped")):
                    groups["odom"].append(name)
                elif t.endswith("/TFMessage"):
                    groups["other"].append(name)
        for k in groups:
            groups[k] = sorted(set(groups[k]))
        return groups

    def subscribe_images(self, topics: list[str]) -> None:
        for t in topics:
            self.subs.append(
                self.create_subscription(Image, t, lambda m, topic=t: self._on_image(topic, m), self.qos)
            )

    def subscribe_infos(self, topics: list[str]) -> None:
        for t in topics:
            self.subs.append(
                self.create_subscription(CameraInfo, t, lambda m, topic=t: self.infos.__setitem__(topic, m), self.qos)
            )

    def subscribe_scans(self, topics: list[str]) -> None:
        for t in topics:
            self.subs.append(
                self.create_subscription(LaserScan, t, lambda m, topic=t: self.scans.__setitem__(topic, m), self.qos)
            )

    def subscribe_tf(self) -> None:
        for t in ("/tf", "/tf_static"):
            self.subs.append(
                self.create_subscription(TFMessage, t, lambda m: self._on_tf(m), 50)
            )

    # ------------------------------------------------------------------ #
    def _on_image(self, topic: str, msg: Image) -> None:
        if topic in self.images:
            return  # 每路只要一帧
        entry = {
            "encoding": msg.encoding,
            "width": msg.width,
            "height": msg.height,
            "frame_id": msg.header.frame_id,
            "stats": None,
            "is_depth": "depth" in topic.lower() or msg.encoding in ("32FC1", "16UC1"),
        }
        arr = decode_2d(msg)
        if arr is not None:
            flat = arr.reshape(-1).astype(np.float64)
            finite = flat[np.isfinite(flat)]
            if finite.size and entry["is_depth"]:
                nz = finite[finite > 0]
                if nz.size > 100:
                    entry["stats"] = {
                        "min": float(np.percentile(nz, 1)),
                        "median": float(np.median(nz)),
                        "max": float(np.percentile(nz, 99)),
                        "zero_ratio": round(float((flat <= 0).mean()), 3),
                        "nonfinite_ratio": round(float((~np.isfinite(flat)).mean()), 3),
                    }
        self.images[topic] = entry

    def _on_tf(self, msg: TFMessage) -> None:
        for tr in msg.transforms:
            self.tf_frames.setdefault(tr.child_frame_id, set()).add(tr.header.frame_id)

    # ------------------------------------------------------------------ #
    def best_pick(self, topics: list[str]) -> str | None:
        """优先挑名字里带 camera / rgb / color 的，其次挑最短的。"""
        if not topics:
            return None
        for token in ("camera", "rgb", "color", "image_raw", "depth"):
            hits = [t for t in topics if token in t.lower()]
            if hits:
                return sorted(hits, key=len)[0]
        return sorted(topics, key=len)[0]


def infer_depth_units(entry: dict) -> tuple[str, str]:
    """由编码和数值量级推断深度单位，返回 (单位, 推理依据)。"""
    stats = entry.get("stats")
    enc = entry["encoding"]
    if stats is None:
        return "unknown", "取不到有效深度值"
    med = stats["median"]
    if enc == "16UC1":
        return "mm", "16UC1 通常为毫米"
    if enc == "32FC1":
        if med > 50:
            return "mm", "32FC1 但中位数 %.1f 远大于房间尺度，疑似毫米" % med
        return "m", "32FC1 且中位数 %.3f 符合米制" % med
    return "unknown", "编码 %s 需要人工确认" % enc


def main() -> int:
    ap = argparse.ArgumentParser(description="Isaac Sim 感知话题探针")
    ap.add_argument("--timeout", type=float, default=10.0, help="等待数据的最长秒数")
    args = ap.parse_args()

    rclpy.init()
    node = Probe()
    try:
        print("=" * 66)
        print("Step 1  发现话题")
        print("=" * 66)
        groups = node.discover()
        for k in ("image", "camera_info", "scan", "odom"):
            label = {"image": "图像", "camera_info": "内参", "scan": "激光", "odom": "里程计"}[k]
            if groups[k]:
                for t in groups[k]:
                    print("  [%s] %s" % (label, t))
            else:
                print("  [%s] （无）" % label)

        if not groups["image"]:
            print("\n没有发现任何图像话题。先确认 Isaac Sim 的 ROS2 Bridge 已启动，且 Action Graph 里发布了相机。")
            print("参考：ROS2 Bridge 扩展 → OmniGraph → ROS2 Publish Image / Camera Info。")
            return 1

        color_topic = next((t for t in groups["image"] if "depth" not in t.lower()), groups["image"][0])
        depth_topic = next((t for t in groups["image"] if "depth" in t.lower()), None)

        to_sub = [color_topic] + ([depth_topic] if depth_topic else [])
        node.subscribe_images(to_sub)
        info_topic = groups["camera_info"][0] if groups["camera_info"] else None
        if info_topic:
            node.subscribe_infos([info_topic])
        scan_topic = groups["scan"][0] if groups["scan"] else None
        if scan_topic:
            node.subscribe_scans([scan_topic])
        node.subscribe_tf()

        print("\n" + "=" * 66)
        print("Step 2  等数据（最多 %.0f 秒）" % args.timeout)
        print("=" * 66)
        import time

        t0 = time.time()
        while time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            got_img = color_topic in node.images and (depth_topic is None or depth_topic in node.images)
            got_other = (info_topic is None or info_topic in node.infos) and (scan_topic is None or scan_topic in node.scans)
            if got_img and got_other and node.tf_frames:
                break

        print("\n" + "=" * 66)
        print("Step 3  报告")
        print("=" * 66)

        cam_frame = None
        for topic, e in node.images.items():
            kind = "深度" if e["is_depth"] else "彩色"
            print("\n[%s图] %s" % (kind, topic))
            print("  encoding = %s,  尺寸 = %dx%d,  frame_id = %s" % (e["encoding"], e["width"], e["height"], e["frame_id"] or "(空)"))
            if e["is_depth"]:
                unit, why = infer_depth_units(e)
                print("  深度单位推断: %s   （依据: %s）" % (unit, why))
                if e["stats"]:
                    s = e["stats"]
                    print("  有效深度 min/median/max = %.3f / %.3f / %.3f" % (s["min"], s["median"], s["max"]))
                    print("  零值占比 %.1f%%,  非有限值占比 %.1f%%" % (s["zero_ratio"] * 100, s["nonfinite_ratio"] * 100))
            else:
                cam_frame = e["frame_id"]

        if info_topic:
            info = node.infos.get(info_topic)
            if info is None:
                print("\n[内参] %s：没收到消息（该话题可能没发布）" % info_topic)
            else:
                k = info.k
                print("\n[内参] %s" % info_topic)
                print("  fx=%.3f  fy=%.3f  cx=%.3f  cy=%.3f  %dx%d" % (k[0], k[4], k[2], k[5], info.width, info.height))

        if scan_topic:
            scan = node.scans.get(scan_topic)
            if scan is None:
                print("\n[激光] %s：没收到消息" % scan_topic)
            else:
                r = np.asarray(scan.ranges, dtype=np.float64)
                good = r[np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)]
                print("\n[激光] %s" % scan_topic)
                print("  点数 %d,  angle_min=%.3f rad (%.1f°),  angle_max=%.3f rad (%.1f°),  角分辨率 %.4f rad" % (
                    len(r), scan.angle_min, math.degrees(scan.angle_min),
                    scan.angle_max, math.degrees(scan.angle_max), scan.angle_increment))
                print("  range_min=%.2f  range_max=%.2f  有效点 %d" % (scan.range_min, scan.range_max, good.size))
                if good.size:
                    print("  距离 1%%/50%%/99%% = %.2f / %.2f / %.2f m" % (
                        np.percentile(good, 1), np.percentile(good, 50), np.percentile(good, 99)))

        print("\n[TF] 已收到的坐标系（child <- parent）")
        if not node.tf_frames:
            print("  没收到 /tf 或 /tf_static。检查 Isaac Sim 是否发布了 TF（ROS2 Publish Transform Tree）。")
        else:
            for child in sorted(node.tf_frames):
                print("  %-32s <- %s" % (child, ", ".join(sorted(node.tf_frames[child]))))

        optical = [c for c in node.tf_frames if "optical" in c.lower()]
        if optical:
            print("\n  光学系候选: %s  ← camera_frame 应该填这个" % ", ".join(sorted(optical)))
        elif cam_frame:
            print("\n  ⚠ 没找到名字带 optical 的坐标系，但图像 frame_id = %s。" % cam_frame)
            print("    ROS 图像默认编码是「x 右 y 下 z 前」的光学系；如果这个 frame 实际是「x 前 z 上」，")
            print("    定位结果会整体错位。需要在 Isaac Sim 里把相机的 TF 发布成 optical frame，")
            print("    或改 perception_node.py 的 camera_frame 参数并在代码里补一次坐标轴旋转。")

        # ---------------- 结论 ----------------
        print("\n" + "=" * 66)
        print("Step 4  建议的 launch 命令")
        print("=" * 66)
        depth_flag = "true" if depth_topic else "false"
        print("ros2 launch atongmu_perception_ros perception.launch.py \\")
        print("    image_topic:=%s \\" % color_topic)
        if depth_topic:
            print("    depth_topic:=%s \\" % depth_topic)
        if info_topic:
            print("    camera_info_topic:=%s \\" % info_topic)
        print("    use_depth:=%s \\" % depth_flag)
        if scan_topic:
            print("    scan_topic:=%s \\" % scan_topic)
        if optical:
            print("    camera_frame:=%s \\" % sorted(optical)[0])
        print("    use_tf_extrinsics:=true")

        print("\n接下来人工确认两件事：")
        print("  1) base_frame 的真实名字（上面 TF 列表里找带 base 的，如 base_link / chassis / jetbot_base）")
        print("  2) laser_frame 的真实名字（找带 lidar / laser / scan 的）")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
