#!/usr/bin/env python3
"""Capture one algorithm camera frame paired with actual detection messages."""
import argparse
import json
from pathlib import Path
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node("capture_planar_preview")
    images, detections = {}, {}
    bridge = CvBridge()
    done = False

    def try_save():
        nonlocal done
        if done:
            return
        pairs = sorted(images.keys() & detections.keys())
        if not pairs:
            return
        stamp = pairs[-1]
        frame = bridge.imgmsg_to_cv2(images[stamp], desired_encoding="bgr8").copy()
        packet = detections[stamp]
        for det in packet["detections"]:
            x1, y1, x2, y2 = [int(v) for v in det["bbox"]]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 255), 1)
            label = "%s %.2f" % (det["class_name"], det["confidence"])
            cv2.putText(frame, label, (x1, max(12, y1 - 3)), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 220, 255), 1)
        if not cv2.imwrite(str(args.out / "detections.png"), frame):
            raise RuntimeError("failed to save image")
        (args.out / "detections.json").write_text(json.dumps(packet, ensure_ascii=False, indent=2), encoding="utf-8")
        done = True

    def on_image(msg):
        key = round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 6)
        images[key] = msg
        while len(images) > 12:
            del images[min(images)]
        try_save()

    def on_detections(msg):
        data = json.loads(msg.data)
        detections[round(data["stamp"], 6)] = data
        while len(detections) > 12:
            del detections[min(detections)]
        try_save()

    image_sub = node.create_subscription(Image, "/jetbot/image_raw", on_image, qos_profile_sensor_data)
    detection_sub = node.create_subscription(String, "/perception/detections", on_detections, 10)
    deadline = time.monotonic() + 30
    try:
        while not done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not done:
        raise SystemExit("No matched camera/detection messages within 30s")
    print(args.out / "detections.png")


if __name__ == "__main__":
    main()
