#!/usr/bin/env python3
"""Read-only ROS2 stream check; records observations without issuing commands."""
import argparse
import json
import math
from pathlib import Path
import time


def validate_message(data, expected_frame):
    if data.get("schema") != "atongmu.planar_observations" or data.get("version") != 1:
        raise ValueError("unexpected schema/version")
    if data.get("source") != "vision" or data.get("frame_id") != expected_frame:
        raise ValueError("unexpected source/frame")
    if not isinstance(data.get("session_id"), str) or not data["session_id"]:
        raise ValueError("missing session_id")
    if type(data.get("sequence")) is not int or data["sequence"] < 0:
        raise ValueError("invalid sequence")
    if type(data.get("stamp")) not in (int, float) or not math.isfinite(data["stamp"]) or data["stamp"] <= 0:
        raise ValueError("invalid simulation stamp")
    if not isinstance(data.get("objects"), list):
        raise ValueError("objects must be a list")
    ids = set()
    for obj in data["objects"]:
        if type(obj.get("track_id")) is not int or obj["track_id"] < 0:
            raise ValueError("invalid track_id")
        expected_id = "vision:%s:%d" % (data["session_id"], obj["track_id"])
        if obj.get("object_id") != expected_id or expected_id in ids:
            raise ValueError("invalid or duplicate object_id")
        ids.add(expected_id)
        xy = obj.get("position_xy")
        if not isinstance(xy, list) or len(xy) != 2 or not all(
                type(v) in (int, float) and math.isfinite(v) for v in xy):
            raise ValueError("position_xy must contain two finite metres")
        confidence = obj.get("confidence")
        if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError("invalid confidence")
        if not isinstance(obj.get("class_name"), str) or not obj["class_name"]:
            raise ValueError("missing class_name")
        if obj.get("localization_method") not in ("depth", "lidar", "lidar+depth", "ground"):
            raise ValueError("invalid localization method")
        reference = "ground_contact_estimate" if obj["localization_method"] == "ground" else "visible_surface"
        if obj.get("position_reference") != reference:
            raise ValueError("invalid position_reference")
        distance = obj.get("range_m")
        if type(distance) not in (int, float) or not math.isfinite(distance) or distance < 0:
            raise ValueError("invalid range_m")
        if any(key in obj for key in ("dimensions", "orientation_xyzw", "position")):
            raise ValueError("unexpected 3D geometry")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/perception/objects_2d")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--seconds", type=float, default=30.)
    ap.add_argument("--max-age", type=float, default=.5)
    ap.add_argument("--out", type=Path, required=True, help="new evidence directory")
    args = ap.parse_args()
    if args.seconds <= 0 or args.max_age <= 0:
        ap.error("seconds/max-age must be positive")
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String

    args.out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node("check_planar_stream", parameter_overrides=[Parameter("use_sim_time", value=True)])
    counts = {"messages": 0, "nonempty": 0, "errors": 0, "waiting_for_clock": 0}
    last = {}
    errors = []
    stream = (args.out / "observations.jsonl").open("w", encoding="utf-8")

    def on_message(msg):
        counts["messages"] += 1
        try:
            data = json.loads(msg.data)
            validate_message(data, args.frame)
            now = node.get_clock().now().nanoseconds * 1e-9
            if now <= 0:
                counts["waiting_for_clock"] += 1
                return  # /clock and data discovery can arrive in either order.
            if not -.1 <= now - data["stamp"] <= args.max_age:
                raise ValueError("observation age %.6fs outside [-0.1, %.3f], clock=%.6f stamp=%.6f"
                                 % (now - data["stamp"], args.max_age, now, data["stamp"]))
            previous = last.get(data["session_id"])
            current = (data["sequence"], data["stamp"])
            if previous is not None and (current[0] <= previous[0] or current[1] <= previous[1]):
                raise ValueError("sequence/stamp did not increase")
            last[data["session_id"]] = current
            counts["nonempty"] += bool(data["objects"])
            stream.write(json.dumps(data, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            counts["errors"] += 1
            if len(errors) < 20:
                errors.append(str(exc))

    sub = node.create_subscription(String, args.topic, on_message, 10)
    deadline = time.monotonic() + args.seconds
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        stream.close()
        node.destroy_node()
        rclpy.shutdown()
    summary = dict(counts, ok=counts["nonempty"] > 0 and counts["errors"] == 0 and not interrupted,
                   interrupted=interrupted, errors_sample=errors, topic=args.topic, frame=args.frame,
                   scope="wire format, freshness and nonempty observations only; not localization accuracy or navigation")
    (args.out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
