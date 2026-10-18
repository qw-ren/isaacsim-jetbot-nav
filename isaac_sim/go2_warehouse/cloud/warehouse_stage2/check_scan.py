#!/usr/bin/env python3
"""Check live scan against independently transformed, stamped source points.

No velocity publishers are created. Raw snapshots are retained for inspection.
"""
import argparse
import json
import math
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage


def stamp(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=15)
    args = parser.parse_args()
    out = Path(args.output)
    # Refuse to overwrite previous evidence.
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node("go2_scan_checker")
    counts = defaultdict(int)
    buffers = {name: {} for name in ("cloud", "scan", "odom")}
    latest_stamps = {}
    backwards = defaultdict(int)
    tf_edge_counts = defaultdict(int)
    extrinsic = None
    paired = []
    positions = []
    scan_stamps = []
    raw_saved = False

    def tf_cb(message):
        nonlocal extrinsic
        for t in message.transforms:
            edge = t.header.frame_id + " -> " + t.child_frame_id
            tf_edge_counts[edge] += 1
            if t.header.frame_id == "unitree_go2/base_link" and t.child_frame_id == "unitree_go2/lidar_frame":
                p, q = t.transform.translation, t.transform.rotation
                extrinsic = (np.array([p.x, p.y, p.z]), Rotation.from_quat([q.x, q.y, q.z, q.w]))

    def compare(s):
        nonlocal raw_saved
        if extrinsic is None or not all(s in b for b in buffers.values()):
            return
        cloud, scan, odom = (buffers[key].pop(s) for key in ("cloud", "scan", "odom"))
        xyz = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        rb = Rotation.from_quat([q.x, q.y, q.z, q.w])
        offset, rl = extrinsic
        base = np.array([p.x, p.y, p.z])
        world = rb.apply(rl.apply(xyz) + offset) + base
        lidar_origin = rb.apply(offset) + base
        full = (rb * rl).as_matrix()
        yaw = math.atan2(full[1, 0], full[0, 0])
        origin = np.array([lidar_origin[0], lidar_origin[1], 0.0])
        projected = Rotation.from_euler("z", -yaw).apply(world - origin)
        radii = np.hypot(projected[:, 0], projected[:, 1])
        angles = np.arctan2(projected[:, 1], projected[:, 0])
        keep = ((world[:, 2] >= 0.10) & (world[:, 2] <= 1.20) &
                (radii >= scan.range_min) & (radii <= scan.range_max) &
                (angles >= scan.angle_min) & (angles < scan.angle_max))
        bins = np.floor((angles[keep] - scan.angle_min) / scan.angle_increment).astype(int)
        in_bounds = (bins >= 0) & (bins < len(scan.ranges))
        expected = np.full(len(scan.ranges), np.inf)
        np.minimum.at(expected, bins[in_bounds], radii[keep][in_bounds])
        actual = np.asarray(scan.ranges, dtype=float)
        finite = np.isfinite(actual)
        common = finite & np.isfinite(expected)
        errors = np.abs(actual[common] - expected[common])
        matched = common & (np.abs(np.where(common, actual, 0) - np.where(common, expected, 0)) < 0.03)
        ground = np.abs(world[:, 2]) < 0.05
        paired.append({
            "stamp_ns": s, "cloud_frame": cloud.header.frame_id, "scan_frame": scan.header.frame_id,
            "rays": len(actual), "finite_rays": int(finite.sum()),
            "finite_mask_agreement": float(np.mean(finite == np.isfinite(expected))),
            "matched_return_fraction": float(matched.sum() / max(1, finite.sum())),
            "range_error_median_m": float(np.median(errors)) if len(errors) else None,
            "range_error_max_m": float(errors.max()) if len(errors) else None,
            "range_error_p99_m": float(np.quantile(errors, .99)) if len(errors) else None,
            "nan_ranges": int(np.isnan(actual).sum()),
            "finite_ranges_within_limits": bool(np.all((actual[finite] >= scan.range_min) &
                                                        (actual[finite] <= scan.range_max))),
            "ground_points": int(ground.sum()), "ground_points_in_height_band": int((ground & keep).sum()),
            "kept_obstacle_points": int(keep.sum()),
            "low_obstacle_points_0p10_to_0p50m": int((keep & (world[:, 2] <= .5)).sum()),
            "scan_time": float(scan.scan_time), "time_increment": float(scan.time_increment),
        })
        if not raw_saved:
            np.savez_compressed(out / "projection_snapshot.npz", xyz_lidar=xyz, xyz_world=world,
                                xyz_scan=projected, keep=keep, expected=expected, actual=actual,
                                origin_world=origin, yaw=np.array(yaw), stamp_ns=np.array(s),
                                angle_min=np.array(scan.angle_min), angle_increment=np.array(scan.angle_increment))
            raw_saved = True

    def callback(key, message):
        counts[key] += 1
        s = stamp(message)
        if key in latest_stamps and s < latest_stamps[key]:
            backwards[key] += 1
        latest_stamps[key] = s
        if key == "odom":
            p = message.pose.pose.position
            positions.append([p.x, p.y, p.z])
        if key == "scan":
            scan_stamps.append(s)
        buffers[key][s] = message
        if len(buffers[key]) > 40:
            buffers[key].pop(next(iter(buffers[key])))
        compare(s)

    static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(TFMessage, "/tf_static", tf_cb, static_qos),
        node.create_subscription(TFMessage, "/tf", tf_cb, 100),
    ]
    for key, kind, topic in (("cloud", PointCloud2, "/unitree_go2/lidar/point_cloud"),
                              ("scan", LaserScan, "/scan"), ("odom", Odometry, "/unitree_go2/odom")):
        subscriptions.append(node.create_subscription(kind, topic, lambda m, k=key: callback(k, m),
                                                       qos_profile_sensor_data))
    started = time.monotonic()
    while time.monotonic() - started < args.seconds:
        rclpy.spin_once(node, timeout_sec=.1)
    graph = {topic: {"publishers": [p.node_name for p in node.get_publishers_info_by_topic(topic)],
                     "subscribers": [p.node_name for p in node.get_subscriptions_info_by_topic(topic)]}
             for topic in ("/scan", "/tf", "/unitree_go2/odom", "/cmd_vel", "/unitree_go2/cmd_vel")}
    displacement = float(np.linalg.norm(np.asarray(positions[-1])[:2] - np.asarray(positions[0])[:2])) if positions else None
    max_excursion = float(np.max(np.linalg.norm(np.asarray(positions)[:, :2] - np.asarray(positions[0])[:2], axis=1))) if positions else None
    intervals = np.diff(scan_stamps) * 1e-9
    checks = {
        "at_least_30_stamped_pairs": len(paired) >= 30,
        "one_scan_publisher": graph["/scan"]["publishers"] == ["go2_cloud_to_scan"],
        "no_external_velocity_publisher": not graph["/cmd_vel"]["publishers"] and not graph["/unitree_go2/cmd_vel"]["publishers"],
        "robot_stationary_within_2cm": max_excursion is not None and max_excursion < .02,
        "timestamps_monotonic": not backwards,
        "only_expected_dynamic_tf_publishers": sorted(graph["/tf"]["publishers"]) == ["go2_scan_ground_frame", "robot_data_manager"],
        "virtual_scan_tf_present": "odom -> unitree_go2/scan_ground" in tf_edge_counts,
        "correct_scan_frame_and_360_rays": bool(paired) and all(p["scan_frame"] == "unitree_go2/scan_ground" and p["rays"] == 360 for p in paired),
        "nonempty_valid_ranges": bool(paired) and all(p["finite_rays"] > 20 and p["nan_ranges"] == 0 and p["finite_ranges_within_limits"] for p in paired),
        "independent_projection_matches": bool(paired) and all(p["matched_return_fraction"] >= .98 and p["finite_mask_agreement"] >= .98 for p in paired),
        "ground_excluded": bool(paired) and all(p["ground_points"] > 1000 and p["ground_points_in_height_band"] == 0 for p in paired),
        "low_obstacle_points_retained": bool(paired) and all(p["low_obstacle_points_0p10_to_0p50m"] > 100 for p in paired),
    }
    report = {"date": "2026-10-14", "mode": "stationary_no_velocity_publisher_created", "counts": dict(counts),
              "pairs": len(paired), "wall_seconds": time.monotonic() - started,
              "displacement_m": displacement, "max_excursion_m": max_excursion,
              "median_scan_stamp_interval_s": float(np.median(intervals)) if len(intervals) else None,
              "timestamp_backwards": dict(backwards), "graph": graph,
              "tf_edge_message_counts": dict(tf_edge_counts),
              "projection_samples": paired, "checks": checks, "passed": all(checks.values()),
              "limitations": ["Stationary sample only; no motion de-skew verification", "Flat z=0 warehouse floor only",
                              "Height band 0.10-1.20m omits obstacles below 10cm or above 1.20m", "No SLAM or Nav2 goal test",
                              "Migrated rclpy omits publisher metadata in subscription callbacks; TF graph and exclusive source-frame ownership inspected separately"]}
    (out / "scan_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: report[key] for key in ("counts", "pairs", "max_excursion_m", "checks", "passed")}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
