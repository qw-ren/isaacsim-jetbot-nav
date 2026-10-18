#!/usr/bin/env python3
"""Exercise ComputePathToPose, not NavigateToPose; never publish velocity."""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--map-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes_before = {name: hashlib.sha256((args.map_dir / name).read_bytes()).hexdigest() for name in ("map.pgm", "map.yaml")}
    boxes = json.loads((args.map_dir / "collision_boxes.json").read_text())
    rclpy.init()
    node = rclpy.create_node("go2_plan_checker", parameter_overrides=[Parameter("use_sim_time", value=True)])
    received, positions, clocks = {}, [], []
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)

    def odom_cb(message):
        p = message.pose.pose.position
        positions.append([p.x, p.y])
        received["odom"] = message

    subscriptions = [
        node.create_subscription(OccupancyGrid, "/map", lambda m: received.update(map=m), durable),
        node.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: received.update(costmap=m), durable),
        node.create_subscription(Odometry, "/unitree_go2/odom", odom_cb, qos_profile_sensor_data),
        node.create_subscription(Clock, "/clock", lambda m: clocks.append(m.clock.sec + m.clock.nanosec * 1e-9), qos_profile_sensor_data),
    ]

    def wait_until(predicate, seconds=20):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not predicate():
            raise TimeoutError("Timed out waiting for planning evidence")

    wait_until(lambda: all(k in received for k in ("map", "costmap", "odom")) and len(clocks) > 2)
    states, simulation_time = {}, {}
    for name in ("map_server", "planner_server", "global_costmap/global_costmap"):
        client = node.create_client(GetState, "/" + name + "/get_state")
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError("Missing lifecycle service " + name)
        future = client.call_async(GetState.Request())
        wait_until(future.done)
        state = future.result().current_state
        states[name] = {"id": state.id, "label": state.label}
        node.destroy_client(client)
        client = node.create_client(GetParameters, "/" + name + "/get_parameters")
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError("Missing parameter service " + name)
        request = GetParameters.Request()
        request.names = ["use_sim_time"]
        future = client.call_async(request)
        wait_until(future.done)
        value = future.result().values[0]
        simulation_time[name] = value.type == ParameterType.PARAMETER_BOOL and value.bool_value
        node.destroy_client(client)

    action = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    if not action.wait_for_server(timeout_sec=10):
        raise RuntimeError("ComputePathToPose unavailable")
    results = []
    for label, target, expected_success in (("corridor", [2.0, -1.0], True),
                                             ("occupied_goal", [0.0, 0.3], False),
                                             ("outside_map", [100.0, 100.0], False)):
        requested_at = time.monotonic()
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = node.get_clock().now().to_msg()
        goal.goal.pose.position.x, goal.goal.pose.position.y = target
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = "GridBased"
        goal.use_start = False
        sent = action.send_goal_async(goal)
        wait_until(sent.done)
        handle = sent.result()
        entry = {"case": label, "target_xy": target, "expected_success": expected_success,
                 "goal_accepted": bool(handle.accepted), "use_start": False}
        if not handle.accepted:
            raise RuntimeError("Planning action rejected before execution: " + label)
        future = handle.get_result_async()
        wait_until(future.done)
        result = future.result()
        poses = result.result.path.poses
        xy = np.array([[p.pose.position.x, p.pose.position.y] for p in poses], dtype=float).reshape(-1, 2)
        entry.update(status=result.status, path_frame=result.result.path.header.frame_id,
                     path_points=len(xy), path_xy=xy.tolist(),
                     wall_request_elapsed_s=time.monotonic()-requested_at,
                     planning_seconds=result.result.planning_time.sec + result.result.planning_time.nanosec * 1e-9)
        if len(xy):
            entry["path_length_m"] = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
            entry["goal_error_m"] = float(np.linalg.norm(xy[-1] - target))
            od = received["odom"].pose.pose.position
            entry["start_error_from_current_odom_m"] = float(np.linalg.norm(xy[0] - [od.x, od.y]))
            dense = np.concatenate([a + np.linspace(0, 1, max(2, math.ceil(np.linalg.norm(b-a)/.01)+1))[:, None]*(b-a)
                                    for a, b in zip(xy[:-1], xy[1:])], axis=0) if len(xy) > 1 else xy
            for key in ("map", "costmap"):
                grid = received[key]
                o = grid.info.origin.position
                cells = np.floor((dense - [o.x, o.y]) / grid.info.resolution).astype(int)
                inside = (cells[:, 0] >= 0) & (cells[:, 0] < grid.info.width) & (cells[:, 1] >= 0) & (cells[:, 1] < grid.info.height)
                values = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)[cells[inside, 1], cells[inside, 0]]
                entry[key + "_all_dense_samples_inside"] = bool(inside.all())
                entry[key + "_max_cost"] = int(values.max()) if len(values) else None
                entry[key + "_no_unknown_or_lethal"] = bool(len(values) and np.all((values >= 0) & (values < (99 if key == "costmap" else 100))))
            minimum = float("inf")
            for box in boxes:
                lo, hi = np.array(box["min"][:2]), np.array(box["max"][:2])
                delta = np.maximum(np.maximum(lo - dense, dense - hi), 0)
                minimum = min(minimum, float(np.linalg.norm(delta, axis=1).min()))
            entry["min_center_distance_to_collision_aabbs_m"] = minimum
        results.append(entry)
    wait_until(lambda: len(positions) > 30)
    map_msg, costmap = received["map"], received["costmap"]
    for key in ("map", "costmap"):
        msg = received[key]
        np.save(args.output / (key + ".npy"), np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width))
    graph = {topic: {"publishers": [p.node_name for p in node.get_publishers_info_by_topic(topic)]}
             for topic in ("/map", "/tf", "/cmd_vel", "/unitree_go2/cmd_vel")}
    nodes = node.get_node_names()
    excursion = float(np.linalg.norm(np.asarray(positions) - np.asarray(positions[0]), axis=1).max())
    good = results[0]
    map_array = np.asarray(map_msg.data)
    cost_array = np.asarray(costmap.data)
    # Independent file-to-ROS round trip, including vertical orientation and unknown cells.
    image = (args.map_dir / "map.pgm").read_bytes().split(b"\n", 4)
    raster = np.frombuffer(image[4], dtype=np.uint8).reshape(map_msg.info.height, map_msg.info.width)[::-1]
    expected_map = np.where(raster == 0, 100, np.where(raster == 254, 0, -1))
    checks = {
        "lifecycle_nodes_active": all(v["id"] == 3 for v in states.values()),
        "all_nodes_use_sim_time": all(simulation_time.values()),
        "one_map_publisher": graph["/map"]["publishers"] == ["map_server"],
        "map_costmap_same_grid": map_msg.info.width == costmap.info.width and map_msg.info.height == costmap.info.height and abs(map_msg.info.resolution-costmap.info.resolution) < 1e-8,
        "map_has_occupied_free_unknown": all(np.any(map_array == v) for v in (-1, 0, 100)),
        "pgm_to_ros_grid_exact_match": bool(np.array_equal(map_array.reshape(expected_map.shape), expected_map)),
        "costmap_inflation_present": bool(np.any((cost_array > 0) & (cost_array < 99) & (map_array == 0))),
        "corridor_action_succeeded": good["status"] == GoalStatus.STATUS_SUCCEEDED and good["path_points"] > 2,
        "path_endpoints_correct": good.get("goal_error_m", 100) < .10 and good.get("start_error_from_current_odom_m", 100) < .10 and good["path_frame"] == "map",
        "dense_path_free_in_map_and_costmap": all(good.get(k + "_all_dense_samples_inside", False) and good.get(k + "_no_unknown_or_lethal", False) for k in ("map", "costmap")),
        "path_provisional_footprint_clearance": good.get("min_center_distance_to_collision_aabbs_m", 0) >= .45,
        "occupied_goal_aborted": results[1]["status"] == GoalStatus.STATUS_ABORTED and results[1]["path_points"] == 0,
        "outside_map_aborted": results[2]["status"] == GoalStatus.STATUS_ABORTED and results[2]["path_points"] == 0,
        "robot_stationary_no_velocity_publishers": excursion < .02 and not graph["/cmd_vel"]["publishers"] and not graph["/unitree_go2/cmd_vel"]["publishers"],
        "no_controller_or_navigation_executor": not any(name in nodes for name in ("controller_server", "bt_navigator", "behavior_server", "amcl", "slam_toolbox")),
        "map_files_unchanged": hashes_before == {name: hashlib.sha256((args.map_dir / name).read_bytes()).hexdigest() for name in hashes_before},
        "clock_advances": len(clocks) > 2 and clocks[-1] > clocks[0],
    }
    report = {"date": "2026-10-15", "test": "planning_only_NOT_navigation_or_SLAM", "lifecycle": states,
              "use_sim_time": simulation_time, "graph": graph, "nodes": nodes, "cases": results,
              "max_robot_excursion_m": excursion, "map_file_sha256": hashes_before,
              "grid": {"width": map_msg.info.width, "height": map_msg.info.height, "resolution": map_msg.info.resolution,
                       "origin": [map_msg.info.origin.position.x, map_msg.info.origin.position.y]},
              "checks": checks, "passed": all(checks.values())}
    (args.output / "plan_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: report[key] for key in ("checks", "passed", "max_robot_excursion_m")}, indent=2))
    action.destroy()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
