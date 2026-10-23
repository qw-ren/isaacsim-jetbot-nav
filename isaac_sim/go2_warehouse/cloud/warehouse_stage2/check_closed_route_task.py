"""Stationary enclosure: full NavigateToPose aborts, reopening restores planning.

The existing safety bridge is never armed. No velocity or costmap-clear request
is published. Raw observations and native action results precede metrics.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose

from check_branch_navigation import LocalizedProbe


def digest_map(message):
    info = message.info
    payload = {"frame": message.header.frame_id, "width": info.width,
               "height": info.height, "resolution": info.resolution,
               "origin": [info.origin.position.x, info.origin.position.y,
                          info.origin.position.z, info.origin.orientation.x,
                          info.origin.orientation.y, info.origin.orientation.z,
                          info.origin.orientation.w], "data": list(message.data)}
    return hashlib.sha256(json.dumps(payload, separators=(",", ":")).encode()).hexdigest()


class ClosedProbe(LocalizedProbe):
    def __init__(self):
        super().__init__()
        self.latest_scan = None
        self.map_message = None
        self.map_digests = []
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.receive_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def receive_map(self, message):
        self.map_message = message
        self.map_digests.append({"wall": time.monotonic(), "digest": digest_map(message)})

    def scan(self, message):
        super().scan(message)
        self.latest_scan = message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--map-image", type=Path)
    parser.add_argument("--bt-xml", type=Path,
                        default=Path(__file__).with_name("navigate_to_pose.xml"))
    parser.add_argument("--action-timeout", type=float, default=120.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    # Pin the actual goal's tree to an existing full navigation tree without
    # recovery operations that could clear obstacle observations automatically.
    tree = ET.parse(args.bt_xml)
    tags = [element.tag for element in tree.iter()]
    if "ComputePathToPose" not in tags or "FollowPath" not in tags:
        raise SystemExit("Expected a full ComputePathToPose + FollowPath navigation tree")
    if any("Clear" in tag or "Recovery" in tag for tag in tags):
        raise SystemExit("This check requires the existing tree without clearing/recovery nodes")
    image = args.map_image or args.map.parent / "map.pgm"
    files = [args.map, image]
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
    rclpy.init()
    probe = ClosedProbe()
    planner = ActionClient(probe, ComputePathToPose, "/compute_path_to_pose")
    result = {"mode": "stationary_start_enclosure_full_navigation_abort_then_plan_recovery",
              "scope": "four walls around start; not remote double-passage closure or moving blockage",
              "guard_arm_true_requested": False, "velocity_publisher_created": False,
              "costmap_clear_requested": False, "map_file_hashes_before": hashes,
              "behavior_tree": str(args.bt_xml.resolve()),
              "behavior_tree_sha256": hashlib.sha256(args.bt_xml.read_bytes()).hexdigest()}
    navigation_handle = None
    map_before = None
    observation_start = None

    def write(name, value):
        (args.output / name).write_text(json.dumps(value, indent=2) + "\n")

    def wait(predicate, timeout=90., label="observation"):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            probe.sample()
        if not predicate():
            raise TimeoutError(label + " timed out")

    def future_result(future, timeout=20., label="action request"):
        wait(future.done, timeout, label)
        return future.result()

    def fixture(enabled, phase):
        command = {"id": args.output.name + "_" + phase,
                   "enclosure_enabled": enabled, "enabled": False}
        request = args.fixture / "obstacle_request.json"
        temporary = request.with_suffix(".tmp")
        temporary.write_text(json.dumps(command))
        temporary.replace(request)
        response = args.fixture / "enclosure_state.json"
        wait(lambda: response.exists() and json.loads(response.read_text()).get("id") == command["id"],
             40., "physical enclosure acknowledgement")
        state = json.loads(response.read_text())
        walls = state.get("enclosure_walls", [])
        if len(walls) != 4 or state.get("enclosure_enabled") != enabled:
            raise RuntimeError("Four-wall enclosure state does not match request")
        if state.get("original_B_barrier_present"):
            raise RuntimeError("Original B barrier must remain absent")
        if any(w["present"] != enabled or w["collision_enabled"] != enabled for w in walls):
            raise RuntimeError("Actual USD enclosure does not match requested state")
        write(phase + "_fixture.json", state)
        return state

    def settle():
        start = probe.rows[-1]["sim"]
        wait(lambda: probe.rows[-1]["sim"] - start >= 4. and time.monotonic() - probe.scan_wall < .5,
             70., "natural scan/costmap update")

    def snapshot(phase):
        grid = probe.costmap
        scan = probe.latest_scan
        if grid is None or scan is None:
            raise RuntimeError("Missing costmap or scan")
        info = grid.info
        np.savez_compressed(args.output / (phase + ".npz"),
                            costmap=np.asarray(grid.data, dtype=np.int8).reshape(info.height, info.width),
                            origin=[info.origin.position.x, info.origin.position.y],
                            resolution=info.resolution,
                            costmap_stamp=grid.header.stamp.sec + grid.header.stamp.nanosec * 1e-9,
                            scan=np.asarray(scan.ranges),
                            scan_angles=[scan.angle_min, scan.angle_increment],
                            map=np.asarray(probe.map_message.data, dtype=np.int8),
                            odom_xy=[probe.rows[-1]["x"], probe.rows[-1]["y"]])

    def healthy():
        if not (probe.rows and probe.states and probe.runtime and probe.map_rows
                and probe.map_message is not None and probe.costmap is not None
                and probe.latest_scan is not None):
            return False
        now = time.monotonic()
        stamp = probe.costmap.header.stamp
        grid_sim = stamp.sec + stamp.nanosec * 1e-9
        return (now - probe.scan_wall < .5 and now - probe.rows[-1]["wall"] < .5
                and now - probe.runtime[-1]["wall"] < .5
                and abs(probe.rows[-1]["sim"] - grid_sim) < 2.5)

    try:
        # A wrapper may have written initial_open before simulation starts so
        # AMCL can first localize against the unchanged prior map.
        wait(healthy, 100., "localized scan/odom/costmap/guard readiness")
        probe.arm(False)
        wait(lambda: bool(probe.states) and not probe.states[-1]["armed"], 10., "DISARMED state")
        lifecycle = probe.create_client(GetState, "/bt_navigator/get_state")
        deadline = time.monotonic() + 40.
        active = False
        while time.monotonic() < deadline and not active:
            if lifecycle.wait_for_service(timeout_sec=1.):
                try:
                    active = future_result(lifecycle.call_async(GetState.Request()), 4., "BT lifecycle").current_state.id == 3
                except TimeoutError:
                    pass
            probe.sample()
        if not active or not probe.nav.wait_for_server(timeout_sec=8.) or not planner.wait_for_server(timeout_sec=8.):
            raise RuntimeError("Full navigation/planning servers not active")
        result["readiness"] = {"bt_navigator_active": active, "sensors_costmap_healthy": healthy(),
                               "guard_disarmed": not probe.states[-1]["armed"]}
        map_before = digest_map(probe.map_message)
        result["map_digest_before"] = map_before
        observation_start = time.monotonic()
        result["observation_start_wall"] = observation_start
        result["start"] = dict(probe.rows[-1])
        transform = probe.buffer.lookup_transform("map", "odom", Time()).transform
        q = transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        target = {"x": transform.translation.x + math.cos(yaw) * 6.5,
                  "y": transform.translation.y + math.sin(yaw) * 6.5, "yaw": yaw,
                  "odom_target": [6.5, 0.]}
        result["target"] = target
        result["closed_fixture"] = fixture(True, "closed")
        settle()
        snapshot("closed_before_navigation")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = probe.get_clock().now().to_msg()
        goal.pose.pose.position.x = target["x"]
        goal.pose.pose.position.y = target["y"]
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        goal.behavior_tree = str(args.bt_xml.resolve())
        navigation_handle = future_result(probe.nav.send_goal_async(goal))
        result["navigate_goal_accepted"] = navigation_handle.accepted
        if not navigation_handle.accepted:
            raise RuntimeError("NavigateToPose rejected rather than executing and aborting")
        native = future_result(navigation_handle.get_result_async(), args.action_timeout, "NavigateToPose finite termination")
        result["navigate_status"] = int(native.status)
        result["navigate_result_repr"] = repr(native.result)
        result["navigation_finished_wall"] = time.monotonic()
        write("native_navigation_result.json", {k: result[k] for k in
              ["navigate_goal_accepted", "navigate_status", "navigate_result_repr", "navigation_finished_wall"]})
        snapshot("closed_after_navigation")
        result["opened_fixture"] = fixture(False, "opened")
        settle()
        snapshot("opened_before_planning")
        request = ComputePathToPose.Goal()
        # Reuse the exact frozen map-frame target, not a newly fitted target.
        request.goal = goal.pose
        request.goal.header.stamp = probe.get_clock().now().to_msg()
        request.planner_id = "GridBased"
        request.use_start = False
        handle = future_result(planner.send_goal_async(request))
        if not handle.accepted:
            raise RuntimeError("Reopened ComputePathToPose rejected")
        response = future_result(handle.get_result_async(), 60., "reopened native plan")
        points = [[pose.pose.position.x, pose.pose.position.y] for pose in response.result.path.poses]
        result["opened_plan_status"] = int(response.status)
        result["opened_plan_points"] = len(points)
        result["opened_plan_result_repr"] = repr(response.result)
        write("native_opened_plan_result.json", {"status": int(response.status), "path": points,
              "target": target, "result_repr": repr(response.result)})
        np.save(args.output / "opened_path.npy", np.asarray(points, dtype=float).reshape(-1, 2))
    except Exception as error:
        result["error"] = str(error)
    finally:
        if navigation_handle is not None:
            try:
                future_result(navigation_handle.cancel_goal_async(), 5., "navigation cleanup")
            except Exception as error:
                result["cancel_cleanup_error"] = str(error)
        try:
            probe.arm(False)
        except Exception as error:
            result["disarm_error"] = str(error)
        try:
            fixture(False, "cleanup_opened")
        except Exception as error:
            result["fixture_cleanup_error"] = str(error)
        # Persist original data before computing any acceptance metrics.
        write("telemetry.json", {"odom": probe.rows, "map_poses": probe.map_rows,
              "amcl_poses": probe.amcl_poses, "commands": probe.commands,
              "runtime": probe.runtime, "guard": probe.states,
              "global_plans": probe.plans, "published_map_digests": probe.map_digests})
        write("action_result_before_metrics.json", result)
        start = observation_start if observation_start is not None else float("inf")
        rows = [row for row in probe.rows if row["wall"] >= start]
        states = [state for state in probe.states if state.get("wall", 0.) >= start]
        commands = [row for row in probe.commands if row["wall"] >= start]
        runtime = [row for row in probe.runtime if row["wall"] >= start]
        xy = np.asarray([[row["x"], row["y"]] for row in rows]).reshape(-1, 2)
        displacement = float(np.linalg.norm(xy - xy[0], axis=1).max()) if len(xy) else None
        result["max_displacement_m"] = displacement
        result["quantitative_policy"] = "No speed, precision or statistical target; <.20m displacement only verifies no traversal"
        result["map_file_hashes_after"] = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
        result["checks"] = {
            "no_error": "error" not in result,
            "full_native_navigation_aborted": result.get("navigate_status") == 6,
            "same_goal_native_plan_recovers": result.get("opened_plan_status") == 4 and result.get("opened_plan_points", 0) >= 2,
            "guard_always_disarmed": bool(states) and all(not state["armed"] and state["reason"] == "DISARMED" for state in states),
            "guard_target_always_zero": bool(states) and all(max(abs(v) for v in state["output"]) < 1e-6 for state in states),
            "runtime_target_always_zero": bool(runtime) and all(abs(row["vx"]) < 1e-6 and abs(row["wz"]) < 1e-6 for row in runtime),
            "no_nonzero_navigation_command": all(abs(row["vx"]) < 1e-6 and abs(row["wz"]) < 1e-6 for row in commands),
            "no_robot_traversal": displacement is not None and displacement < .20,
            "saved_prior_map_unchanged": result["map_file_hashes_after"] == hashes,
            "published_map_unchanged": map_before is not None and bool(probe.map_digests) and all(sample["digest"] == map_before for sample in probe.map_digests if sample["wall"] >= start) and digest_map(probe.map_message) == map_before,
            "map_server_sole_map_source": [info.node_name for info in probe.get_publishers_info_by_topic("/map")] == ["map_server"],
            "cleanup_opened_and_disarmed": "fixture_cleanup_error" not in result and "disarm_error" not in result,
        }
        result["passed"] = all(result["checks"].values())
        write("closed_route_check.json", result)
        summary = {key: result.get(key) for key in ['mode', 'navigate_status', 'opened_plan_status', 'opened_plan_points', 'max_displacement_m', 'checks', 'passed', 'error']}
        print(json.dumps(summary, indent=2), flush=True)
        probe.destroy_node()
        rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
