"""M3 live physical box -> scan -> global costmap -> Navfn; no velocity publisher."""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    files = [args.map, args.map.parent / 'map.pgm']
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    rclpy.init()
    node = rclpy.create_node('go2_m3_costmap_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    received, positions, scans = {}, [], []
    durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def odom(message):
        p = message.pose.pose.position
        positions.append([p.x, p.y])
    subscriptions = [
        node.create_subscription(OccupancyGrid, '/map', lambda m: received.update(map=m), durable),
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: received.update(costmap=m), durable),
        node.create_subscription(Odometry, '/unitree_go2/odom', odom, qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data),
        node.create_subscription(Clock, '/clock', lambda m: received.update(sim=m.clock.sec + m.clock.nanosec * 1e-9), qos_profile_sensor_data),
    ]
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    planner = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    result = {'mode': 'stationary_physical_obstacle_global_costmap_Navfn', 'velocity_commands_sent': False}
    def wait(predicate, timeout=90.):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        if not predicate():
            raise TimeoutError('M3 evidence timeout')
    def fixture(enabled, phase):
        request = args.fixture / 'obstacle_request.json'
        cmd = {'id': f'{args.output.name}_{phase}', 'enabled': enabled}
        temporary = request.with_suffix('.tmp')
        temporary.write_text(json.dumps(cmd))
        temporary.replace(request)
        response = args.fixture / 'obstacle_state.json'
        wait(lambda: response.exists() and json.loads(response.read_text()).get('id') == cmd['id'])
        return json.loads(response.read_text())
    def settle():
        start = received['sim']; count = len(scans)
        wait(lambda: received['sim'] - start >= 4. and len(scans) - count >= 10, 60.)
    def transform(x, y):
        t = buffer.lookup_transform('map', 'odom', Time()).transform
        q = t.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return [t.translation.x + math.cos(yaw)*x - math.sin(yaw)*y,
                t.translation.y + math.sin(yaw)*x + math.cos(yaw)*y]
    def snapshot(phase):
        grid = received['costmap']; info = grid.info
        values = np.asarray(grid.data).reshape(info.height, info.width)
        yy, xx = np.indices(values.shape)
        xx = info.origin.position.x + (xx+.5)*info.resolution
        yy = info.origin.position.y + (yy+.5)*info.resolution
        center = transform(4., -1.)
        mask = (abs(xx-center[0])<.15) & (abs(yy-center[1])<.15)
        if not mask.any(): raise RuntimeError('Fixture outside costmap')
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'; goal.goal.header.stamp = node.get_clock().now().to_msg()
        target = transform(6., -1.)
        goal.goal.pose.position.x, goal.goal.pose.position.y = target
        goal.goal.pose.orientation.w = 1.; goal.planner_id = 'GridBased'; goal.use_start = False
        future = planner.send_goal_async(goal); wait(future.done)
        handle = future.result()
        if not handle.accepted: raise RuntimeError('Planner rejected request')
        future = handle.get_result_async(); wait(future.done)
        response = future.result()
        points = np.asarray([[p.pose.position.x, p.pose.position.y] for p in response.result.path.poses])
        np.savez_compressed(args.output / f'{phase}.npz', costmap=values, path=points, center=center,
                            resolution=info.resolution, origin=[info.origin.position.x,info.origin.position.y],
                            scan=np.asarray(scans[-1].ranges), scan_angles=[scans[-1].angle_min,scans[-1].angle_increment])
        # Square box is nearly aligned; use circumscribed circle for conservative path clearance.
        clearance = None
        if len(points):
            dense = np.concatenate([np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.01)+1)) for a,b in zip(points[:-1],points[1:])])
            clearance = float(np.min(np.linalg.norm(dense-np.asarray(center),axis=1)) - math.sqrt(2)*.3)
        surface_mask = (abs(xx-center[0])<.4) & (abs(yy-center[1])<.4)
        return {'surface_lethal_cells': int(np.sum(values[surface_mask] == 100)), 'sim': received['sim'], 'status': response.status, 'path_points': len(points),
                'box_center_map': center, 'interior_min_cost': int(values[mask].min()),
                'interior_max_cost': int(values[mask].max()), 'conservative_box_clearance_m': clearance}
    try:
        wait(lambda: all(k in received for k in ['map','costmap','sim']) and positions and len(scans)>10)
        if not planner.wait_for_server(timeout_sec=10): raise RuntimeError('No planner action')
        initial_map = list(received['map'].data)
        fixture(False, 'before'); settle(); result['before'] = snapshot('before')
        result['fixture_enabled'] = fixture(True, 'enabled'); settle(); result['enabled'] = snapshot('enabled')
        fixture(False, 'removed'); settle(); result['removed'] = snapshot('removed')
        motion = float(np.max(np.linalg.norm(np.asarray(positions)-positions[0],axis=1)))
        result['max_displacement_m'] = motion
        before, enabled, removed = (result[k] for k in ['before','enabled','removed'])
        result['checks'] = {
            'baseline_box_region_free': before['interior_max_cost'] == 0,
            'physical_box_marked_lethal': enabled['surface_lethal_cells'] > 0,
            'baseline_plan_succeeds': before['status'] == 4,
            'baseline_path_crosses_new_box': before['conservative_box_clearance_m'] is not None and before['conservative_box_clearance_m'] < 0.,
            'obstacle_plan_succeeds': enabled['status'] == 4,
            'new_path_clears_box': enabled['conservative_box_clearance_m'] is not None and enabled['conservative_box_clearance_m'] >= .45,
            'removed_box_cleared': removed['interior_max_cost'] == 0,
            'removed_plan_succeeds': removed['status'] == 4,
            'saved_map_file_unchanged': all(hashlib.sha256(p.read_bytes()).hexdigest() == hashes[str(p)] for p in files),
            'published_map_unchanged': list(received['map'].data) == initial_map,
            'no_commanded_motion': motion < .2,
        }
    except Exception as error:
        result['error'] = str(error)
    finally:
        try: fixture(False, 'cleanup')
        except Exception as error: result['cleanup_error'] = str(error)
        result['passed'] = not result.get('error') and bool(result.get('checks')) and all(result['checks'].values())
        (args.output / 'costmap_check.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
