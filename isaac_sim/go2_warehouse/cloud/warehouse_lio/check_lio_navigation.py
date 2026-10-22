"""Actual navigation using estimated odom, with no truth subscription/control."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'warehouse_stage2'))
from check_localized_navigation import LocalizedProbe


class LioNavigationProbe(LocalizedProbe):
    def __init__(self):
        super().__init__()
        self.edges = set()
        self.tf_audit_sub = self.create_subscription(TFMessage, '/tf', self.tf_audit, 100)

    def tf_audit(self, message):
        self.edges.update((t.header.frame_id, t.child_frame_id) for t in message.transforms)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument('--goal', nargs=2, type=float)
    target.add_argument('--route', type=Path)
    parser.add_argument('--adapter-output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {str(p):hashlib.sha256(p.read_bytes()).hexdigest()
              for p in [args.map, args.map.parent/'map.pgm']}
    rclpy.init(args=[])
    p = LioNavigationProbe()
    goals = [args.goal] if args.goal is not None else [v['xy'] for v in json.loads(args.route.read_text())['goals']]
    result = dict(mode='isolated_LIO_odom_AMCL_NavigateToPose', goals=goals,legs=[],
                  truth_used_for_control=False, map=str(args.map))
    handle = None
    start = float('inf')
    end = None
    try:
        deadline = time.monotonic()+150
        while time.monotonic() < deadline and not (p.map_rows and p.rows and p.states and time.monotonic()-p.scan_wall < .4):
            p.sample()
        if not (p.map_rows and p.rows and p.states):
            raise RuntimeError('Missing localized LIO navigation inputs')
        p.arm(False)
        life = p.create_client(GetState, '/bt_navigator/get_state')
        deadline = time.monotonic()+40
        active = False
        while time.monotonic() < deadline:
            if life.wait_for_service(timeout_sec=1):
                try:
                    active = p.wait(life.call_async(GetState.Request()), 3).current_state.id == 3
                except RuntimeError:
                    pass
            if active:
                break
            p.sample()
        if not active or not p.nav.wait_for_server(timeout_sec=8):
            raise RuntimeError('Navigator is not ready')
        result['start_pose'] = p.map_pose()
        start = time.monotonic()
        for index,xy in enumerate(goals):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = p.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = xy
            goal.pose.pose.orientation.w = 1.
            leg = dict(index=index, goal=xy, start_pose=p.map_pose())
            result['legs'].append(leg)
            leg_start = time.monotonic()
            handle = p.wait(p.nav.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Navigation goal rejected')
            future = handle.get_result_async()
            deadline = time.monotonic()+(600 if args.route else 240)
            armed = False
            while not future.done():
                p.sample()
                if time.monotonic() > deadline:
                    raise RuntimeError('Navigation wall timeout')
                if (args.adapter_output/'FAULT.json').exists():
                    raise RuntimeError('LIO odometry adapter fault')
                if p.rows[-1]['tilt'] > .5 or not .2 <= p.rows[-1]['z'] <= .85:
                    raise RuntimeError('Posture limit')
                if not armed and p.commands and p.commands[-1]['wall'] > leg_start:
                    p.arm(True)
                    armed, armed_wall = True, time.monotonic()
                if armed and p.states and p.states[-1]['wall'] > armed_wall+.2 and not p.states[-1]['armed']:
                    raise RuntimeError('Guard latched: '+p.states[-1]['reason'])
            leg['status'] = future.result().status
            leg['arrival_pose'] = p.map_pose()
            p.arm(False)
            print(json.dumps(dict(completed_leg=leg)),flush=True)
            (args.output/'PROGRESS.json').write_text(json.dumps(result,indent=2)+'\n')
            if leg['status'] != 4:
                raise RuntimeError('Navigation action did not succeed on leg '+str(index))
            warm_until = time.monotonic()+2
            while time.monotonic() < warm_until:
                p.sample()
        end = time.monotonic()
        result['action_status'] = 4
        result['arrival_pose'] = p.map_pose()
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if handle is not None:
            try:
                p.wait(handle.cancel_goal_async(), 3)
            except Exception:
                pass
        try:
            p.arm(False)
        except Exception as exc:
            result['disarm_error'] = str(exc)
        until = time.monotonic()+4
        while time.monotonic() < until:
            p.sample()
        tf_sources = {x.node_name for x in p.get_publishers_info_by_topic('/tf')}
        edges = [dict(parent=a, child=b) for a,b in sorted(p.edges)]
        odom_sources = [x.node_name for x in p.get_publishers_info_by_topic('/unitree_go2/odom')]
        base_edges = [x for x in edges if x['child'] == 'unitree_go2/base_link']
        moving = [x for x in p.commands if x['wall'] >= start and x['vx'] > 1e-3]
        rows = [x for x in p.rows if start <= x['wall'] <= (end or time.monotonic())]
        result['interfaces'] = dict(odom_sources=odom_sources, dynamic_edges=edges, tf_sources=sorted(tf_sources),
            tf_audit_basis='node graph plus frozen Bridge isolation; cloud rclpy has no per-message MessageInfo callback')
        result['checks'] = dict(
            no_error='error' not in result, action_succeeded=result.get('action_status') == 4,
            exclusively_estimated_odom=odom_sources == ['go2_lio_nav_odometry'],
            exclusively_estimated_base_tf=bool(base_edges) and all(x['parent'] == 'odom' for x in base_edges) and 'go2_lio_nav_odometry' in tf_sources and tf_sources <= {'go2_lio_nav_odometry','amcl','go2_scan_ground_frame'},
            lio_adapter_healthy=len(p.rows)>=10 and odom_sources==['go2_lio_nav_odometry'] and not (args.adapter_output/'FAULT.json').exists(),
            commands_bounded=bool(moving) and all(0. <= x['vx'] <= .80001 and abs(x['wz']) <= .60001 for x in moving),
            guard_not_latched=not any(x['wall'] >= start and x['reason'] not in ['ENABLED', 'DISARMED'] for x in p.states),
            disarmed_zero=bool(p.states) and not p.states[-1]['armed'] and max(abs(x) for x in p.states[-1]['output']) < 1e-6,
            saved_map_unchanged=all(hashlib.sha256(Path(name).read_bytes()).hexdigest() == h for name,h in hashes.items()))
        result['information'] = dict(estimated_vx_mean_mps=float(np.mean([x['vx'] for x in rows])) if rows else None,
                                     odom_max_wall_interval_s=float(np.diff([x['wall'] for x in p.rows]).max()) if len(p.rows)>1 else None,
                                     amcl_pose_updates=len(p.amcl_poses), map_source='USD collision-derived auxiliary CAD prior; not SLAM',
                                     precision_docking_test=False)
        result['passed'] = all(result['checks'].values())
        (args.output/'telemetry.json').write_text(json.dumps(dict(odom=p.rows,map_poses=p.map_rows,amcl_poses=p.amcl_poses,commands=p.commands,runtime=p.runtime,guard=p.states))+'\n')
        (args.output/'navigation_check.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2), flush=True)
        p.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
