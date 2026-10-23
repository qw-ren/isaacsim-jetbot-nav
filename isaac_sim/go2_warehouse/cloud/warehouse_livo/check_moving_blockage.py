"""Native navigation with a sensor-visible physical obstruction inserted in motion.

Only action goals, guard enable and owned fixture requests are published.
No truth subscription, direct velocity, map change or costmap clearing exists.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath, OccupancyGrid

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'warehouse_lio'))
from check_lio_navigation import LioNavigationProbe


class MovingProbe(LioNavigationProbe):
    def __init__(self):
        super().__init__()
        self.plans=[];self.costmap=None
        self.through=ActionClient(self,NavigateThroughPoses,"/navigate_through_poses")
        self.plan_sub=self.create_subscription(NavPath,'/plan',self.plan,10)
        self.cost_sub=self.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:setattr(self,'costmap',m),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))

    def plan(self,m):
        self.plans.append(dict(wall=time.monotonic(),stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                               xy=[[v.pose.position.x,v.pose.position.y] for v in m.poses]))


def request(fixture,identity,enabled):
    p=fixture/'obstacle_request.json';temporary=p.with_suffix('.tmp')
    temporary.write_text(json.dumps(dict(id=identity,enabled=enabled))+'\n');temporary.replace(p)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixture',type=Path,required=True)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--adapter-output',type=Path,required=True)
    parser.add_argument('--prelude',type=Path)
    args=parser.parse_args();args.output.mkdir(exist_ok=False)
    config=json.loads((args.fixture/'dynamic_config.json').read_text())
    hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [args.map,args.map.parent/'map.pgm']}
    goals=[g['xy'] for g in json.loads(args.prelude.read_text())['goals']] if args.prelude else []
    prelude_goals=list(goals)
    if args.prelude:
        prelude_goals += [[-2.,0.],config['goal_xy']]
        goals=[config['goal_xy']]
    else:goals.append(config['goal_xy'])
    result=dict(kind=config['kind'],scope='native continuous navigation; physical insertion after completed prefix while moving',legs=[],
                truth_used_for_control=False,direct_velocity_publisher=False,costmap_clear_requested=False)
    rclpy.init(args=[]);p=MovingProbe();handle=None;injected=False;ack=None
    start=time.monotonic();trigger_wall=None
    try:
        deadline=time.monotonic()+140
        while time.monotonic()<deadline and not (p.map_rows and p.rows and p.states and time.monotonic()-p.scan_wall<.4):p.sample()
        if not (p.map_rows and p.rows and p.states):raise RuntimeError('Missing estimated navigation inputs')
        initial=json.loads((args.fixture/'obstacle_state.json').read_text())
        if initial['enabled']:raise RuntimeError('Obstacle is already present before moving')
        p.arm(False)
        service=p.create_client(GetState,'/bt_navigator/get_state');active=False;deadline=time.monotonic()+40
        while time.monotonic()<deadline:
            if service.wait_for_service(timeout_sec=1) and p.wait(service.call_async(GetState.Request()),3).current_state.id==3:active=True;break
            p.sample()
        if not active or not p.nav.wait_for_server(timeout_sec=8):raise RuntimeError('Native navigator is not active')
        if args.prelude:
            # First creation of the native through-poses BT also creates TF consumers.
            # Prime that tree while the unchanged velocity gate stays disarmed.
            if not p.through.wait_for_server(timeout_sec=8):raise RuntimeError('Native prelude server missing')
            warm=NavigateThroughPoses.Goal()
            for xy in prelude_goals:
                m=PoseStamped();m.header.frame_id='map';m.header.stamp=p.get_clock().now().to_msg()
                m.pose.position.x,m.pose.position.y=xy;m.pose.orientation.w=1.;warm.poses.append(m)
            warm_handle=p.wait(p.through.send_goal_async(warm))
            if not warm_handle.accepted:raise RuntimeError('Disarmed native warmup rejected')
            warm_future=warm_handle.get_result_async();deadline=time.monotonic()+3
            while not warm_future.done() and time.monotonic()<deadline:p.sample()
            if not warm_future.done():
                p.wait(warm_handle.cancel_goal_async(),3);deadline=time.monotonic()+5
                while not warm_future.done() and time.monotonic()<deadline:p.sample()
            if not warm_future.done():raise RuntimeError('Warmup did not end; refusing second active goal')
            result['prelude_warmup']=dict(native_status=warm_future.result().status,guard_disarmed=True,counted_as_navigation_success=False)
            (args.output/'PRELUDE_WARMUP.json').write_text(json.dumps(result['prelude_warmup'],indent=2)+'\n')
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:p.sample()
        for index,xy in enumerate(goals):
            final=index==len(goals)-1
            leg=dict(index=index,goal=xy,final_blockage_leg=final,start_pose=p.map_pose())
            result['legs'].append(leg)
            continuous=bool(args.prelude) and index==0
            def pose(xy):
                m=PoseStamped();m.header.frame_id='map';m.header.stamp=p.get_clock().now().to_msg()
                m.pose.position.x,m.pose.position.y=xy;m.pose.orientation.w=1.;return m
            client=p.through if continuous else p.nav
            if continuous:
                if not client.wait_for_server(timeout_sec=8):raise RuntimeError('Native through-poses server missing')
                goal=NavigateThroughPoses.Goal();goal.poses=[pose(g) for g in prelude_goals]
                leg['waypoints']=prelude_goals;leg['pre_insertion_waypoints']=prelude_goals[:-2];leg['native_action']='NavigateThroughPoses';leg['feedback']=[]
                def feedback(message,target_leg=leg):target_leg['feedback'].append(dict(wall=time.monotonic(),remaining=message.feedback.number_of_poses_remaining))
            else:
                goal=NavigateToPose.Goal();goal.pose=pose(xy);feedback=None;leg['native_action']='NavigateToPose'
            leg_start=time.monotonic();handle=p.wait(client.send_goal_async(goal,feedback_callback=feedback))
            if not handle.accepted:raise RuntimeError('Navigation goal rejected')
            future=handle.get_result_async();deadline=time.monotonic()+(1500 if continuous else 650);armed=False;armed_wall=0.
            travelled=0.;last_xy=None;last_sim=None
            while not future.done():
                p.sample()
                if time.monotonic()>deadline:raise RuntimeError('Native navigation wall timeout')
                if (args.adapter_output/'FAULT.json').exists():raise RuntimeError('Estimated odometry adapter fault')
                if p.rows[-1]['tilt']>.5 or not .2<=p.rows[-1]['z']<=.85:raise RuntimeError('Posture limit')
                if not armed and p.commands and p.commands[-1]['wall']>leg_start and (abs(p.commands[-1]['vx'])>1e-4 or abs(p.commands[-1]['wz'])>1e-4):
                    p.arm(True);armed=True;armed_wall=time.monotonic()
                if armed and p.states and p.states[-1]['wall']>armed_wall+.2 and not p.states[-1]['armed']:
                    reason=p.states[-1]['reason']
                    if not (final and config['kind']=='C' and injected and reason in ['FRONT_SCAN_CLEARANCE','PREDICTED_STATIC_CLEARANCE']):
                        raise RuntimeError('Guard latched: '+reason)
                    result['C_safety_latch']=dict(p.states[-1])
                if p.rows and p.rows[-1]['sim']!=last_sim:
                    current=(p.rows[-1]['x'],p.rows[-1]['y'])
                    if last_xy is not None:travelled+=math.dist(current,last_xy)
                    last_xy=current;last_sim=p.rows[-1]['sim']
                if final and armed and not injected:
                    pose=p.map_pose();origin=leg['start_pose']
                    displacement=math.hypot(pose['x']-origin['x'],pose['y']-origin['y'])
                    recent=[v for v in p.rows[-10:] if v['wall']>armed_wall]
                    moving=bool(recent) and np.mean([v['speed'] for v in recent])>.2
                    route_ready=not continuous or bool(leg['feedback'] and leg['feedback'][-1]['remaining']<=2)
                    if continuous:displacement=travelled
                    if route_ready and displacement>=.25 and moving and abs(pose['y'])<.8 and pose['x']>=1.:
                        if pose['x']>1.8:raise RuntimeError('Missed safe insertion window')
                        trigger_wall=time.monotonic()
                        result['trigger']=dict(wall=trigger_wall,estimated_map_pose=pose,estimated_motion=recent,
                                               displacement_m=displacement,displacement_basis='cumulative estimated odom' if continuous else 'net estimated map',native_remaining=leg['feedback'][-1]['remaining'] if continuous else None,last_native_command=p.commands[-1])
                        request(args.fixture,'insert_during_motion',True);injected=True
                        (args.output/'TRIGGER.json').write_text(json.dumps(result['trigger'],indent=2)+'\n')
                if injected and ack is None:
                    state=json.loads((args.fixture/'obstacle_state.json').read_text())
                    if state['id']=='insert_during_motion' and state['enabled']:
                        ack=state;result['applied_obstacle']=ack
                        (args.output/'APPLIED.json').write_text(json.dumps(ack,indent=2)+'\n')
            leg['status']=future.result().status;leg['arrival_pose']=p.map_pose();p.arm(False)
            (args.output/'PROGRESS.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(dict(completed_leg=leg)),flush=True)
            expected=6 if final and config['kind']=='C' else 4
            if leg['status']!=expected:raise RuntimeError('Unexpected native action result '+str(leg['status']))
            if continuous and (not leg['feedback'] or min(r['remaining'] for r in leg['feedback'])>1):raise RuntimeError('Native prelude ended without passing declared waypoints')
            settle=p.rows[-1]['sim'];deadline=time.monotonic()+20
            while p.rows[-1]['sim']-settle<.5 and time.monotonic()<deadline:p.sample()
        if not injected or ack is None:raise RuntimeError('No acknowledged in-motion insertion')
    except Exception as exc:
        result['error']=str(exc)
    finally:
        if handle is not None:
            try:p.wait(handle.cancel_goal_async(),3)
            except Exception:pass
        try:p.arm(False)
        except Exception as exc:result['disarm_error']=str(exc)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:p.sample()
        outputs=[s for s in p.states if s['wall']>time.monotonic()-2]
        sources=[v.node_name for v in p.get_publishers_info_by_topic('/unitree_go2/odom')]
        after_plans=[v for v in p.plans if trigger_wall is not None and v['wall']>trigger_wall]
        expected=6 if config['kind']=='C' else 4
        checks=dict(no_error='error' not in result,estimated_odom_only=sources==['go2_lio_nav_odometry'],
            all_prelude_goals_succeeded=all(v.get('status')==4 for v in result['legs'][:-1]),
            final_native_result=bool(result['legs']) and result['legs'][-1].get('status')==expected,
            physical_insertion_acknowledged=ack is not None and all(v['collision_enabled'] for v in ack['actual_usd']),
            moving_trigger=injected and result['trigger']['displacement_m']>=.25,
            commands_bounded=bool(p.commands) and all(0<=c['vx']<=.80001 and abs(c['wz'])<=.60001 for c in p.commands if c['wall']>=start),
            saved_map_unchanged=all(hashlib.sha256(Path(n).read_bytes()).hexdigest()==h for n,h in hashes.items()),
            disarmed_zero=bool(outputs) and all(not s['armed'] and max(abs(v) for v in s['output'])<1e-6 for s in outputs),
            adapter_healthy=not (args.adapter_output/'FAULT.json').exists())
        if config['kind']!='C':
            checks['post_insertion_native_plan']=bool(after_plans)
            checks['guard_not_latched']=not any(s['wall']>=start and s['reason'] not in ['ENABLED','DISARMED'] for s in p.states)
        result.update(checks=checks,passed=all(checks.values()),native_plans_after_insertion=len(after_plans),
                      acceptance='functional dynamic navigation and fail-stop; no precise docking requirement')
        (args.output/'navigation_check.json').write_text(json.dumps(result,indent=2)+'\n')
        (args.output/'telemetry.json').write_text(json.dumps(dict(odom=p.rows,map_poses=p.map_rows,commands=p.commands,
            runtime=p.runtime,guard=p.states,plans=p.plans,amcl_poses=p.amcl_poses))+'\n')
        print(json.dumps(result,indent=2),flush=True)
        p.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
