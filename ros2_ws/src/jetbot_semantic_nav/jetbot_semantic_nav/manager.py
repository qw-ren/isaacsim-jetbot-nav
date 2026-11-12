"""One semantic mission owner; native Nav2 actions and existing arm service only."""
import argparse
import copy
import json
import math
import signal
import threading
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import SpeedLimit
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener
from .config import load,write_json,claim
from .core import (parse_command,approaches,distance_to_object,path_length,sample_path,
                   grid_cost,validate_observation,resolve_query,rank_reachable,proximity_speed_limit,native_speed_limit_value)


class MissionStopped(Exception):
    pass


class Manager(Node):
    def __init__(self,case):
        super().__init__('semantic_goal_manager',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.case=Path(case);self.ownership=claim(case,'semantic_goal_manager');self.runtime,self.cfg=load(case);self.topics=self.cfg['topics']
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.planner=ActionClient(self,ComputePathToPose,self.cfg['actions']['plan'])
        self.navigator=ActionClient(self,NavigateToPose,self.cfg['actions']['navigate'])
        self.arm=self.create_client(SetBool,self.cfg['services']['arm'])
        self.costmap=self.create_client(GetCostmap,self.cfg['services']['global_costmap'])
        self.lifecycle=self.create_client(GetState,'/bt_navigator/get_state')
        self.initial=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.pub=self.create_publisher(String,self.topics['status'],10)
        self.goal_pub=self.create_publisher(PoseStamped,self.topics['selected_goal'],10)
        self.speed_pub=(self.create_publisher(SpeedLimit,self.cfg['speed_policy']['topic'],10)if self.cfg.get('speed_policy')else None)
        self.last_speed_limit=None;self.speed_published_wall=0.
        self.create_subscription(String,self.topics['objects'],self.objects_cb,10)
        self.create_subscription(String,self.topics['command'],self.command_cb,10)
        self.create_subscription(String,self.topics['cancel'],self.cancel_cb,10)
        self.rows=[];self.rows_wall=0.;self.rows_stamp=0.;self.lock=threading.Lock()
        self.stopping=threading.Event();self.cancel=threading.Event();self.ready=False
        self.worker=None;self.active_id=None;self.seen=set();self.cancel_reason='user_cancelled'
        self.log=(self.case/'semantic_tasks.jsonl').open('x')
        self.tasks=self.case/'semantic_tasks';self.tasks.mkdir()
        self.startup=threading.Thread(target=self.prepare);self.startup.start()

    def emit(self,state,task_id=None,**extra):
        row=dict(version=1,task_id=task_id,state=state,wall=time.monotonic(),
                 stamp=self.get_clock().now().nanoseconds*1e-9,mode=self.cfg['mode'],
                 robot_truth_input=False,velocity_publisher=False,**extra)
        with self.lock:
            self.log.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');self.log.flush()
            write_json(self.case/'SEMANTIC_STATUS.json',row)
        self.pub.publish(String(data=json.dumps(row,ensure_ascii=False)))

    def objects_cb(self,msg):
        try:
            packet=json.loads(msg.data)
            if packet['version']!=1 or packet['frame_id']!='map':raise ValueError('Invalid object map')
            rows=[validate_observation(r)|{'last_seen':float(r['last_seen'])} for r in packet['objects']]
            if len({r['object_id']for r in rows})!=len(rows):raise ValueError('Duplicate id')
            with self.lock:
                self.rows=rows;self.rows_wall=time.monotonic();self.rows_stamp=float(packet['stamp'])
        except Exception as exc:self.get_logger().warning(str(exc))

    def active_objects(self):
        now=self.get_clock().now().nanoseconds*1e-9
        with self.lock:
            if time.monotonic()-self.rows_wall>self.cfg['object_ttl_s'] or not -.1<=now-self.rows_stamp<=self.cfg['object_ttl_s']:
                return []
            return copy.deepcopy([r for r in self.rows if -.1<=now-r['last_seen']<=self.cfg['object_ttl_s']])

    def check_stop(self):
        if self.stopping.is_set():raise MissionStopped('session_stopping')
        if self.cancel.is_set():raise MissionStopped(self.cancel_reason)

    def wait(self,future,timeout,cleanup=False):
        end=time.monotonic()+timeout
        while not future.done():
            if not cleanup:self.check_stop()
            if time.monotonic()>end:raise TimeoutError('ROS action/service timed out')
            self.stopping.wait(.02) if not cleanup else time.sleep(.02)
        result=future.result()
        if result is None:raise RuntimeError('ROS returned no result')
        return result

    def arm_gate(self,enable):
        req=SetBool.Request();req.data=enable
        result=self.wait(self.arm.call_async(req),3,cleanup=not enable)
        if enable and not result.success:raise RuntimeError('Gate refused enable: '+result.message)
        return result.success

    def gate(self):
        return json.loads((self.case/'GATE_PROGRESS.json').read_text())

    def prepare(self):
        try:
            self.emit('STARTING')
            deadline=time.monotonic()+100
            while time.monotonic()<deadline:
                self.check_stop()
                if self.planner.server_is_ready() and self.navigator.server_is_ready() and self.arm.service_is_ready() and self.costmap.service_is_ready() and self.lifecycle.service_is_ready():
                    if self.wait(self.lifecycle.call_async(GetState.Request()),5).current_state.id==3:break
                time.sleep(.2)
            else:raise TimeoutError('Nav2 not ready')
            if not self.arm_gate(False):raise RuntimeError('Could not disarm')
            for _ in range(5):
                msg=PoseWithCovarianceStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
                # Same declared CAD spawn as existing operator_session; never robot truth.
                x,y,yaw=self.cfg['declared_spawn_map'];msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
                msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
                msg.pose.covariance[0]=msg.pose.covariance[7]=.0025;msg.pose.covariance[35]=.0004
                self.initial.publish(msg);time.sleep(.25)
            deadline=time.monotonic()+90;healthy_since=None
            while time.monotonic()<deadline:
                self.check_stop()
                if (self.case/'ODOM_FAULT.json').exists():raise RuntimeError('Estimator output fault; fresh case required')
                try:
                    self.buffer.lookup_transform('map','chassis',rclpy.time.Time())
                    snapshot=self.gate();ages=snapshot.get('stream_wall_ages',{})
                    healthy=(not snapshot.get('fault')and all(ages.get(k,999.)<limit for k,limit in [('clock',.2),('odom',.2),('scan',.6)]))
                    if healthy and self.active_objects() and self.get_clock().now().nanoseconds*1e-9>=self.cfg.get('ready_after_sim_s',10.):
                        healthy_since=healthy_since or time.monotonic()
                        if time.monotonic()-healthy_since>=self.cfg.get('healthy_ready_wall_s',2.):break
                    else:healthy_since=None
                except Exception:pass
                time.sleep(.1)
            else:raise TimeoutError('Localization or objects unavailable')
            self.ready=True;self.emit('READY',initial_pose_source='declared_CAD_spawn_once')
        except Exception as exc:self.emit('STARTUP_FAILED',error=str(exc))

    def command_cb(self,msg):
        try:
            request=json.loads(msg.data);task_id=request['task_id']
            if not isinstance(task_id,str) or not 1<=len(task_id)<=64 or not task_id.replace('_','').isalnum():raise ValueError('Invalid task id')
            if request.get('version')!=1:raise ValueError('Unsupported version')
            if type(request.get('dry_run',False))is not bool:raise ValueError('dry_run must be boolean')
            if task_id in self.seen:
                self.emit('DUPLICATE',task_id);return
            if not self.ready or self.active_id is not None:
                self.emit('BUSY' if self.ready else 'NOT_READY',task_id);return
            parsed=parse_command(request['text']);self.seen.add(task_id)
            self.cancel.clear();self.cancel_reason='user_cancelled';self.active_id=task_id
            self.worker=threading.Thread(target=self.execute,args=(request,parsed));self.worker.start()
        except Exception as exc:self.emit('INVALID_COMMAND',locals().get('task_id'),error=str(exc))

    def cancel_cb(self,msg):
        try:
            request=json.loads(msg.data)
            if request.get('version')==1 and request['task_id']==self.active_id:
                self.cancel_reason='user_cancelled';self.cancel.set()
        except Exception:pass

    def pose(self,target):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=target[:2]
        msg.pose.orientation.z=math.sin(target[2]/2);msg.pose.orientation.w=math.cos(target[2]/2)
        return msg

    def target_valid(self,target):
        row=next((r for r in self.active_objects()if r['object_id']==target['object_id']),None)
        if row is None:raise MissionStopped('target_expired')
        if row.get('row_index')!=target.get('row_index'):raise MissionStopped('target_reference_changed')
        if math.dist(row['position'],target['position'])>self.cfg['target_move_m']:
            raise MissionStopped('target_moved')
        dot=abs(sum(a*b for a,b in zip(row['orientation_xyzw'],target['orientation_xyzw'])))
        delta=2*math.acos(min(1.,dot))
        if abs(delta)>self.cfg['target_rotate_rad'] or max(abs(a-b)for a,b in zip(row['dimensions'],target['dimensions']))>.1:
            raise MissionStopped('target_geometry_changed')

    def apply_speed_policy(self,task_id,force=False):
        if self.speed_pub is None:return
        policy=self.cfg['speed_policy']
        if self.speed_pub.get_subscription_count()<1 or len(self.get_publishers_info_by_topic(policy['topic']))!=1:
            raise RuntimeError('Native speed-limit subscriber missing or competing speed-limit owner')
        if not force and time.monotonic()-self.speed_published_wall<.2:return
        tr=self.buffer.lookup_transform('map','chassis',rclpy.time.Time()).transform.translation
        limit,distance,reason=proximity_speed_limit(self.active_objects(),[tr.x,tr.y],policy,
            self.runtime['motion_limits']['max_linear_m_s'],self.last_speed_limit)
        msg=SpeedLimit();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='map'
        msg.percentage=False;msg.speed_limit=native_speed_limit_value(limit,self.runtime['motion_limits']['max_linear_m_s']);self.speed_pub.publish(msg)
        self.speed_published_wall=time.monotonic()
        if limit!=self.last_speed_limit:
            self.emit('SPEED_LIMIT',task_id,limit_m_s=limit,native_message_speed_limit=msg.speed_limit,
                nearest_prop_surface_m=distance,reason=reason,native_interface=True)
        self.last_speed_limit=limit

    def reset_speed_policy(self,task_id):
        if self.speed_pub is None:return
        # Restore only our own cap, only after disarm and terminal action confirmation.
        if len(self.get_publishers_info_by_topic(self.cfg['speed_policy']['topic']))!=1:
            self.ready=False;raise RuntimeError('Competing speed-limit owner; disarmed without overriding its limit')
        msg=SpeedLimit();msg.header.stamp=self.get_clock().now().to_msg();msg.percentage=False;msg.speed_limit=0.
        self.speed_pub.publish(msg);self.last_speed_limit=None
        self.emit('SPEED_LIMIT_RESET',task_id,reason='disarmed_and_action_terminal',native_no_speed_limit=True)

    def plan(self,target):
        handle=None;done=False
        try:
            goal=ComputePathToPose.Goal();goal.goal=self.pose(target);goal.planner_id='GridBased';goal.use_start=False
            # Always receive acceptance, even after cancellation, so it can be cancelled below.
            handle=self.wait(self.planner.send_goal_async(goal),5,cleanup=True)
            self.check_stop()
            if not handle.accepted:return None
            response=self.wait(handle.get_result_async(),self.cfg['plan_timeout_wall_s']);done=True
            if response.status!=4:return None
            points=[[p.pose.position.x,p.pose.position.y]for p in response.result.path.poses]
            if not points or math.dist(points[-1],target[:2])>.05:return None
            return points
        finally:
            if handle and handle.accepted and not done:
                self.wait(handle.cancel_goal_async(),3,cleanup=True)

    def navigate(self,target,obj,task_id,phase,references=()):
        """Native action phase; disarm before cancellation and require terminal state."""
        handle=None;finished=False
        try:
            self.apply_speed_policy(task_id,force=True)
            if self.speed_pub is not None:time.sleep(.15)
            goal=NavigateToPose.Goal();goal.pose=self.pose(target)
            handle=self.wait(self.navigator.send_goal_async(goal),10,cleanup=True)
            self.check_stop()
            if not handle.accepted:raise RuntimeError('Nav2 rejected '+phase)
            end=time.monotonic()+10
            while True:
                self.check_stop();self.target_valid(obj)
                for reference in references:self.target_valid(reference)
                self.apply_speed_policy(task_id)
                try:self.arm_gate(True);break
                except RuntimeError:
                    if self.gate().get('fault') or time.monotonic()>end:raise
                    time.sleep(.1)
            self.emit(phase,task_id,object_id=obj['object_id'],goal=target)
            future=handle.get_result_async();deadline=time.monotonic()+self.cfg['navigation_timeout_wall_s']
            while not future.done():
                self.check_stop();self.target_valid(obj)
                for reference in references:self.target_valid(reference)
                self.apply_speed_policy(task_id)
                if self.gate().get('fault'):raise RuntimeError('Safety gate: '+self.gate()['fault'])
                if time.monotonic()>deadline:raise TimeoutError('Navigation timeout')
                time.sleep(.05)
            response=future.result();finished=True
            return response
        finally:
            disarm_error=None
            try:
                if not self.arm_gate(False):raise RuntimeError('Disarm failed')
            except Exception as exc:self.ready=False;disarm_error=exc
            if handle and handle.accepted and not finished:
                try:
                    reply=self.wait(handle.cancel_goal_async(),3,cleanup=True)
                    terminal=self.wait(handle.get_result_async(),5,cleanup=True)
                    self.emit('CANCEL_ACK',task_id,cancel_accepted=bool(reply.goals_canceling),native_status=terminal.status)
                    if terminal.status not in(4,5,6):raise RuntimeError('Action not terminal')
                except Exception:
                    self.ready=False;raise
            if disarm_error:raise disarm_error
            self.reset_speed_policy(task_id)

    def execute(self,request,parsed):
        task_id=request['task_id']
        result=dict(request=request,parsed=parsed,passed=False,mode=self.cfg['mode'],speed_policy=self.cfg.get('speed_policy'))
        state='FAILED'
        try:
            self.emit('QUERYING',task_id,parsed=parsed)
            if self.gate().get('fault'):raise RuntimeError('Gate fault requires fresh session')
            if not self.arm_gate(False):raise RuntimeError('Cannot disarm')
            objects=self.active_objects()
            candidates,relation_audit,query_error=resolve_query(objects,parsed,self.cfg.get('near_shelf_distance_m',1.5))
            result['relation_audit']=relation_audit
            references=relation_audit['reference_objects']
            result['objects']=candidates
            if query_error:state=query_error;return
            clearance=self.cfg['planning_radius_m']+self.cfg['footprint_padding_m']
            grid=self.wait(self.costmap.call_async(GetCostmap.Request()),5).map
            if grid.header.frame_id!='map':raise RuntimeError('Unexpected costmap frame')
            ranked=[];audits=[];deadline=time.monotonic()+self.cfg['query_timeout_wall_s']
            self.emit('PLANNING',task_id,candidates=candidates)
            for obj in candidates:
                for target in approaches(obj,self.cfg['planning_radius_m'],self.cfg['footprint_padding_m'],self.cfg['approach_margin_m'],self.cfg['approach_samples']):
                    self.check_stop();self.target_valid(obj)
                    for reference in references:self.target_valid(reference)
                    if time.monotonic()>deadline:raise TimeoutError('Semantic query budget exceeded')
                    audit=dict(object_id=obj['object_id'],goal=target);audits.append(audit)
                    if grid_cost(grid,target)>self.cfg.get('max_goal_cost',20) or any(distance_to_object(target,r)<clearance+self.cfg['approach_margin_m']-.001 for r in objects):
                        audit['rejected']='unsafe_goal';continue
                    points=self.plan(target)
                    if points is None:audit['rejected']='native_no_path_or_endpoint_tolerance';continue
                    # Unknown/fatal cost or any registered object crossing rejects the whole path.
                    unsafe=None
                    for p in sample_path(points):
                        cost=grid_cost(grid,p)
                        crossing=[r['object_id']for r in objects if distance_to_object(p,r)<clearance]
                        if cost>=253 or crossing:
                            unsafe=dict(point=p,cost=cost,object_ids=crossing);break
                    if unsafe:
                        audit.update(rejected='unsafe_path',first_unsafe=unsafe);continue
                    length=path_length(points);audit['path_length_m']=length
                    ranked.append((length,obj['object_id'],target,obj,points))
            result['approach_audit']=audits
            if not ranked:state='UNREACHABLE';return
            ranked=rank_reachable(ranked,parsed['selection'])
            result['object_ranking']=[dict(object_id=r[1],shortest_approach_path_m=r[0],goal=r[2])for r in ranked]
            length,_,target,obj,points=ranked[0]
            self.target_valid(obj);self.check_stop()
            result.update(selected_object=obj,goal=target,path_length_m=length,planned_path=points)
            self.goal_pub.publish(self.pose(target));self.emit('SELECTED',task_id,object=obj,goal=target,path_length_m=length)
            if request.get('dry_run',False):state='DRY_RUN_COMPLETE';return
            response=self.navigate(target,obj,task_id,'NAVIGATING',references);result['native_status']=response.status
            state='SUCCEEDED' if response.status==4 else 'NAVIGATION_FAILED'
            result['passed']=response.status==4 and not self.gate().get('fault')
            tr=self.buffer.lookup_transform('map','chassis',rclpy.time.Time()).transform
            result['final_estimated_position']=[tr.translation.x,tr.translation.y,tr.translation.z]
            result['gate_at_result']=self.gate()
        except MissionStopped as exc:
            state='CANCELLED' if str(exc)=='user_cancelled' else 'TARGET_INVALID' if str(exc).startswith('target_') else 'STOPPED'
            result['reason']=str(exc)
        except Exception as exc:
            result['error']=repr(exc)
            if isinstance(exc,TimeoutError):self.ready=False
        finally:
            # Disable output before waiting on action cancellation. Never publish Twist.
            try:
                result['disarmed']=self.arm_gate(False)
                if not result['disarmed']:raise RuntimeError('Disarm refused')
            except Exception as exc:
                result['disarm_error']=repr(exc);result['passed']=False;state='SAFETY_CLEANUP_FAILED';self.ready=False
            result['state']=state;result['finished_stamp']=self.get_clock().now().nanoseconds*1e-9
            write_json(self.tasks/(task_id+'.json'),result)
            self.active_id=None
            if self.gate().get('fault'):self.ready=False
            self.emit(state,task_id,result=result)


def main():
    p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);args=p.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Manager(args.case)
    def stop(*_):node.stopping.set()
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while not node.stopping.is_set():rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.stopping.set();deadline=time.monotonic()+15
        while (node.startup.is_alive() or(node.worker and node.worker.is_alive()))and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        node.log.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
