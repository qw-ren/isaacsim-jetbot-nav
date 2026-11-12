"""ROS domain-isolated fixture tests, NOT physical/Isaac navigation acceptance.

Runs the real mapper/manager against deterministic action/service fixtures.
No Twist publisher, no simulator, no connection to domain 51.
"""
import argparse
import copy
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from array import array
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID')!='52':raise RuntimeError('Requires isolated ROS_DOMAIN_ID=52')
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped,PoseStamped
from tf2_ros import StaticTransformBroadcaster
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import SpeedLimit
from lifecycle_msgs.srv import GetState
from jetbot_semantic_nav.mapper import Mapper
from jetbot_semantic_nav.manager import Manager


class Fixture(Node):
    def __init__(self,cfg):
        super().__init__('semantic_test_fixture');self.cfg=cfg;self.stamp=1.;self.free=False
        self.publish_objects=True;self.x=5.;self.requests=[];self.enables=[];self.cancellations=0
        self.reference_x=None;self.extra_objects=[]
        self.speed_limits=[]
        self.create_subscription(SpeedLimit,cfg.get('speed_policy',{}).get('topic','/speed_limit'),lambda m:self.speed_limits.append(m.speed_limit),10)
        self.clock=self.create_publisher(Clock,'/clock',10)
        self.objects=self.create_publisher(String,cfg['topics']['observations'],10)
        self.commands=self.create_publisher(String,cfg['topics']['command'],10)
        self.cancel=self.create_publisher(String,cfg['topics']['cancel'],10)
        self.states=[];self.create_subscription(String,cfg['topics']['status'],lambda m:self.states.append(json.loads(m.data)),10)
        self.tf=StaticTransformBroadcaster(self);transforms=[]
        for frame,position,q in [('chassis',[0.,6.,0.],[0.,0.,0.,1.]),
                                 ('fixture_optical',[1.,2.,0.],[0.,0.,math.sin(math.pi/4),math.cos(math.pi/4)])]:
            tr=TransformStamped();tr.header.frame_id='map';tr.child_frame_id=frame
            tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=position
            tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=q
            transforms.append(tr)
        self.tf.sendTransform(transforms);group=ReentrantCallbackGroup()
        self.create_service(SetBool,cfg['services']['arm'],self.arm,callback_group=group)
        self.create_service(GetState,'/bt_navigator/get_state',self.lifecycle,callback_group=group)
        self.create_service(GetCostmap,cfg['services']['global_costmap'],self.costmap,callback_group=group)
        self.plan_server=ActionServer(self,ComputePathToPose,cfg['actions']['plan'],self.plan,callback_group=group)
        self.nav_server=ActionServer(self,NavigateToPose,cfg['actions']['navigate'],self.navigate,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
        self.create_timer(.05,self.tick)

    def tick(self):
        self.stamp+=.05;msg=Clock();msg.clock.sec=int(self.stamp);msg.clock.nanosec=round((self.stamp-int(self.stamp))*1e9)
        self.clock.publish(msg)
        if self.publish_objects:
            row=dict(object_id='fixture/box',class_name='box',confidence=1.,position=[self.x,6.,.3],
                dimensions=[.7,.5,.6],orientation_xyzw=[0.,0.,0.,1.],source='ground_truth')
            rows=[row,*copy.deepcopy(self.extra_objects)]
            if self.reference_x is not None:
                rows.append(dict(object_id='fixture/shelf2',class_name='shelf',row_index=2,confidence=1.,
                    position=[self.reference_x,6.,1.5],dimensions=[1.,4.,3.],orientation_xyzw=[0.,0.,0.,1.],source='ground_truth'))
            self.objects.publish(String(data=json.dumps(dict(version=1,frame_id='map',stamp=self.stamp,objects=rows))))

    def arm(self,req,res):
        self.enables.append(req.data);res.success=True;return res

    def lifecycle(self,req,res):
        res.current_state.id=3;return res

    def costmap(self,req,res):
        grid=res.map;grid.header.frame_id='map';grid.metadata.resolution=.5
        grid.metadata.size_x=grid.metadata.size_y=80;grid.metadata.origin.position.x=grid.metadata.origin.position.y=-20.
        grid.metadata.origin.orientation.w=1.;grid.data=array('B',[0 if self.free else 253])*6400;return res

    def plan(self,handle):
        result=ComputePathToPose.Result();result.path.header.frame_id='map'
        end=handle.request.goal.pose.position
        for i in range(21):
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=end.x*i/20
            p.pose.position.y=6.+(end.y-6.)*i/20;p.pose.orientation.w=1.;result.path.poses.append(p)
        handle.succeed();return result

    def navigate(self,handle):
        self.requests.append(handle.request)
        end=time.monotonic()+30
        while time.monotonic()<end:
            if handle.is_cancel_requested:
                self.cancellations+=1;handle.canceled();return NavigateToPose.Result()
            time.sleep(.02)
        handle.abort();return NavigateToPose.Result()

    def command(self,task,text='到最近的纸箱那里'):
        self.commands.publish(String(data=json.dumps(dict(version=1,task_id=task,text=text))))


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',required=True,type=Path);p.add_argument('--output',required=True,type=Path);args=p.parse_args()
    assert not args.output.exists();cfg=json.loads(args.config.read_text());cfg['semantic']['object_ttl_s']=2.
    root=Path(tempfile.mkdtemp(prefix='semantic_ros_fixture_'));(root/'CASE.json').write_text(json.dumps(dict(simulation_only=True,navigation=cfg)))
    runtime=json.loads((root/'CASE.json').read_text());runtime['motion_limits']=dict(max_linear_m_s=cfg['max_linear_m_s'],scan_clearance_m=.4)
    (root/'CASE.json').write_text(json.dumps(runtime))
    (root/'GATE_PROGRESS.json').write_text(json.dumps(dict(fault=None,reason='DISARMED',output=[0.,0.],stream_wall_ages=dict(clock=.01,odom=.01,scan=.01))))
    rclpy.init();fixture=Fixture(cfg['semantic']);mapper=Mapper(root);manager=Manager(root)
    executor=MultiThreadedExecutor(num_threads=6)
    for node in(fixture,mapper,manager):executor.add_node(node)
    thread=threading.Thread(target=executor.spin);thread.start();checks=[]
    def until(predicate,label,timeout=15):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():return
            time.sleep(.03)
        raise AssertionError('Timeout '+label)
    def state(task,label):return any(r.get('task_id')==task and r['state']==label for r in fixture.states)
    try:
        until(lambda:manager.ready,'startup',timeout=30)
        fixture.command('unreachable');until(lambda:state('unreachable','UNREACHABLE'),'unreachable')
        assert not fixture.requests and not any(fixture.enables)
        checks.append('all_approaches_blocked_no_navigation_or_enable')
        fixture.free=True;fixture.command('moved');until(lambda:state('moved','NAVIGATING'),'moving start')
        fixture.x+=.3;until(lambda:state('moved','TARGET_INVALID'),'moving cancellation')
        assert fixture.cancellations==1 and fixture.enables[-1]is False
        checks.append('moving_target_disarm_and_native_cancel')
        fixture.command('expired');until(lambda:state('expired','NAVIGATING'),'expiry start')
        fixture.publish_objects=False;until(lambda:state('expired','TARGET_INVALID'),'expiry cancellation')
        assert fixture.cancellations==2 and fixture.enables[-1]is False
        checks.append('expired_target_disarm_and_native_cancel')
        fixture.publish_objects=True;fixture.x=-5.
        until(lambda:any(r['position'][0]==-5. for r in manager.active_objects()),'new object map snapshot')
        fixture.command('align_cancel');until(lambda:state('align_cancel','NAVIGATING'),'native navigation')
        fixture.cancel.publish(String(data=json.dumps(dict(version=1,task_id='align_cancel'))))
        until(lambda:state('align_cancel','CANCELLED'),'alignment cancellation')
        assert fixture.cancellations==3 and fixture.enables[-1]is False
        checks.append('user_cancel_during_native_navigation')
        count=len(fixture.requests);fixture.command('align_cancel');until(lambda:state('align_cancel','DUPLICATE'),'duplicate')
        assert len(fixture.requests)==count;checks.append('duplicate_task_not_replayed')
        fixture.command('missing_reference','走到第二排货架旁边的那个纸箱')
        until(lambda:state('missing_reference','REFERENCE_NOT_FOUND'),'missing reference')
        assert len(fixture.requests)==count;checks.append('missing_reference_no_navigation')
        fixture.reference_x=-7.
        fixture.extra_objects=[dict(object_id='fixture/box2',class_name='box',confidence=1.,
            position=[-5.,7.,.3],dimensions=[.7,.5,.6],orientation_xyzw=[0.,0.,0.,1.],source='ground_truth')]
        until(lambda:len(manager.active_objects())==3,'reference and two candidates')
        fixture.command('ambiguous','走到第二排货架旁边的那个纸箱')
        until(lambda:state('ambiguous','AMBIGUOUS'),'ambiguous relation')
        assert len(fixture.requests)==count and fixture.enables[-1]is False
        checks.append('ambiguous_relation_no_navigation')
        fixture.extra_objects=[];until(lambda:len(manager.active_objects())==2,'extra candidate expiry')
        fixture.command('reference_moved','走到第二排货架旁边的那个纸箱')
        until(lambda:state('reference_moved','NAVIGATING'),'reference navigation')
        fixture.reference_x-=.3
        until(lambda:state('reference_moved','TARGET_INVALID'),'reference movement cancellation')
        assert fixture.cancellations==4 and fixture.enables[-1]is False
        checks.append('reference_movement_disarm_and_native_cancel')
        row=dict(object_id='fixture/vision_sign',class_name='sign',source='vision',confidence=.9,
            position=[2.,3.,.5],orientation_xyzw=[0.,0.,0.,1.],dimensions=[.3,.1,.5])
        fixture.objects.publish(String(data=json.dumps(dict(version=1,frame_id='fixture_optical',stamp=fixture.stamp,objects=[row]))))
        until(lambda:any(r['object_id']=='fixture/vision_sign'for r in manager.active_objects()),'vision TF')
        sign=next(r for r in manager.active_objects()if r['object_id']=='fixture/vision_sign')
        assert math.dist(sign['position'],[-2.,4.,.5])<1e-6;checks.append('vision_timestamped_nonidentity_tf')
        if cfg['semantic'].get('speed_policy'):
            fixture.reference_x=None;fixture.x=5.
            fixture.extra_objects=[dict(object_id='fixture/near_cart',class_name='cart',confidence=1.,
                position=[0.,4.5,.3],dimensions=[.4,.4,.6],orientation_xyzw=[0.,0.,0.,1.],source='ground_truth')]
            until(lambda:any(r['object_id']=='fixture/near_cart'for r in manager.active_objects()),'near object')
            fixture.command('near_speed');until(lambda:state('near_speed','NAVIGATING'),'near speed navigation')
            expected=cfg['semantic']['speed_policy']['near_speed_m_s']
            until(lambda:fixture.speed_limits and abs(fixture.speed_limits[-1]-expected)<1e-8,'native speed limit')
            fixture.extra_objects=[]
            until(lambda:fixture.speed_limits[-1]==0.,'native free-cruise limit clearing')
            assert manager.active_id=='near_speed', 'Cruise restoration must occur during active navigation'
            fixture.cancel.publish(String(data=json.dumps(dict(version=1,task_id='near_speed'))))
            until(lambda:state('near_speed','CANCELLED'),'near speed cancellation')
            until(lambda:fixture.speed_limits[-1]==0.,'native speed limit reset')
            assert fixture.enables[-1]is False;checks.append('native_proximity_speed_limit_cruise_clear_and_disarmed_reset')
        # Deliberately omit the CLI ROS status subscription; manager still receives ROS command.
        # This validates durable acknowledgements without replaying pre-request task history.
        for replay,expected in [(False,'REFERENCE_NOT_FOUND'),(True,'DUPLICATE')]:
            command=[sys.executable,'-m','jetbot_semantic_nav.cli','--case',str(root),'--id','durable_ack',
                '--case-log-only','--timeout','10','到第九排货架旁边的那个纸箱']
            cli=subprocess.run(command,capture_output=True,text=True,timeout=20)
            messages=[]
            for line in cli.stdout.splitlines():
                try:messages.append(json.loads(line))
                except ValueError:pass
            assert messages and messages[-1]['state']==expected and messages[-1]['ack_transport']=='case_event_log',cli.stdout+cli.stderr
        checks.append('cli_durable_ack_without_ros_status_and_duplicate_not_replayed')
        report=dict(passed=True,test_type='isolated_ROS_fixtures_not_Isaac_or_physical_navigation',
            ros_domain_id=52,checks=checks,nav_goals=len(fixture.requests),cancellations=fixture.cancellations,
            temporary_case=str(root),twist_publishers=0,states=fixture.states)
        with args.output.open('x')as f:json.dump(report,f,ensure_ascii=False,indent=2)
        print(json.dumps(dict(passed=True,checks=checks)))
    except Exception as exc:
        with args.output.open('x')as f:json.dump(dict(passed=False,error=repr(exc),checks=checks,
            temporary_case=str(root),states=fixture.states,test_type='isolated_ROS_fixtures_not_Isaac'),f,ensure_ascii=False,indent=2)
        raise
    finally:
        manager.stopping.set()
        for worker in(manager.startup,manager.worker):
            if worker:worker.join(timeout=8)
        executor.shutdown();thread.join(timeout=3)
        for node in(manager,mapper,fixture):node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
