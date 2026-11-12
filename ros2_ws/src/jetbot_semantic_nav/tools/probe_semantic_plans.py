"""Bounded read-only plan endpoints + actual estimated TF, no publishers or truth input."""
import argparse
import json
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Path as PathMessage
from tf2_ros import Buffer,TransformListener

p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);p.add_argument('--seconds',type=float,default=600.)
a=p.parse_args();assert 0<a.seconds<=1800
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
n=Node('semantic_plan_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
b=Buffer();listener=TransformListener(b,n);subscriptions={};stop=False;count=0;size=0;last={}
stream=(a.case/'semantic_plan_endpoints.jsonl').open('x');started=time.monotonic()

def receive(msg,topic):
    global count,size
    now=time.monotonic()
    if now-last.get(topic,0.)<.15 or not msg.poses:return
    last[topic]=now
    def pose(p):
        q=p.pose.orientation
        return dict(xyz=[p.pose.position.x,p.pose.position.y,p.pose.position.z],xyzw=[q.x,q.y,q.z,q.w])
    row=dict(topic=topic,stamp=n.get_clock().now().nanoseconds*1e-9,
        path_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,frame=msg.header.frame_id,
        count=len(msg.poses),first=pose(msg.poses[0]),last=pose(msg.poses[-1]),estimated_pose=None)
    try:
        tr=b.lookup_transform('map','chassis',rclpy.time.Time());v,q=tr.transform.translation,tr.transform.rotation
        row['estimated_pose']=dict(stamp=tr.header.stamp.sec+tr.header.stamp.nanosec*1e-9,xyz=[v.x,v.y,v.z],xyzw=[q.x,q.y,q.z,q.w])
    except Exception:pass
    line=json.dumps(row)+'\n';stream.write(line);stream.flush();size+=len(line.encode());count+=1

def finish(*_):
    global stop
    stop=True

signal.signal(signal.SIGINT,finish);signal.signal(signal.SIGTERM,finish)
try:
    while not stop and time.monotonic()-started<a.seconds and size<16*1024*1024 and not(a.case/'RUNNER_RESULT.json').exists():
        for topic,types in n.get_topic_names_and_types():
            if 'nav_msgs/msg/Path'in types and topic not in subscriptions:
                subscriptions[topic]=n.create_subscription(PathMessage,topic,lambda m,t=topic:receive(m,t),qos_profile_sensor_data)
        rclpy.spin_once(n,timeout_sec=.05)
finally:
    stream.close();(a.case/'SEMANTIC_PLAN_PROBE_RESULT.json').write_text(json.dumps(dict(
        messages=count,bytes=size,topics=list(subscriptions),elapsed_wall_s=time.monotonic()-started,
        readonly=True,truth_input=False,command_publishers=0),indent=2)+'\n')
    n.destroy_node()
    if rclpy.ok():rclpy.shutdown()
