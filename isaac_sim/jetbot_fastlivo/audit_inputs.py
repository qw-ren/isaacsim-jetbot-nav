"""Read-only actual ROS wire-layout, frame and TF audit; no commands or truth input."""
import argparse,hashlib,json,time,signal
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import Imu,Image,CameraInfo,PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);p.add_argument('--name',default='INPUT_AUDIT');a=p.parse_args()
if not a.name.replace('_','').isalnum():raise ValueError('Invalid report name')
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('jetbot_input_audit');tf_trace=(a.case/(a.name+'_tf.jsonl')).open('x');counts={};errors=[];duplicates={};last={};ranges={};static={};dynamic={};live_pub={};next_pub=0.;start=time.monotonic()
(a.case/(a.name+'_source.py')).write_bytes(Path(__file__).read_bytes())
runtime_wall_s=float(json.loads((a.case/'CASE.json').read_text()).get('runtime_wall_s',600))+30 if(a.case/'CASE.json').exists() else 650
stop_requested=False;fatal_error=None
def request_stop(*_):
 global stop_requested;stop_requested=True
signal.signal(signal.SIGINT,request_stop);signal.signal(signal.SIGTERM,request_stop)
def callback(topic,message):
 counts[topic]=counts.get(topic,0)+1
 try:
  if topic in ['/tf','/tf_static']:
   target=static if topic=='/tf_static'else dynamic
   for t in message.transforms:
    target[t.child_frame_id]=t.header.frame_id
    if topic=='/tf':
     p=t.transform.translation;q=t.transform.rotation;tf_trace.write(json.dumps(dict(parent=t.header.frame_id,child=t.child_frame_id,stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9,xyz=[p.x,p.y,p.z],xyzw=[q.x,q.y,q.z,q.w]))+'\n')
   return
  ts=message.header.stamp.sec+message.header.stamp.nanosec/1e9
  if ts<last.get(topic,-1):raise ValueError('backwards timestamp')
  if ts==last.get(topic,-1):
   if topic!='/jetbot/livo/filter_updates':raise ValueError('duplicate measurement timestamp')
   duplicates[topic]=duplicates.get(topic,0)+1 # LIO and visual corrections may share fusion time.
  last[topic]=ts
  if isinstance(message,PointCloud2):
   if message.header.frame_id!='jetbot/lidar' or message.point_step!=24 or len(message.data)!=message.row_step*message.height:raise ValueError('point layout/frame mismatch')
   expected={'x':0,'y':4,'z':8,'intensity':12,'time':16,'ring':20}
   if {f.name:f.offset for f in message.fields}!=expected:raise ValueError('point field offsets')
   dtype=np.dtype({'names':list(expected),'formats':['<f4']*5+['<u2'],'offsets':list(expected.values()),'itemsize':24});data=np.frombuffer(message.data,dtype=dtype)
   if not len(data)or not all(np.isfinite(data[k]).all()for k in ['x','y','z','time']):raise ValueError('empty/nonfinite point input')
   if data['ring'].max()>=32 or data['time'].min()<0 or data['time'].max()>.165:raise ValueError('ring/time bounds')
   ranges[topic]=dict(min_points=min(len(data),ranges.get(topic,{}).get('min_points',len(data))),max_points=max(len(data),ranges.get(topic,{}).get('max_points',len(data))),stride=24,maximum_point_offset_s=max(float(data['time'].max()),ranges.get(topic,{}).get('maximum_point_offset_s',0)))
  if isinstance(message,Imu):
   if message.header.frame_id!='jetbot/imu' or message.orientation_covariance[0]!=-1.:raise ValueError('IMU frame/orientation-validity mismatch')
   v=message.linear_acceleration;w=message.angular_velocity
   if not np.isfinite([v.x,v.y,v.z,w.x,w.y,w.z]).all():raise ValueError('IMU nonfinite')
  if isinstance(message,Image):
   if message.header.frame_id!='jetbot/camera_optical' or message.encoding!='rgb8' or len(message.data)!=message.height*message.step:raise ValueError('image frame/layout')
  if isinstance(message,CameraInfo):
   if message.header.frame_id!='jetbot/camera_optical' or message.k[0]<=0 or message.k[4]<=0:raise ValueError('CameraInfo frame/intrinsics')
 except Exception as e:
  if len(errors)<20:errors.append(dict(topic=topic,error=str(e)))
for topic,kind in [('/jetbot/imu',Imu),('/jetbot/livo/points',PointCloud2),('/jetbot/image_raw',Image),('/jetbot/camera_info',CameraInfo),('/jetbot/odom',Odometry),('/jetbot/livo/filter_updates',Odometry),('/jetbot/livo/imu_propagated',Odometry),('/tf',TFMessage)]:node.create_subscription(kind,topic,lambda msg,t=topic:callback(t,msg),qos_profile_sensor_data)
qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE);node.create_subscription(TFMessage,'/tf_static',lambda msg:callback('/tf_static',msg),qos)
try:
 while not stop_requested and time.monotonic()-start<runtime_wall_s:
  rclpy.spin_once(node,timeout_sec=.1)
  if time.monotonic()>=next_pub:
   next_pub=time.monotonic()+1.
   live_pub={topic:[x.node_name for x in node.get_publishers_info_by_topic(topic)]for topic in ['/jetbot/odom','/tf','/jetbot/cmd_vel']}
  if (a.case/'RUNNER_RESULT.json').exists():break
except (KeyboardInterrupt,ExternalShutdownException):pass
except Exception as exc:
 fatal_error=repr(exc);raise
finally:
 tf_trace.close()
 expected={'jetbot/imu':'chassis','jetbot/lidar':'chassis','jetbot/camera_optical':'chassis'}
 pub=live_pub
 checks=dict(no_wire_errors=not errors,all_sensors_received=all(counts.get(t,0)>5 for t in ['/jetbot/imu','/jetbot/livo/points','/jetbot/image_raw','/jetbot/camera_info']),sensor_tf_connected=all(static.get(k)==v for k,v in expected.items()),estimated_odom_received=counts.get('/jetbot/odom',0)>50,estimated_odom_tf=dynamic.get('chassis')=='odom',native_corrections_received=counts.get('/jetbot/livo/filter_updates',0)>10)
 result=dict(passed=all(checks.values()) and fatal_error is None,audit_error=fatal_error,stop_requested=stop_requested,checks=checks,counts=counts,errors=errors,expected_same_fusion_time_corrections=duplicates,ranges=ranges,static_tf=static,dynamic_tf=dynamic,publishers_at_end=pub,truth_subscribed=False,commands_published=False,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
 (a.case/(a.name+'.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
