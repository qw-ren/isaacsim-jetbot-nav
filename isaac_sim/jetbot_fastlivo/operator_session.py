"""Bounded operator mission session: native Nav2 actions, no Twist or truth input."""
import argparse,json,time,math,os,signal,hashlib
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import SetBool
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener
p=argparse.ArgumentParser();p.add_argument('--case',required=True,type=Path);a=p.parse_args();root=a.case;cfg=json.loads((root/'CASE.json').read_text());stop_requested=False;mission_trace=None;completed=[];fatal=None
rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('jetbot_operator_session',parameter_overrides=[Parameter('use_sim_time',value=True)]);buffer=Buffer();listener=TransformListener(buffer,node);client=ActionClient(node,NavigateToPose,'navigate_to_pose');arm=node.create_client(SetBool,'/jetbot/arm');initial=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10);requests=root/'mission_requests';requests.mkdir();missions=root/'missions';missions.mkdir()
def stop(*_):
 global stop_requested;stop_requested=True
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
def write(name,row):
 tmp=root/(name+'.tmp');tmp.write_text(json.dumps(row,indent=2)+'\n');tmp.replace(root/name)
def status(state,**extra):write('OPERATOR_STATUS.json',dict(state=state,pid=os.getpid(),wall=time.monotonic(),truth_input=False,velocity_publisher=False,**extra))
def wait(future,timeout,cleanup=False):
 end=time.monotonic()+timeout
 while rclpy.ok()and not future.done()and time.monotonic()<end:
  if stop_requested and not cleanup:raise InterruptedError('Session stopping')
  rclpy.spin_once(node,timeout_sec=.05)
 if not future.done():raise TimeoutError('Native action/service timeout')
 return future.result()
def available(method,timeout,label):
 end=time.monotonic()+timeout
 while not method(timeout_sec=.5):
  if stop_requested:raise InterruptedError('Session stopping')
  if time.monotonic()>end:raise TimeoutError(label+' unavailable')
  rclpy.spin_once(node,timeout_sec=.01)
def disarm():
 request=SetBool.Request();request.data=False;return wait(arm.call_async(request),3,cleanup=True)
def pose():
 t=buffer.lookup_transform('map','chassis',rclpy.time.Time());return[t.transform.translation.x,t.transform.translation.y]
def gate():
 for _ in range(10):
  try:return json.loads((root/'GATE_PROGRESS.json').read_text())
  except (FileNotFoundError,json.JSONDecodeError):rclpy.spin_once(node,timeout_sec=.02)
 raise RuntimeError('Gate status unavailable')
def feedback(message):
 if mission_trace is None:return
 f=message.feedback;p=f.current_pose.pose.position;row=dict(stamp=f.current_pose.header.stamp.sec+f.current_pose.header.stamp.nanosec*1e-9,xy=[p.x,p.y],distance_remaining=f.distance_remaining);mission_trace.write(json.dumps(row)+'\n');mission_trace.flush();write('NAVIGATION_PROGRESS.json',row)
def execute(request_path):
 global mission_trace
 request=json.loads(request_path.read_text());mid=request['id'];assert mid==request_path.stem and mid.replace('_','').isalnum();target=list(map(float,request['goal']));assert len(target)==3 and all(math.isfinite(v)for v in target);timeout=float(request.get('timeout_wall_s',180));assert 0<timeout<=1800
 directory=missions/mid;directory.mkdir();(directory/'REQUEST.json').write_bytes(request_path.read_bytes());result=dict(id=mid,goal=target,passed=False,truth_input=False,velocity_publisher=False,request_sha256=hashlib.sha256(request_path.read_bytes()).hexdigest());handle=None;finished=False;mission_trace=(directory/'feedback.jsonl').open('x')
 try:
  snapshot=gate()
  if snapshot.get('fault')is not None or snapshot['reason']not in('DISARMED','ENABLED'):raise RuntimeError('Gate fault requires a fresh runtime')
  disarm()
  result['start_map_pose']=pose();goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg();goal.pose.pose.position.x,goal.pose.pose.position.y=target[:2];goal.pose.pose.orientation.z=math.sin(target[2]/2);goal.pose.pose.orientation.w=math.cos(target[2]/2);status('WAIT_ACCEPT',mission=mid)
  handle=wait(client.send_goal_async(goal,feedback_callback=feedback),10)
  if not handle.accepted:raise RuntimeError('Native action rejected goal')
  status('WAIT_CONTROLLER',mission=mid);end=time.monotonic()+10
  while True:
   req=SetBool.Request();req.data=True;reply=wait(arm.call_async(req),3)
   if reply.success:break
   if time.monotonic()>end:raise RuntimeError('Gate refused enable: '+reply.message)
   rclpy.spin_once(node,timeout_sec=.05)
  status('RUNNING',mission=mid);response=wait(handle.get_result_async(),timeout);finished=True;result['native_status']=response.status;result['final_map_pose']=pose();result['estimated_goal_error_m']=math.dist(result['final_map_pose'],target[:2]);result['gate_at_result']=gate();result['passed']=response.status==4 and result['gate_at_result']['reason']=='ENABLED'
 except Exception as exc:result['error']=repr(exc)
 finally:
  if handle is not None and handle.accepted and not finished:
   try:result['cancel_reply_received']=wait(handle.cancel_goal_async(),3,cleanup=True)is not None
   except Exception as exc:result['cancel_error']=repr(exc)
  try:result['disarm_success']=bool(disarm().success);result['passed']=result['passed'] and result['disarm_success']
  except Exception as exc:result['disarm_error']=repr(exc);result['passed']=False
  trace=mission_trace;mission_trace=None;trace.close();(directory/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');completed.append(result)
 return result
try:
 status('STARTING')
 available(client.wait_for_server,50,'Nav2 action');available(arm.wait_for_service,20,'Velocity gate')
 lifecycle=node.create_client(GetState,'/bt_navigator/get_state');end=time.monotonic()+50
 while True:
  if lifecycle.wait_for_service(timeout_sec=1)and wait(lifecycle.call_async(GetState.Request()),5).current_state.id==3:break
  if stop_requested:raise InterruptedError('Session stopping')
  if time.monotonic()>end:raise TimeoutError('Nav2 lifecycle not active')
 disarm()
 for _ in range(5):
  m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg();m.pose.pose.position.y=6.;m.pose.pose.orientation.w=1.;m.pose.covariance[0]=m.pose.covariance[7]=.0025;m.pose.covariance[35]=.0004;initial.publish(m);end=time.monotonic()+.25
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
 end=time.monotonic()+40
 while True:
  if stop_requested:raise InterruptedError('Session stopping')
  try:initial_map=pose();break
  except Exception:
   if time.monotonic()>end:raise TimeoutError('Map pose unavailable')
   rclpy.spin_once(node,timeout_sec=.05)
 write('OPERATOR_READY.json',dict(pid=os.getpid(),case=str(root),ready=True,initial_pose_source='declared CAD spawn, initialized once only',initial_map_pose=initial_map,truth_input=False,velocity_publisher=False));status('IDLE')
 while not stop_requested and rclpy.ok():
  rclpy.spin_once(node,timeout_sec=.1)
  pending=[f for f in sorted(requests.glob('*.json'))if not(missions/f.stem/'RESULT.json').exists()]
  if pending:
   execute(pending[0]);snapshot=gate();status('FAULT_REQUIRES_RESTART'if snapshot.get('fault')else'IDLE',gate_reason=snapshot['reason'],gate_fault=snapshot.get('fault'))
except InterruptedError:pass
except Exception as exc:fatal=repr(exc);raise
finally:
 try:disarm()
 except Exception:pass
 status('STOPPED',error=fatal);write('OPERATOR_RESULT.json',dict(error=fatal,stop_requested=stop_requested,mission_count=len(completed),passed_missions=sum(v['passed']for v in completed),missions=completed,truth_input=False,velocity_publisher=False));node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
