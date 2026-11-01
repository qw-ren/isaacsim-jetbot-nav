"""Native Nav2 configured route; CAD initial pose, no truth/velocity input."""
import argparse,json,time,math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from nav2_msgs.srv import GetCostmap
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath, OccupancyGrid
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import SetBool
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer,TransformListener
from builtin_interfaces.msg import Time
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();cfg=json.loads((a.case/'CASE.json').read_text()).get('navigation',{});goals=cfg.get('goals',[[1.5,6.,0.]]);through=len(goals)>1;action=NavigateThroughPoses if through else NavigateToPose;rclpy.init();node=Node('jetbot_navigation_check',parameter_overrides=[Parameter('use_sim_time',value=True)]);buffer=Buffer();listener=TransformListener(buffer,node);client=ActionClient(node,action,'navigate_through_poses' if through else 'navigate_to_pose');arm=node.create_client(SetBool,'/jetbot/arm');initial=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10);result=dict(passed=False,goal=goals[-1],goals=goals,action=action.__name__,truth_input=False,velocity_publisher=False,initial_pose_source='declared CAD scene spawn, one initialization only',scope='JetBot FAST-LIVO + AMCL + native Nav2 configured route')
def wait(future,timeout):
 end=time.monotonic()+timeout
 while rclpy.ok()and time.monotonic()<end and not future.done():rclpy.spin_once(node,timeout_sec=.05)
 if not future.done():raise TimeoutError('ROS action/service timeout')
 return future.result()
trace=(a.case/'navigation_feedback.jsonl').open('x')
paths=(a.case/'navigation_paths.jsonl').open('x')
def record_path(m,topic):
 paths.write(json.dumps(dict(topic=topic,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,frame=m.header.frame_id,xy=[[p.pose.position.x,p.pose.position.y] for p in m.poses]))+'\n');paths.flush()
path_sub=node.create_subscription(NavPath,'/plan',lambda m:record_path(m,'/plan'),10)
local_sub=node.create_subscription(NavPath,'/local_plan',lambda m:record_path(m,'/local_plan'),10)
costmap_log=(a.case/'costmap_observations.jsonl').open('x');saved_costmaps=set();costmap_counter=[0];(a.case/'costmap_snapshots').mkdir()
def record_costmap(m):
 ts=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;grid=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width);events=[]
 f=a.case/'obstacle_events.jsonl'
 if f.exists():events=[json.loads(l)for l in f.read_text().splitlines()]
 costmap_counter[0]+=1;np.savez_compressed(a.case/'costmap_snapshots'/('%05d.npz'%costmap_counter[0]),data=grid,origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution,stamp=ts)
 label='initial' if not events else events[-1]['kind']
 row=dict(stamp=ts,frame=m.header.frame_id,occupied_cells=int(np.sum(grid>=65)),events_seen=len(events))
 if label not in saved_costmaps and (not events or ts-events[-1]['sim']>=2.):
  np.savez_compressed(a.case/('global_costmap_'+label+'.npz'),data=grid,origin=[m.info.origin.position.x,m.info.origin.position.y],resolution=m.info.resolution,stamp=ts);saved_costmaps.add(label)
 costmap_log.write(json.dumps(row)+'\n');costmap_log.flush()
costmap_sub=node.create_subscription(OccupancyGrid,'/global_costmap/costmap',record_costmap,2)
def feedback(message):
 f=message.feedback;p=f.current_pose.pose.position;q=f.current_pose.pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));trace.write(json.dumps(dict(stamp=f.current_pose.header.stamp.sec+f.current_pose.header.stamp.nanosec*1e-9,xy=[p.x,p.y],yaw=yaw,distance_remaining=f.distance_remaining))+'\n');trace.flush()
def progress(message):
 f=message.feedback;p=f.current_pose.pose.position;row=dict(stamp=f.current_pose.header.stamp.sec+f.current_pose.header.stamp.nanosec*1e-9,xy=[p.x,p.y]);tmp=a.case/'NAVIGATION_PROGRESS.tmp';tmp.write_text(json.dumps(row)+'\n');tmp.replace(a.case/'NAVIGATION_PROGRESS.json');feedback(message)
def pose():
 t=buffer.lookup_transform('map','chassis',rclpy.time.Time());return [t.transform.translation.x,t.transform.translation.y]
try:
 if not client.wait_for_server(timeout_sec=50):raise TimeoutError('Nav2 action server unavailable')
 if not arm.wait_for_service(timeout_sec=20):raise TimeoutError('Velocity gate service unavailable')
 lifecycle=node.create_client(GetState,'/bt_navigator/get_state');end=time.monotonic()+50
 while True:
  if lifecycle.wait_for_service(timeout_sec=1):
   pending=lifecycle.call_async(GetState.Request())
   try:state=wait(pending,min(5.,max(.01,end-time.monotonic())))
   except TimeoutError:
    pending.cancel();result['lifecycle_query_retries']=result.get('lifecycle_query_retries',0)+1
    if time.monotonic()>end:raise TimeoutError('Nav2 lifecycle not active within original 50s budget')
    continue
   if state.current_state.id==3:break
  if time.monotonic()>end:raise TimeoutError('Nav2 lifecycle not active')
  rclpy.spin_once(node,timeout_sec=.1)
 # AMCL receives the declared scene start. This is not fitted to simulator state.
 for i in range(5):
  m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg();m.pose.pose.position.y=6.;m.pose.pose.orientation.w=1.;m.pose.covariance[0]=m.pose.covariance[7]=.0025;m.pose.covariance[35]=.0004;initial.publish(m)
  end=time.monotonic()+.25
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
 end=time.monotonic()+40
 while True:
  rclpy.spin_once(node,timeout_sec=.05)
  try:result['start_map_pose']=pose();break
  except Exception:
   if time.monotonic()>end:raise TimeoutError('AMCL map->chassis unavailable')
 goal=action.Goal();poses=[]
 for x,y,yaw in goals:
  m=PoseStamped();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg();m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.orientation.z=math.sin(yaw/2);m.pose.orientation.w=math.cos(yaw/2);poses.append(m)
 if through:goal.poses=poses
 else:goal.pose=poses[0]
 handle=wait(client.send_goal_async(goal,feedback_callback=progress),10)
 if not handle.accepted:raise RuntimeError('Nav2 rejected goal')
 # The active action may plan while the gate remains disarmed and outputs zero.
 end=time.monotonic()+10
 while True:
  request=SetBool.Request();request.data=True;reply=wait(arm.call_async(request),3)
  if reply.success:break
  if time.monotonic()>end:raise RuntimeError('Gate refused enable: '+reply.message)
  rclpy.spin_once(node,timeout_sec=.1)
 response=wait(handle.get_result_async(),float(cfg.get('timeout_wall_s',120)));result['native_status']=response.status;result['final_map_pose']=pose();result['estimated_goal_error_m']=math.hypot(result['final_map_pose'][0]-goals[-1][0],result['final_map_pose'][1]-goals[-1][1])
 gate=json.loads((a.case/'GATE_PROGRESS.json').read_text());result['gate']=gate;expected=int(cfg.get('expected_native_status',4));assert expected in(4,6);result['expected_native_status']=expected;result['navigation_succeeded']=response.status==4;result['passed']=response.status==expected and gate['fault']is None and gate['reason']=='ENABLED'
except Exception as exc:
 result['error']=repr(exc)
 try:result['final_map_pose']=pose()
 except Exception:pass
finally:
 try:
  request=SetBool.Request();request.data=False;wait(arm.call_async(request),3)
 except Exception:pass
 # Read final obstacle-layer state after disarming; no actuation/ground truth.
 try:
  snapshot_client=node.create_client(GetCostmap,'/global_costmap/get_costmap')
  if snapshot_client.wait_for_service(timeout_sec=1.):
   response_map=wait(snapshot_client.call_async(GetCostmap.Request()),2.);m=response_map.map;info=m.metadata;raw=np.asarray(m.data,dtype=np.uint8).reshape(info.size_y,info.size_x);grid=np.where(raw==255,-1,np.where(raw==254,100,np.where(raw==253,99,np.rint(raw.astype(float)*98/252)))).astype(np.int8)
   np.savez_compressed(a.case/'global_costmap_terminal.npz',data=grid,origin=[info.origin.position.x,info.origin.position.y],resolution=info.resolution,stamp=info.update_time.sec+info.update_time.nanosec*1e-9)
 except Exception as exc:result['terminal_snapshot_error']=repr(exc)
 trace.close();paths.close();costmap_log.close()
 (a.case/'NAVIGATION_RESULT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
