#!/usr/bin/env python3
"""Read-only lifecycle, parameters and publisher ownership audit."""
import argparse,json,time
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
rclpy.init();n=rclpy.create_node('go2_motion_runtime_audit',parameter_overrides=[Parameter('use_sim_time',value=True)]);edges={};guard=[]
def tf(m):
 for t in m.transforms:
  k=t.header.frame_id+' -> '+t.child_frame_id;edges[k]=edges.get(k,0)+1
subs=[n.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data),n.create_subscription(String,'/nav_m1/guard_state',lambda m:guard.append(json.loads(m.data)),10)]
def wait(f):
 until=time.monotonic()+10
 while time.monotonic()<until and not f.done():rclpy.spin_once(n,timeout_sec=.05)
 if not f.done():raise RuntimeError('Service request timeout')
 return f.result()
result={'states':{},'parameters':{}}
for name in ['map_server','planner_server','controller_server','bt_navigator','local_costmap/local_costmap','global_costmap/global_costmap']:
 c=n.create_client(GetState,'/'+name+'/get_state')
 if not c.wait_for_service(timeout_sec=3):raise RuntimeError('Missing '+name)
 s=wait(c.call_async(GetState.Request())).current_state;result['states'][name]={'id':s.id,'label':s.label};n.destroy_client(c)
 c=n.create_client(GetParameters,'/'+name+'/get_parameters');r=GetParameters.Request();r.names=['use_sim_time']
 if name=='controller_server':r.names+=['odom_topic','FollowPath.min_vel_x','FollowPath.max_vel_x','FollowPath.min_speed_xy','controller_frequency','goal_checker.plugin','goal_checker.xy_goal_tolerance','goal_checker.yaw_goal_tolerance','goal_checker.trans_stopped_velocity','goal_checker.rot_stopped_velocity','FollowPath.RotateToGoal.lookahead_time','FollowPath.sim_time','FollowPath.xy_goal_tolerance']
 c.wait_for_service(timeout_sec=3);values=wait(c.call_async(r)).values;result['parameters'][name]={k:(v.bool_value if v.type==1 else v.double_value if v.type==3 else v.string_value) for k,v in zip(r.names,values)};n.destroy_client(c)
until=time.monotonic()+5
while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
result['publishers']={t:[x.node_namespace.rstrip('/')+'/'+x.node_name for x in n.get_publishers_info_by_topic(t)] for t in ['/tf','/scan','/nav_m1/cmd_vel','/unitree_go2/cmd_vel','/cmd_vel','/nav_m1/runtime_cmd']};result['tf_edges_observed']=edges;result['guard_last']=guard[-1] if guard else None
result['checks']={'all_active':all(x['id']==3 for x in result['states'].values()),'all_sim_time':all(x['use_sim_time'] is True for x in result['parameters'].values()),'single_velocity_output':result['publishers']['/unitree_go2/cmd_vel']==['/go2_sim_velocity_guard'],'single_controller_input':result['publishers']['/nav_m1/cmd_vel']==['/controller_server'],'no_alias_publisher':not result['publishers']['/cmd_vel'],'disarmed_zero':bool(guard) and not guard[-1]['armed'] and guard[-1]['output']==[0.,0.],'two_expected_tf_publishers':sorted(result['publishers']['/tf'])==['/go2_scan_ground_frame','/robot_data_manager']}
result['passed']=all(result['checks'].values());args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
raise SystemExit(0 if result['passed'] else 1)
