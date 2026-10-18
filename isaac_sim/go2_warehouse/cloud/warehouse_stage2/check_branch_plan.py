"""Stationary B branch switch: physical barrier -> scan -> native Navfn; no velocity publisher."""
import argparse, hashlib, json, math, time
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
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--fixture',type=Path,required=True);ap.add_argument('--map',type=Path,required=True);a=ap.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    config=json.loads((a.fixture/'box_config.json').read_text())
    files=[a.map,a.map.parent/'map.pgm'];hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    rclpy.init();node=rclpy.create_node('go2_branch_plan_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
    received={};positions=[];scans=[]
    durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs=[node.create_subscription(OccupancyGrid,'/map',lambda m:received.update(map=m),durable),node.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:received.update(costmap=m),durable),node.create_subscription(Odometry,'/unitree_go2/odom',lambda m:positions.append([m.pose.pose.position.x,m.pose.pose.position.y]),qos_profile_sensor_data),node.create_subscription(LaserScan,'/scan',lambda m:scans.append(m),qos_profile_sensor_data),node.create_subscription(Clock,'/clock',lambda m:received.update(sim=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)]
    buffer=Buffer(node=node);listener=TransformListener(buffer,node);planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    result={'mode':'stationary_B_two_branches_prior_divider_and_new_barrier','velocity_commands_sent':False,'fixture':config,'map_file_hashes_before':hashes}
    def wait(predicate,timeout=90):
        deadline=time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if not predicate():raise TimeoutError('B observation/action timeout')
    def fixture(enabled,phase):
        cmd={'id':a.output.name+'_'+phase,'enabled':enabled};p=a.fixture/'obstacle_request.json';t=p.with_suffix('.tmp');t.write_text(json.dumps(cmd));t.replace(p)
        response=a.fixture/'obstacle_state.json';wait(lambda:response.exists() and json.loads(response.read_text()).get('id')==cmd['id'])
        return json.loads(response.read_text())
    def settle():
        start=received['sim'];count=len(scans);wait(lambda:received['sim']-start>=4 and len(scans)-count>=10,60)
    def alignment():
        t=buffer.lookup_transform('map','odom',Time()).transform;q=t.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));return np.array([t.translation.x,t.translation.y]),np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
    def clearance(points,box):
        c=np.array(box['center'][:2]);h=np.array(box['size'][:2])/2;d=np.maximum(np.maximum(c-h-points,points-(c+h)),0);return float(np.linalg.norm(d,axis=1).min())
    def snapshot(phase):
        translation,rotation=alignment();grid=received['costmap'];info=grid.info;values=np.asarray(grid.data,dtype=np.int8).reshape(info.height,info.width)
        request=ComputePathToPose.Goal();request.goal.header.frame_id='map';request.goal.header.stamp=node.get_clock().now().to_msg();target=rotation@np.array([6.5,0.])+translation;request.goal.pose.position.x,request.goal.pose.position.y=map(float,target);request.goal.pose.orientation.w=1.;request.planner_id='GridBased';request.use_start=False
        future=planner.send_goal_async(request);wait(future.done);handle=future.result()
        if not handle.accepted:raise RuntimeError('Planner rejected request')
        future=handle.get_result_async();wait(future.done);response=future.result();points=np.asarray([[p.pose.position.x,p.pose.position.y] for p in response.result.path.poses],dtype=float).reshape(-1,2);odom=(points-translation)@rotation
        # Save action/path/grid before any metric computation.
        np.savez_compressed(a.output/(phase+'.npz'),path=points,path_odom=odom,costmap=values,resolution=info.resolution,origin=[info.origin.position.x,info.origin.position.y],map_to_odom_translation=translation,map_to_odom_rotation=rotation,scan=np.asarray(scans[-1].ranges),scan_angles=[scans[-1].angle_min,scans[-1].angle_increment])
        raw={'status':int(response.status),'path_points':len(points),'sim':received['sim']};(a.output/(phase+'_action.json')).write_text(json.dumps(raw,indent=2)+'\n')
        if len(odom)<2:return raw
        dense=np.concatenate([np.linspace(x,y,max(2,int(np.linalg.norm(y-x)/.01)+1)) for x,y in zip(odom[:-1],odom[1:])]);middle=dense[(dense[:,0]>=4.0)&(dense[:,0]<=4.6)]
        yy,xx=np.indices(values.shape);map_xy=np.stack([info.origin.position.x+(xx+.5)*info.resolution,info.origin.position.y+(yy+.5)*info.resolution],axis=-1);world=(map_xy-translation)@rotation;b=config['barrier'];center=np.array(b['center'][:2]);half=np.array(b['size'][:2])/2;mask=(abs(world[...,0]-center[0])<half[0]+.15)&(abs(world[...,1]-center[1])<half[1]+.15)
        raw.update(divider_clearance_m=clearance(dense,config['divider']),barrier_clearance_m=clearance(dense,b),middle_y_min=float(middle[:,1].min()) if len(middle) else None,middle_y_max=float(middle[:,1].max()) if len(middle) else None,barrier_region_lethal_cells=int((values[mask]==100).sum()),length_m=float(np.linalg.norm(np.diff(odom,axis=0),axis=1).sum()))
        return raw
    try:
        wait(lambda:all(k in received for k in ['map','costmap','sim']) and positions and len(scans)>10)
        if not planner.wait_for_server(timeout_sec=10):raise RuntimeError('No native Navfn server')
        initial=list(received['map'].data);result['before_fixture']=fixture(False,'before');settle();result['before']=snapshot('before');result['enabled_fixture']=fixture(True,'enabled');settle();result['blocked']=snapshot('blocked')
        motion=float(np.max(np.linalg.norm(np.asarray(positions)-positions[0],axis=1)));result['max_displacement_m']=motion;b=result['before'];c=result['blocked']
        result['checks']={'both_native_plans_succeed':b['status']==c['status']==4,'prior_route_uses_upper_branch':b.get('middle_y_min',-99)>-1.1,'prior_route_intersects_future_barrier':b.get('barrier_clearance_m',99)==0.,'new_physical_barrier_marked':c.get('barrier_region_lethal_cells',0)-b.get('barrier_region_lethal_cells',0)>=20,'blocked_route_uses_lower_branch':c.get('middle_y_max',99)<-1.5,'new_path_clears_divider':c.get('divider_clearance_m',0)>=.55,'new_path_clears_barrier':c.get('barrier_clearance_m',0)>=.55,'map_files_unchanged':all(hashlib.sha256(p.read_bytes()).hexdigest()==hashes[str(p)] for p in files),'published_map_unchanged':list(received['map'].data)==initial,'no_commanded_motion':motion<.2,'one_map_source':[p.node_name for p in node.get_publishers_info_by_topic('/map')]==['map_server']}
    except Exception as exc:result['error']=str(exc)
    finally:
        (a.output/'positions.json').write_text(json.dumps(positions)+'\n');result['passed']=not result.get('error') and bool(result.get('checks')) and all(result['checks'].values());(a.output/'branch_plan_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True);node.destroy_node();rclpy.shutdown()
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
