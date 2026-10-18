"""Bounded guarded teleoperation for a B prior with a physically present fixed divider, not Nav2."""
import argparse,json,time,math,hashlib
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from slam_toolbox.srv import SaveMap,SerializePoseGraph
from rcl_interfaces.srv import GetParameters
from check_motion import Probe
class MappingProbe(Probe):
 def __init__(self):
  super().__init__();self.maps=[];self.latest_map=None;self.map_tf=[];self.static_map_edges=[]
  q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  self.map_sub=self.create_subscription(OccupancyGrid,'/map',self.on_map,q)
  self.tf_sub=self.create_subscription(TFMessage,'/tf',self.on_tf,100)
  self.static_sub=self.create_subscription(TFMessage,'/tf_static',self.on_static,q)
 def on_map(self,m):
  self.latest_map=m;d=np.asarray(m.data);self.maps.append({'wall':time.monotonic(),'sim':self.sim,'width':m.info.width,'height':m.info.height,'occupied':int((d>=65).sum()),'free':int((d==0).sum()),'unknown':int((d==-1).sum())})
 def on_tf(self,m):
  for t in m.transforms:
   if t.header.frame_id=='map' and t.child_frame_id=='odom':
    p=t.transform.translation;q=t.transform.rotation;self.map_tf.append({'wall':time.monotonic(),'x':p.x,'y':p.y,'z':p.z,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))})
 def on_static(self,m):
  for t in m.transforms:
   if t.header.frame_id=='map' and t.child_frame_id=='odom':self.static_map_edges.append('map->odom')
 def call(self,typ,name,request):
  c=self.create_client(typ,name)
  if not c.wait_for_service(timeout_sec=10):raise RuntimeError('Service unavailable: '+name)
  f=c.call_async(request);until=time.monotonic()+30
  while not f.done() and time.monotonic()<until:self.send(0);rclpy.spin_once(self,timeout_sec=.05)
  if not f.done():raise RuntimeError('Service timeout: '+name)
  return f.result()
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--map-dir',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False);a.map_dir.mkdir(parents=True,exist_ok=False)
 rclpy.init();p=MappingProbe();r={'mode':'SLAM_sensor_built_B_prior_divider_NOT_full_warehouse','truth_odometry_input':True,'nav2_goal_executed':False}
 try:
  until=time.monotonic()+110
  while time.monotonic()<until and not(p.maps and p.map_tf and p.rows and p.runtime and p.states):p.send(0);rclpy.spin_once(p,timeout_sec=.05)
  if not p.maps or not p.map_tf:raise RuntimeError('SLAM map/TF not ready')
  req=GetParameters.Request();req.names=['use_sim_time','base_frame','mode'];pars=p.call(GetParameters,'/slam_toolbox/get_parameters',req);r['parameters']={'use_sim_time':pars.values[0].bool_value,'base_frame':pars.values[1].string_value,'mode':pars.values[2].string_value}
  p.phase('mapping_ready',2);r['start']=dict(p.rows[-1])
  if not p.arm(True):raise RuntimeError('Guard enable refused')
  p.phase('mapping_forward',6.5,.8,check_gate=True);r['forward_end']=dict(p.rows[-1]);p.phase('mapping_zero',3,0,check_gate=False);r['end']=dict(p.rows[-1]);p.arm(False)
  req=SaveMap.Request();req.name.data=str(a.map_dir/'map');r['save_map_result']=int(p.call(SaveMap,'/slam_toolbox/save_map',req).result)
  req=SerializePoseGraph.Request();req.filename=str(a.map_dir/'posegraph');r['serialize_result']=int(p.call(SerializePoseGraph,'/slam_toolbox/serialize_map',req).result)
  active=[x for x in p.rows if x['stage']=='mapping_forward' and x['sim']>=r['forward_end']['sim']-1];r['cruise_mean_mps']=float(np.mean([x['vx'] for x in active]));r['travel_m']=math.hypot(r['forward_end']['x']-r['start']['x'],r['forward_end']['y']-r['start']['y'])
 except Exception as exc:r['error']=str(exc)
 finally:
  p.arm(False)
  for _ in range(10):p.send(0);rclpy.spin_once(p,timeout_sec=.02)
  m=p.latest_map
  if m is not None:
   np.savez_compressed(a.output/'slam_grid.npz',grid=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width),resolution=m.info.resolution,origin=[m.info.origin.position.x,m.info.origin.position.y,m.info.origin.position.z]);r['last_map']=p.maps[-1]
  r['map_publishers']=[x.node_name for x in p.get_publishers_info_by_topic('/map')];r['tf_publishers']=[x.node_name for x in p.get_publishers_info_by_topic('/tf')];r['map_to_odom']=p.map_tf[-1] if p.map_tf else None;r['guard_faults']=sorted({x['reason'] for x in p.states if x['reason'] not in ['DISARMED','ENABLED']})
  r['checks']={'no_error':'error' not in r,'map_messages':len(p.maps)>=2,'nonempty_occupancy':m is not None and p.maps[-1]['occupied']>=100 and p.maps[-1]['free']>=100,'unknown_preserved':m is not None and p.maps[-1]['unknown']>0,'guarded_cruise':r.get('cruise_mean_mps',0)>=.6,'corridor_travel':r.get('travel_m',0)>=3.,'no_guard_fault':not r['guard_faults'],'one_slam_map_source':r['map_publishers']==['slam_toolbox'],'no_truth_map_static_tf':not p.static_map_edges,'slam_dynamic_map_tf':bool(p.map_tf) and 'slam_toolbox' in r['tf_publishers'],'sim_mapping_parameters':r.get('parameters',{}).get('use_sim_time',False) and r.get('parameters',{}).get('mode')=='mapping','saved_grid':r.get('save_map_result')==0 and (a.map_dir/'map.pgm').is_file() and (a.map_dir/'map.yaml').is_file(),'saved_graph':r.get('serialize_result')==0 and bool(list(a.map_dir.glob('*.posegraph')))}
  r['passed']=all(r['checks'].values());(a.output/'slam_check.json').write_text(json.dumps(r,indent=2)+'\n');(a.output/'telemetry.json').write_text(json.dumps({'odom':p.rows,'guard':p.states,'runtime':p.runtime,'map_updates':p.maps,'map_tf':p.map_tf},indent=2)+'\n');(a.map_dir/'map.sha256').write_text(''.join(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+f.name+'\n' for f in sorted(a.map_dir.glob('*')) if f.is_file() and f.name!='map.sha256'));print(json.dumps(r,indent=2));p.destroy_node();rclpy.shutdown()
 return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
