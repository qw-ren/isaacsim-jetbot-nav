"""Warm-start AMCL on a sensor-built saved map; bounded guarded motion, no Nav2 goal."""
import argparse,json,time,math,hashlib
from pathlib import Path
import numpy as np,yaml
from PIL import Image
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import ParticleCloud
from lifecycle_msgs.srv import GetState
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from check_motion import Probe
class LocalizationProbe(Probe):
 def __init__(self):
  super().__init__();self.poses=[];self.particles=[];self.grid=None;self.static_edges=[];self.map_tf=[]
  q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  self.map_sub=self.create_subscription(OccupancyGrid,'/map',self.map_cb,q);self.pose_sub=self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.pose_cb,qos_profile_sensor_data);self.particle_sub=self.create_subscription(ParticleCloud,'/particle_cloud',lambda m:self.particles.append(len(m.particles)),qos_profile_sensor_data);self.static_sub=self.create_subscription(TFMessage,'/tf_static',self.static_cb,q);self.tf_sub=self.create_subscription(TFMessage,'/tf',self.tf_cb,100)
 def map_cb(self,m):self.grid=m
 def pose_cb(self,m):
  p=m.pose.pose.position;q=m.pose.pose.orientation;self.poses.append({'wall':time.monotonic(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'frame':m.header.frame_id,'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'covariance_diag':[m.pose.covariance[i] for i in [0,7,35]]})
 def static_cb(self,m):
  for t in m.transforms:
   if t.header.frame_id=='map' and t.child_frame_id=='odom':self.static_edges.append('map->odom')
 def tf_cb(self,m):
  for t in m.transforms:
   if t.header.frame_id=='map' and t.child_frame_id=='odom':self.map_tf.append({'wall':time.monotonic(),'x':t.transform.translation.x,'y':t.transform.translation.y})
 def call(self,typ,name):
  c=self.create_client(typ,name)
  if not c.wait_for_service(timeout_sec=10):raise RuntimeError('Service unavailable '+name)
  f=c.call_async(typ.Request());until=time.monotonic()+10
  while not f.done() and time.monotonic()<until:self.send(0);rclpy.spin_once(self,timeout_sec=.05)
  if not f.done():raise RuntimeError('Service timeout '+name)
  return f.result()
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--map',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False);cfg=yaml.safe_load(a.map.read_text());pgm=a.map.parent/cfg['image'];before={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in [a.map,pgm]};gray=np.asarray(Image.open(pgm).convert('L'))[::-1];prob=(255-gray.astype(float))/255;expected=np.full(gray.shape,-1,dtype=np.int8);expected[prob>cfg['occupied_thresh']]=100;expected[prob<cfg['free_thresh']]=0
 rclpy.init();p=LocalizationProbe();r={'mode':'AMCL_warm_start_with_saved_SLAM_map_NOT_global_relocalization','nav2_goal_executed':False}
 try:
  until=time.monotonic()+60
  while time.monotonic()<until and not(p.rows and p.runtime and p.states and p.grid is not None):p.send(0);rclpy.spin_once(p,timeout_sec=.05)
  r['lifecycle']={n:int(p.call(GetState,f'/{n}/get_state').current_state.id) for n in ['map_server','amcl']}
  p.call(Empty,'/request_nomotion_update');until=time.monotonic()+15
  while time.monotonic()<until and not(p.poses and p.particles):p.send(0);rclpy.spin_once(p,timeout_sec=.05)
  if not p.poses:raise RuntimeError('No AMCL pose')
  p.phase('localized_ready',1);r['pose_start']=p.poses[-1];r['odom_start']=p.rows[-1];before_count=len(p.poses)
  if not p.arm(True):raise RuntimeError('Guard enable refused')
  p.phase('localization_forward',1.8,.8,check_gate=True);r['odom_end']=dict(p.rows[-1]);r['pose_motion_updates']=len(p.poses)-before_count;p.phase('localization_zero',2,0,check_gate=False);p.arm(False);p.call(Empty,'/request_nomotion_update');p.phase('localization_observe',1,0)
  r['pose_end']=p.poses[-1];r['odom_displacement_m']=math.hypot(r['odom_end']['x']-r['odom_start']['x'],r['odom_end']['y']-r['odom_start']['y']);r['amcl_displacement_m']=math.hypot(r['pose_end']['x']-r['pose_start']['x'],r['pose_end']['y']-r['pose_start']['y']);rows=[x for x in p.rows if x['stage']=='localization_forward' and x['sim']>=r['odom_end']['sim']-.7];r['cruise_mps']=float(np.mean([x['vx'] for x in rows]))
 except Exception as e:r['error']=str(e)
 finally:
  p.arm(False)
  for _ in range(10):p.send(0);rclpy.spin_once(p,timeout_sec=.02)
  r['map_publishers']=[x.node_name for x in p.get_publishers_info_by_topic('/map')];r['tf_publishers']=[x.node_name for x in p.get_publishers_info_by_topic('/tf')];r['guard_faults']=sorted({x['reason'] for x in p.states if x['reason'] not in ['DISARMED','ENABLED']});actual=np.asarray(p.grid.data,dtype=np.int8).reshape(p.grid.info.height,p.grid.info.width) if p.grid else None
  r['particle_counts']=p.particles[-10:];r['checks']={'no_error':'error' not in r,'lifecycle_active':r.get('lifecycle')=={'map_server':3,'amcl':3},'loaded_grid_matches_file':actual is not None and np.array_equal(actual,expected),'map_origin_matches_file':p.grid is not None and max(abs(p.grid.info.origin.position.x-cfg['origin'][0]),abs(p.grid.info.origin.position.y-cfg['origin'][1]))<1e-6,'unknown_preserved':actual is not None and int((actual==-1).sum())>0,'files_unchanged':all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==h for f,h in before.items()),'one_saved_map_source':r['map_publishers']==['map_server'],'amcl_dynamic_map_tf':bool(p.map_tf) and 'amcl' in r['tf_publishers'] and 'slam_toolbox' not in r['tf_publishers'],'no_static_truth_map_tf':not p.static_edges,'amcl_poses_finite':bool(p.poses) and all(x['frame']=='map' and np.isfinite([x['x'],x['y'],x['yaw'],*x['covariance_diag']]).all() for x in p.poses),'particles_present':bool(p.particles) and max(p.particles)>=500,'guarded_motion':r.get('cruise_mps',0)>=.6 and r.get('odom_displacement_m',0)>.5,'pose_updates_during_motion':r.get('pose_motion_updates',0)>=2,'amcl_tracks_motion':r.get('amcl_displacement_m',0)>.5 and abs(r.get('amcl_displacement_m',99)-r.get('odom_displacement_m',0))<.4,'no_guard_fault':not r['guard_faults']}
  r['passed']=all(r['checks'].values());(a.output/'localization_check.json').write_text(json.dumps(r,indent=2)+'\n');(a.output/'telemetry.json').write_text(json.dumps({'odom':p.rows,'guard':p.states,'runtime':p.runtime,'amcl_poses':p.poses,'particle_counts':p.particles,'map_tf':p.map_tf},indent=2)+'\n');print(json.dumps(r,indent=2));p.destroy_node();rclpy.shutdown()
 return 0 if r['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
