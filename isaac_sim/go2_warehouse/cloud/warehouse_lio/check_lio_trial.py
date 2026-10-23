"""Read-only isolated trial observer. Truth is recorded for offline evaluation only."""
import argparse,json,time,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
DTYPE=np.dtype({'names':['x','y','z','intensity','time','ring'],'formats':['<f4','<f4','<f4','<f4','<f4','<u2'],'offsets':[0,4,8,12,16,20],'itemsize':24})
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9
class Observer(Node):
 def __init__(self,output):
  super().__init__('lio_frame20ms_observer');self.output=output;self.clock=None;self.start=None;self.truth=[];self.lio=[];self.imu=[];self.points=[]
  self.create_subscription(Clock,'/clock',self.on_clock,10)
  self.create_subscription(Imu,'/lio/imu',lambda m:self.imu.append(stamp(m)),qos_profile_sensor_data)
  self.create_subscription(Odometry,'/unitree_go2/odom',lambda m:self.odometry(m,self.truth),qos_profile_sensor_data)
  self.create_subscription(Odometry,'/lio/odometry',lambda m:self.odometry(m,self.lio),qos_profile_sensor_data)
  self.create_subscription(PointCloud2,'/lio/points_frame20ms',self.on_cloud,qos_profile_sensor_data)
 def on_clock(self,m):
  self.clock=m.clock.sec+m.clock.nanosec/1e9
  if self.start is None and self.clock>0:self.start=self.clock
 def odometry(self,m,rows):
  p=m.pose.pose.position;q=m.pose.pose.orientation;rows.append({'stamp':stamp(m),'xyz':[p.x,p.y,p.z],'xyzw':[q.x,q.y,q.z,q.w]})
 def on_cloud(self,m):
  fields={x.name:(x.offset,x.datatype) for x in m.fields};expected={n:(o,4 if n=='ring' else 7) for n,o in [('x',0),('y',4),('z',8),('intensity',12),('time',16),('ring',20)]}
  schema=fields==expected and m.point_step==24 and m.height==1 and not m.is_bigendian
  row={'stamp':stamp(m),'points':m.width,'schema':schema,'frame':m.header.frame_id,'data_bytes':len(m.data),'expected_bytes':m.width*24}
  if len(m.data)!=m.width*24:row['byte_length_valid']=False;self.points.append(row);return
  row['byte_length_valid']=True
  if schema:
   a=np.frombuffer(m.data,dtype=DTYPE,count=m.width);ts=np.unique(a['time']);row.update({'finite':bool(all(np.isfinite(a[k]).all() for k in ['x','y','z','time','intensity'])),'unique_offsets':ts.astype(float).tolist(),'five_capture_times':bool(len(ts)==5 and np.allclose(ts,[0,.02,.04,.06,.08],atol=1e-7)),'ring_range':[int(a['ring'].min()),int(a['ring'].max())]})
  self.points.append(row)
 def readiness(self):
  if self.clock and self.clock>=2 and len(self.imu)>=200 and len(self.points)>=10 and all(x.get('finite') and x.get('byte_length_valid') for x in self.points[-10:]) and not (self.output/'sensors_ready.json').exists():(self.output/'sensors_ready.json').write_text(json.dumps({'sim':self.clock,'readiness':'sensors only; no velocity authorization'})+'\n')
  if len(self.lio)>=10 and not (self.output/'lio_ready.json').exists():(self.output/'lio_ready.json').write_text(json.dumps({'sim':self.clock,'odometry_messages':len(self.lio),'scope':'isolated trial output ready; not accuracy approval'})+'\n')
p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--seconds',type=float,default=18);p.add_argument('--wall-timeout',type=float,default=240);p.add_argument('--motion-done',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
rclpy.init();n=Observer(a.output);wall=time.monotonic();error=None
try:
 while time.monotonic()-wall<a.wall_timeout:
  rclpy.spin_once(n,timeout_sec=.02);n.readiness()
  if n.start is not None and n.clock-n.start>=a.seconds:break
  if a.motion_done and a.motion_done.exists():
   done=json.loads(a.motion_done.read_text())
   if n.clock is not None and n.clock>=done['finished_sim']+5:break
 else:error='Observer wall timeout'
except Exception as exc:error=repr(exc)
finally:
 raw={'truth':n.truth,'lio':n.lio,'imu_stamps':n.imu,'points':n.points,'clock':n.clock,'error':error};(a.output/'telemetry.json').write_text(json.dumps(raw)+'\n')
 pubs={topic:[{'node':x.node_name,'type':x.topic_type} for x in n.get_publishers_info_by_topic(topic)] for topic in ['/tf','/lio/evaluation_tf','/lio/odometry','/lio/imu','/lio/points_frame20ms']}
 im=np.diff(n.imu);pt=np.diff([x['stamp'] for x in n.points]);li=np.diff([x['stamp'] for x in n.lio]);checks={'no_observer_error':error is None,'cloud_count':len(n.points)>=30,'serialized_length':bool(n.points and all(x.get('byte_length_valid') for x in n.points)),'cloud_rate':bool(len(pt) and abs(float(np.median(pt))-.1)<1e-5),'real_schema_and_finite':bool(n.points and all(x.get('schema') and x.get('finite') for x in n.points)),'five_quantized_capture_times':bool(n.points and all(x.get('five_capture_times') for x in n.points)),'native_rings':bool(n.points and all(x.get('ring_range')==[0,31] for x in n.points)),'imu_200hz_monotonic':bool(len(im)>200 and np.all(im>0) and abs(float(np.median(im))-.005)<1e-5),'lio_output':len(n.lio)>=20,'lio_time_monotonic':bool(len(li) and np.all(li>0)),'lio_pose_finite':bool(n.lio and all(np.isfinite(x['xyz']+x['xyzw']).all() for x in n.lio)),'one_lio_source':len(pubs['/lio/odometry'])==1,'one_input_cloud_source':len(pubs['/lio/points_frame20ms'])==1,'one_input_imu_source':len(pubs['/lio/imu'])==1,'tf_isolated':bool(pubs['/lio/evaluation_tf'] and not any('fastlio' in x['node'] for x in pubs['/tf']))}
 (a.output/'interfaces.json').write_text(json.dumps({'checks':checks,'passed':all(checks.values()),'publishers':pubs,'counts':{'imu':len(n.imu),'cloud':len(n.points),'lio':len(n.lio),'truth':len(n.truth)},'clock':n.clock,'error':error,'continuous_per_ray_timing_validated':False,'production_odom_switched':False},indent=2)+'\n');print((a.output/'interfaces.json').read_text());n.destroy_node();rclpy.shutdown()
