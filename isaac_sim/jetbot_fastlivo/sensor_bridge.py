"""Native IMU/RGB/RTX adapter. Truth exists only in isolated evaluation output."""
import json,math,time
from array import array
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist,TransformStamped
from sensor_msgs.msg import Imu,Image,CameraInfo,PointCloud2,PointField,LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from pxr import UsdGeom,Usd
import omni.usd
import omni.timeline
import omni.replicator.core as rep
import omni.graph.core as og
from isaacsim.sensors.camera import Camera
from isaacsim.sensors.physics import _sensor
from isaacsim.core.nodes.bindings import _isaacsim_core_nodes

DTYPE=np.dtype({'names':['x','y','z','intensity','time','ring'],'formats':['<f4']*5+['<u2'],'offsets':[0,4,8,12,16,20],'itemsize':24})
CAMERA='/World/jetbot/chassis/rgb_camera/jetbot_camera'
LIDAR='/World/jetbot/chassis/lidar/XT_32_10hz/PandarXT_32_10hz'
IMU='/World/jetbot/chassis/imu/Imu_Sensor'
def stamp(value):
 from builtin_interfaces.msg import Time
 n=round(value*1e9);return Time(sec=n//10**9,nanosec=n%10**9)

class SensorBridge:
 def __init__(self,world,robot,root):
  self.world,self.robot,self.root=world,robot,Path(root)
  timing_audit=json.loads((self.root/'CASE.json').read_text()).get('navigation',{}).get('timing_audit',False);self.timing_stream=(self.root/'native_time_audit.jsonl').open('x')if timing_audit else None
  self.point_time_mode=json.loads((self.root/'CASE.json').read_text()).get('navigation',{}).get('point_time_mode','packet_capture');assert self.point_time_mode in('packet_capture','native_frame_rays');self.native_frame_offset=None
  self.motion_limits=json.loads((self.root/'CASE.json').read_text()).get('motion_limits',{});self.max_linear=float(self.motion_limits.get('max_linear_m_s',.3));self.max_angular=float(self.motion_limits.get('max_angular_rad_s',.8));self.wheel_max=float(self.motion_limits.get('max_wheel_rad_s',1e20));self.wheel_radius=float(self.motion_limits.get('wheel_radius_m',.03));self.wheel_base=float(self.motion_limits.get('wheel_base_m',.1125))
  rclpy.init();self.node=Node('jetbot_native_bridge',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.clock=self.node.create_publisher(Clock,'/clock',10)
  self.imu_pub=self.node.create_publisher(Imu,'/jetbot/imu',200)
  self.scan_pub=self.node.create_publisher(LaserScan,'/jetbot/scan',10)
  self.cloud_pub=self.node.create_publisher(PointCloud2,'/jetbot/livo/points',5)
  self.image_pub=self.node.create_publisher(Image,'/jetbot/image_raw',10)
  self.info_pub=self.node.create_publisher(CameraInfo,'/jetbot/camera_info',10)
  self.depth_enabled=json.loads((self.root/'CASE.json').read_text()).get('navigation',{}).get('extensions',{}).get('depth',False);assert type(self.depth_enabled)is bool
  self.depth_pub=self.node.create_publisher(Image,'/jetbot/depth/image_rect_raw',5)if self.depth_enabled else None
  self.truth=self.node.create_publisher(Odometry,'/truth/jetbot_odom',10)
  self.sub=self.node.create_subscription(Twist,'/jetbot/cmd_vel',self.receive,1)
  self.tf=StaticTransformBroadcaster(self.node)
  self.last_command=None;self.last_wall=None;self.last_sim=None;self.command_fault=None
  self.scan_deskew=json.loads((self.root/'CASE.json').read_text()).get('navigation',{}).get('scan_deskew_gyro',False);assert type(self.scan_deskew)is bool;assert not self.scan_deskew or self.point_time_mode=='native_frame_rays';self.gyro_history=deque(maxlen=1000);self.previous_gyro=None;self.integrated_yaw=0.
  self.robot_scale=float(json.loads((self.root/'CASE.json').read_text()).get('robot_variant',{}).get('scale',1.));self.self_boxes=[]
  self.counts=dict(imu=0,points=0,images=0,packet_discontinuities=0,commands=0)
  self.interface=_sensor.acquire_imu_sensor_interface();self.core=_isaacsim_core_nodes.acquire_interface()
  self.world.add_physics_callback('jetbot_native_imu',lambda dt:self.sample_imu())
  self.camera=Camera(prim_path=CAMERA,frequency=10,resolution=(320,240));self.camera.initialize()
  if self.depth_enabled:self.camera.add_distance_to_image_plane_to_frame()
  product=rep.create.render_product(LIDAR,(1,1));self.product=product
  self.raw=rep.AnnotatorRegistry.get_annotator('RtxSensorCpuIsaacReadRTXLidarData');self.raw.initialize(keepOnlyPositiveDistance=False);self.raw.attach(product.path)
  self.time_reader=None
  if self.timing_stream is not None or self.point_time_mode=='native_frame_rays':
   self.time_reader=rep.AnnotatorRegistry.get_annotator('IsaacReadTimes');self.time_reader.attach(product.path)
  self.last_imu=-1.;self.last_rgb=-1.;self.last_render=-1;self.last_capture=None;self.last_native=None;self.group=[];self.contract_ready=False;self.next_progress=0.;self.raw_debug_saved=False;self.raw_state={};self.packet_log=(self.root/'packet_timing.jsonl').open('x')
  self.stream=(self.root/'sensor_audit.jsonl').open('x');self.controls=(self.root/'control_audit.jsonl').open('x');self.truth_log=(self.root/'truth_trajectory.jsonl').open('x')
  stage=omni.usd.get_context().get_stage();cache=UsdGeom.XformCache();base=cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/jetbot/chassis'))
  self.mounts={};transforms=[]
  for frame,path,optical in [('jetbot/imu',IMU,False),('jetbot/lidar',LIDAR,False),('jetbot/camera_optical',CAMERA,True)]:
   m=np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))*base.GetInverse())
   raw=m[:3,:3].T;u,scale,v=np.linalg.svd(raw);rotation=u@v
   if optical:rotation=rotation@np.diag([1.,-1.,-1.])
   position=m[3,:3]*float(json.loads((self.root/'CASE.json').read_text()).get('robot_variant',{}).get('scale',1.));self.mounts[frame]=dict(R=rotation.tolist(),T=position.tolist(),authored_scale=scale.tolist())
   tr=TransformStamped();tr.header.frame_id='chassis';tr.child_frame_id=frame
   tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=map(float,position)
   q=Rotation.from_matrix(rotation).as_quat();tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=map(float,q);transforms.append(tr)
  if self.robot_scale!=1.:
   bbox_cache=UsdGeom.BBoxCache(Usd.TimeCode.Default(),[UsdGeom.Tokens.default_,UsdGeom.Tokens.render])
   for prim in Usd.PrimRange(stage.GetPrimAtPath('/World/jetbot')):
    if prim.GetTypeName()not in('Mesh','Cube','Sphere','Cylinder','Capsule'):continue
    bounds=bbox_cache.ComputeWorldBound(prim);bounds.Transform(base.GetInverse());bounds=bounds.ComputeAlignedRange()
    if bounds.IsEmpty():continue
    lo=np.asarray(bounds.GetMin())*self.robot_scale;hi=np.asarray(bounds.GetMax())*self.robot_scale
    if np.isfinite(np.r_[lo,hi]).all():self.self_boxes.append(dict(path=str(prim.GetPath()),min=lo.tolist(),max=hi.tolist()))
   assert self.self_boxes,'Scaled robot self geometry is missing'
   (self.root/'LIDAR_SELF_FILTER.json').write_text(json.dumps(dict(scope='own USD mesh AABBs in chassis meters only; conservative mesh volumes, no environment/ground masking, no truth pose input',padding_m=.005,boxes=self.self_boxes),indent=2)+'\n')
  self.tf.sendTransform(transforms)
  (self.root/'SENSOR_CONTRACT.json').write_text(json.dumps(dict(mounts=self.mounts,imu_orientation_valid=False,native_imu_no_added_noise=True,depth_enabled=self.depth_enabled,depth_contract="32FC1 meters along optical Z, same RGB render product/header/intrinsics, NaN invalid",truth_topics=['/truth/jetbot_odom'],navigation_truth_input=False,tf_parent='chassis',dynamic_tf_owner='external FAST-LIVO adapter only',point_timing=('same-render-product frame clock plus native per-ray deltaTimes; moving validation case37'if self.point_time_mode=='native_frame_rays'else 'five measured render packets; capture offsets only, native per-ray timing unverified'),scan_timing=('native acquisition midpoint, raw IMU gyro yaw deskew only; translation uncompensated'if self.scan_deskew else 'legacy packet-capture header; raw scan motion deskew not implemented'),command_topic='/jetbot/cmd_vel',command_caps=[self.max_linear,self.max_angular],joint_limits_modified=False,command_timeout_wall_s=.35,command_timeout_sim_s=.35,simulation_only=True),indent=2)+'\n')
 def receive(self,m):
  values=np.array([m.linear.x,m.angular.z],float)
  if not np.isfinite(values).all():self.command_fault='NONFINITE_COMMAND';return
  publishers=self.node.get_publishers_info_by_topic('/jetbot/cmd_vel')
  if len(publishers)>1:self.command_fault='MULTIPLE_COMMAND_PUBLISHERS';return
  w=float(np.clip(values[1],-self.max_angular,self.max_angular));linear_cap=min(self.max_linear,max(0.,self.wheel_max*self.wheel_radius-abs(w)*self.wheel_base/2));self.last_command=[float(np.clip(values[0],-linear_cap,linear_cap)),w]
  self.last_wall=time.monotonic();self.last_sim=float(self.world.current_time);self.counts['commands']+=1
 def spin(self):
  # Drain clock and command callbacks without waiting; one callback per rendered frame can starve commands when RGB-D rendering is slow.
  for _ in range(8):rclpy.spin_once(self.node,timeout_sec=0.)
 def command(self):
  safe=self.last_wall is not None and self.command_fault is None and time.monotonic()-self.last_wall<=.35 and self.world.current_time-self.last_sim<=.35
  values=self.last_command if safe else [0.,0.]
  if self.last_wall is not None and not safe and any(abs(v)>1e-6 for v in self.last_command):self.command_fault=self.command_fault or 'COMMAND_TIMEOUT'
  # No automatic motion restart after a fault; a new runtime is required.
  self.controls.write(json.dumps(dict(sim=float(self.world.current_time),wall=time.monotonic(),output=values,fault=self.command_fault,command_wall_age_s=(None if self.last_wall is None else time.monotonic()-self.last_wall)))+'\n')
  return values
 def tick(self):
  now=float(self.world.current_time);clock=Clock();clock.clock=stamp(now);self.clock.publish(clock)
  self.sample_imu()
  frame=self.camera.get_current_frame();rgba=np.asarray(frame.get('rgba',[]));ts=float(frame.get('rendering_time',-1))
  if rgba.ndim==3 and ts>self.last_rgb and ts>0:
   if not self.contract_ready:self.camera_contract(rgba.shape[1],rgba.shape[0]);self.contract_ready=True
   if now-ts>.15 or ts-now>.021:raise RuntimeError('Camera acquisition time mismatch')
   self.last_rgb=ts;rgb=np.ascontiguousarray(rgba[:,:,:3],dtype=np.uint8)
   m=Image();m.header.stamp=stamp(ts);m.header.frame_id='jetbot/camera_optical';m.height,m.width=rgb.shape[:2];m.encoding='rgb8';m.step=m.width*3;m.data=rgb.tobytes();self.image_pub.publish(m)
   info=CameraInfo();info.header=m.header;info.height=m.height;info.width=m.width;info.k=self.k.ravel().tolist();info.r=np.eye(3).ravel().tolist();info.p=np.c_[self.k,np.zeros(3)].ravel().tolist();info.distortion_model='plumb_bob';info.d=[0.]*5;self.info_pub.publish(info);self.counts['images']+=1
   if self.depth_enabled:
    depth=np.asarray(frame.get('distance_to_image_plane',[]),dtype='<f4').squeeze()
    if depth.shape!=rgb.shape[:2]:self.counts['depth_not_ready']=self.counts.get('depth_not_ready',0)+1
    else:
     depth=np.ascontiguousarray(np.where(np.isfinite(depth)&(depth>0),depth,np.nan),dtype='<f4');dm=Image();dm.header=m.header;dm.height,dm.width=depth.shape;dm.encoding='32FC1';dm.is_bigendian=0;dm.step=dm.width*4;dm.data=depth.tobytes();self.depth_pub.publish(dm);self.counts['depth']=self.counts.get('depth',0)+1
     if self.counts['depth']in[1,30,100]:np.savez_compressed(self.root/('depth_%04d.npz'%self.counts['depth']),depth_m=depth,stamp_s=ts,K=self.k)
   if self.counts['images'] in [1,30,100]:
    from PIL import Image as Pillow
    Pillow.fromarray(rgb).save(self.root/f'rgb_{self.counts["images"]:04d}.png')
  self.lidar_tick(now)
  position,orientation=self.robot.get_world_pose();self.truth_log.write(json.dumps(dict(stamp=now,xyz=position.tolist(),wxyz=orientation.tolist()))+'\n');m=Odometry();m.header.stamp=stamp(now);m.header.frame_id='truth_world';m.child_frame_id='truth_jetbot'
  m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,position);m.pose.pose.orientation.w,m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z=map(float,orientation);self.truth.publish(m)
  if now>=self.next_progress:
   self.next_progress=now+.5
   self.stream.flush();self.controls.flush();(self.root/'PROGRESS.json').write_text(json.dumps(dict(sim_s=now,counts=self.counts,raw_lidar=self.raw_state,command_fault=self.command_fault,truth_for_evaluation_only=position.tolist()))+'\n')
 def sample_imu(self):
  reading=self.interface.get_sensor_reading(IMU,use_latest_data=True,read_gravity=True)
  if reading.is_valid and float(reading.time)>self.last_imu:
   if self.previous_gyro is not None:
    old_t,old_z=self.previous_gyro;self.integrated_yaw+=(float(reading.time)-old_t)*(old_z+float(reading.ang_vel_z))/2
   self.previous_gyro=(float(reading.time),float(reading.ang_vel_z));self.gyro_history.append((float(reading.time),self.integrated_yaw))
   self.last_imu=float(reading.time);m=Imu();m.header.stamp=stamp(self.last_imu);m.header.frame_id='jetbot/imu';m.orientation.w=1.;m.orientation_covariance[0]=-1.
   m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,[reading.lin_acc_x,reading.lin_acc_y,reading.lin_acc_z])
   m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,[reading.ang_vel_x,reading.ang_vel_y,reading.ang_vel_z]);self.imu_pub.publish(m);self.counts['imu']+=1
 def camera_contract(self,width,height):
  self.k=np.asarray(self.camera.get_intrinsics_matrix());mount=self.mounts['jetbot/camera_optical'];imu=self.mounts['jetbot/imu'];lid=self.mounts['jetbot/lidar'];rc=np.array(mount['R']);rl=np.array(lid['R']);ri=np.array(imu['R'])
  c=dict(width=width,height=height,k=self.k.tolist(),Rcl=(rc.T@rl).tolist(),Pcl=(rc.T@(np.array(lid['T'])-mount['T'])).tolist(),extrinsic_R=(ri.T@rl).ravel().tolist(),extrinsic_T=(ri.T@(np.array(lid['T'])-imu['T'])).tolist(),mounts=self.mounts,camera_axes='ROS optical +Z forward +X right +Y down; actual USD pose rotated 180 degrees about local X',native_pinhole=True)
  (self.root/'camera_contract.json').write_text(json.dumps(c,indent=2)+'\n')
 def lidar_tick(self,now):
  d=self.raw.get_data()
  if d and now>1. and not self.raw_debug_saved:
   arrays={}
   for k,v in d.items():
    try:
     value=np.asarray(v)
     if value.dtype.kind in 'biuf' and value.size:arrays[k]=value.copy()
    except (ValueError,TypeError):pass
   np.savez_compressed(self.root/'raw_lidar_first.npz',**arrays)
   (self.root/'RAW_LIDAR_KEYS.json').write_text(json.dumps({k:dict(shape=list(np.asarray(v).shape),dtype=str(np.asarray(v).dtype))for k,v in d.items()},indent=2)+'\n')
   self.raw_debug_saved=True
  self.raw_state=dict(keys=list(d) if isinstance(d,dict) else str(type(d)),frame=int(d.get('frameId',-1)) if isinstance(d,dict) else -1,timestamp=int(d.get('timestampNs',-1)) if isinstance(d,dict) else -1)
  if not d or 'frameId'not in d or int(d.get('timestampNs',0))<=0:return
  fid=int(d['frameId'])
  if fid==self.last_render:return
  self.last_render=fid
  node=og.Controller().node('/Render/PostProcess/SDGPipeline/PostProcessDispatcher');reference=int(node.get_attribute('outputs:referenceTimeNumerator').get());capture=float(self.core.get_sim_time_at_swh_frame(reference));native=int(d['timestampNs'])/1e9
  if abs(capture-now)>.0051:raise RuntimeError('Unmatched RTX capture time')
  if self.timing_stream is not None:
   # Reference pose is logged ONLY for offline frame-time validation, never used by point timestamps or control.
   p,q=self.robot.get_world_pose();delta=np.asarray(d['deltaTimes']);clock_probes={}
   for frame in [fid,reference,reference-1,reference-2,reference-3,reference-4]:
    try:clock_probes[str(frame)]=float(self.core.get_sim_time_at_swh_frame(frame))
    except Exception:clock_probes[str(frame)]=None
   render_times={}
   if self.time_reader is not None:
    for key,value in (self.time_reader.get_data()or{}).items():
     field_value=np.asarray(value)
     if field_value.size==1 and field_value.dtype.kind in 'iuf':render_times[key]=field_value.item()
   row=dict(render_times=render_times,timeline_now=float(omni.timeline.get_timeline_interface().get_current_time()),monotonic_now=float(self.core.get_sim_time_monotonic_at_swh_frame(0)),system_now=float(self.core.get_system_time_at_swh_frame(0)),frame_id=fid,reference_swh=reference,clock_probes=clock_probes,capture=capture,now=now,native=native,delta_min_ns=int(delta.min()),delta_max_ns=int(delta.max()),transform=np.asarray(d['transform']).reshape(4,4).tolist(),transform_start=np.asarray(d['transformStart']).reshape(4,4).tolist(),reference_xyz=p.tolist(),reference_wxyz=q.tolist());self.timing_stream.write(json.dumps(row)+'\n');self.timing_stream.flush()
  dt=None if self.last_capture is None else capture-self.last_capture
  ndt=None if self.last_native is None else native-self.last_native
  self.packet_log.write(json.dumps(dict(frame=fid,capture=capture,native=native,dt=dt,native_dt=ndt))+'\n')
  if dt is not None and not 0<dt<=.041:self.group=[];self.counts['packet_discontinuities']+=1
  self.last_capture,self.last_native=capture,native
  distance=np.asarray(d['distances']);az=np.deg2rad(d['azimuths']);el=np.deg2rad(d['elevations']);rings=np.asarray(d['channels']);intensity=np.asarray(d['intensities']);valid=np.isfinite(distance)&(distance>.15)&(distance<40)&np.isfinite(az)&np.isfinite(el)&np.isfinite(intensity)&(rings<32)
  if self.robot_scale!=1.:
   mount=self.mounts['jetbot/lidar'];xyz=np.c_[distance*np.cos(el)*np.cos(az),distance*np.cos(el)*np.sin(az),distance*np.sin(el)]@np.asarray(mount['R']).T+np.asarray(mount['T']);rejected=np.zeros(len(distance),dtype=bool)
   for box in self.self_boxes:rejected|=np.all((xyz>=np.asarray(box['min'])-.005)&(xyz<=np.asarray(box['max'])+.005),axis=1)
   rejected&=valid;self.counts['self_returns_filtered']=self.counts.get('self_returns_filtered',0)+int(rejected.sum());valid&=~rejected
  dist,az,el=distance[valid],az[valid],el[valid];points=np.zeros(len(dist),dtype=DTYPE);points['x']=dist*np.cos(el)*np.cos(az);points['y']=dist*np.cos(el)*np.sin(az);points['z']=dist*np.sin(el);points['ring']=rings[valid];points['intensity']=intensity[valid]
  packet_start,packet_end=capture,capture
  if self.point_time_mode=='native_frame_rays':
   from native_lidar_timing import packet_timing
   timing=packet_timing(fid,self.time_reader.get_data(),d['deltaTimes'],now,self.native_frame_offset)
   if timing is None:self.group=[];self.counts['packet_discontinuities']+=1;return
   if self.native_frame_offset is None:
    self.native_frame_offset=timing['frame_offset'];(self.root/'NATIVE_TIMING_RUNTIME.json').write_text(json.dumps(dict(frame_number_offset=self.native_frame_offset,clock='same render product IsaacReadTimes.simulationTime',ray_times='native deltaTimes ns from frame start',validation_case='timing_20261025_37',reference_pose_used=False),indent=2)+'\n')
   packet_start,packet_end=timing['start'],timing['end'];points['time']=timing['ray_offsets'][valid]
  self.group.append((packet_start,fid,points,capture,packet_end))
  if len(self.group)<5:return
  group=self.group;self.group=[];begin=group[0][0]
  if not 0<group[-1][4]-begin<=.165:raise RuntimeError('Non-contiguous lidar packets')
  for ts,_,points,_,_ in group:
   if self.point_time_mode=='native_frame_rays':points['time']+=ts-begin
   else:points['time']=ts-begin
  points=np.zeros(sum(len(x[2])for x in group),dtype=DTYPE);cursor=0
  for _,_,part,_,_ in group:points[cursor:cursor+len(part)]=part;cursor+=len(part)
  if points.dtype.itemsize!=24 or points.nbytes!=len(points)*24:raise RuntimeError('Invalid PointXYZIRT stride')
  m=PointCloud2();m.header.stamp=stamp(begin);m.header.frame_id='jetbot/lidar';m.height=1;m.width=len(points);m.point_step=24;m.row_step=24*len(points);m.is_dense=True
  m.fields=[PointField(name=n,offset=o,datatype=PointField.UINT16 if n=='ring'else PointField.FLOAT32,count=1)for n,o in [('x',0),('y',4),('z',8),('intensity',12),('time',16),('ring',20)]];m.data=array('B',points.tobytes())
  if len(m.data)!=m.row_step*m.height:raise RuntimeError('Cloud payload length does not match declared stride')
  self.cloud_pub.publish(m);self.publish_scan(points,stamp(group[0][3]),group[-1][3]-group[0][3]+.02,point_start=begin);self.counts['points']+=1
  self.stream.write(json.dumps(dict(stamp=begin,end=group[-1][4],point_time_mode=self.point_time_mode,frame_ids=[x[1]for x in group],points=len(points),ring_range=[int(points['ring'].min()),int(points['ring'].max())]if len(points)else [],native_timestamp=native,capture=capture,per_ray_times_verified=(self.point_time_mode=='native_frame_rays'),time_validation='SDK offsets and moving frame endpoint validation; no per-ray hit-pose truth input'))+'\n')
  if self.counts['points'] in [1,10,100]:np.savez_compressed(self.root/f'points_{self.counts["points"]:04d}.npz',points=points,stamp=begin)
 def publish_scan(self,points,measurement_stamp,scan_time,point_start=None):
  mount=self.mounts['jetbot/lidar'];xyz=np.column_stack([points[k]for k in ['x','y','z']])@np.array(mount['R']).T+np.array(mount['T'])
  if getattr(self,'scan_deskew',False):
   from scan_yaw_deskew import deskew_yaw
   try:xyz,reference=deskew_yaw(xyz,float(point_start)+points['time'].astype(np.float64),self.gyro_history)
   except ValueError:self.counts['scan_deskew_skips']=self.counts.get('scan_deskew_skips',0)+1;return
   measurement_stamp=stamp(reference);self.counts['scan_deskew_frames']=self.counts.get('scan_deskew_frames',0)+1
  distance=np.linalg.norm(xyz[:,:2],axis=1);valid=(xyz[:,2]>=.03)&(xyz[:,2]<=.40)&(distance>=.15)&(distance<=20.)
  # No eligible height-band return means unknown, never a confirmed free ray.
  angles=np.arctan2(xyz[valid,1],xyz[valid,0]);step=math.pi/720.;bins=np.rint((angles+math.pi)/step).astype(int)%1440;ranges=np.full(1440,np.nan);np.fmin.at(ranges,bins,distance[valid])
  m=LaserScan();m.header.stamp=measurement_stamp;m.header.frame_id='chassis';m.angle_min=-math.pi;m.angle_increment=step;m.angle_max=-math.pi+1439*step;m.range_min=.15;m.range_max=20.;m.scan_time=float(scan_time);m.time_increment=0.;m.ranges=ranges.tolist();self.scan_pub.publish(m)
  if self.counts['points']in [0,9,99]:np.savez_compressed(self.root/f'scan_{self.counts["points"]+1:04d}.npz',ranges=ranges,stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)
 def close(self):
  if self.timing_stream is not None:self.timing_stream.close()
  self.stream.close();self.controls.close();self.packet_log.close();self.truth_log.close();(self.root/'SENSOR_RESULT.json').write_text(json.dumps(dict(counts=self.counts,command_fault=self.command_fault,last_imu=self.last_imu,last_rgb=self.last_rgb,camera_contract_ready=self.contract_ready),indent=2)+'\n');self.node.destroy_node();rclpy.shutdown()
