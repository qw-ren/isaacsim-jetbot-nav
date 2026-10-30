"""JetBot IMU lever arm and unique estimated odom/TF. No truth subscription."""
import argparse,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def seconds(s):return s.sec+s.nanosec*1e-9
class EstimatedOdometry(Node):
 def __init__(self,case):
  super().__init__('jetbot_livo_odometry',parameter_overrides=[Parameter('use_sim_time',value=True)])
  self.twist_projection=json.loads((case/'CASE.json').read_text()).get('navigation',{}).get('odom_twist_projection','native')
  assert self.twist_projection in('native','differential_planar')
  self.root=case;self.latest=None;self.correction=None;self.gravity=[];self.alignment=None;self.origin=None;self.previous=None;self.fault=None;self.count=0;self.latest_gyro=None
  c=json.loads((case/'camera_contract.json').read_text());self.lever=np.array(c['mounts']['jetbot/imu']['T']);assert np.allclose(c['mounts']['jetbot/imu']['R'],np.eye(3),atol=1e-5)
  self.pub=self.create_publisher(Odometry,'/jetbot/odom',10);self.tf=TransformBroadcaster(self)
  self.create_subscription(Imu,'/jetbot/imu',self.imu,qos_profile_sensor_data);self.create_subscription(Odometry,'/jetbot/livo/imu_propagated',self.estimate,qos_profile_sensor_data);self.create_subscription(Odometry,'/jetbot/livo/filter_updates',self.correct,100)
  self.stream=(case/'estimated_odom.jsonl').open('x')
 def correct(self,m):self.correction=seconds(m.header.stamp)
 def imu(self,m):
  g=m.angular_velocity;self.latest_gyro=(seconds(m.header.stamp),np.array([g.x,g.y,g.z]))
  if self.alignment is not None or self.latest is None:return
  ts=seconds(m.header.stamp);when,rotation=self.latest
  if not 0<=ts-when<=.20:return
  a=m.linear_acceleration;v=np.array([a.x,a.y,a.z])
  if not np.isfinite(v).all()or not 7<np.linalg.norm(v)<13:return
  self.gravity.append((ts,rotation.apply(v)))
  if len(self.gravity)>=250 and ts-self.gravity[0][0]>=1.5:
   up=np.mean([x[1]for x in self.gravity],axis=0);up/=np.linalg.norm(up);cross=np.cross(up,[0.,0.,1.]);cos=float(up[2]);skew=np.array([[0.,-cross[2],cross[1]],[cross[2],0.,-cross[0]],[-cross[1],cross[0],0.]])
   if cos<-.99:self.fault='INVERTED_GRAVITY';return
   self.alignment=Rotation.from_matrix(np.eye(3)+skew+skew@skew/(1+cos));(self.root/'ODOM_ALIGNMENT.json').write_text(json.dumps(dict(source='stationary native IMU and concurrent FAST-LIVO estimate only',samples=len(self.gravity),imu_mount_m=self.lever.tolist(),frozen=True,truth_used=False),indent=2)+'\n')
 def estimate(self,m):
  if self.fault:return
  try:
   if m.header.frame_id!='world' or m.child_frame_id!='':raise ValueError('Unexpected native propagated frames')
   ts=seconds(m.header.stamp);p=m.pose.pose.position;q=m.pose.pose.orientation;rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]);self.latest=(ts,rotation)
   if self.alignment is None or self.correction is None:return
   if not 0<=ts-self.correction<=.35:return # Stop output; do not fabricate freshness.
   if self.previous is not None and ts-self.previous[0]<.019:return
   if len(self.get_publishers_info_by_topic('/jetbot/odom'))>1:raise ValueError('Conflicting odometry publisher')
   position=np.array([p.x,p.y,p.z])-rotation.apply(self.lever)
   if self.origin is None:self.origin=position.copy()
   position=self.alignment.apply(position-self.origin);orientation=self.alignment*rotation
   if self.previous is None:self.previous=(ts,position,orientation);return
   dt=ts-self.previous[0]
   if not 0<dt<=.30:raise ValueError('Estimator timestamp discontinuity')
   pose_step=float(np.linalg.norm(position-self.previous[1]));rotation_step=float((self.previous[2].inv()*orientation).magnitude())
   if self.latest_gyro is None or abs(self.latest_gyro[0]-ts)>.03:return
   angular=self.latest_gyro[1].copy()
   velocity=m.twist.twist.linear;native_velocity=np.array([velocity.x,velocity.y,velocity.z]);linear=rotation.inv().apply(native_velocity)-np.cross(angular,self.lever)
   raw_body_linear=linear.copy()
   if self.twist_projection=='differential_planar':
    # Nonholonomic navigation twist only. Keep the estimated pose, forward
    # velocity and native gyro unchanged; never use truth or wheel odometry.
    linear[1:]=0.
   # Native filter corrections can change pose within one selected interval.
   # Do not mislabel a correction/dt as measured physical velocity. Keep native
   # propagated velocity and the actual native gyro, including the IMU lever arm.
   if not np.isfinite(np.r_[position,raw_body_linear,linear,angular]).all() or np.linalg.norm(raw_body_linear)>3 or np.linalg.norm(angular)>6 or pose_step>1. or rotation_step>1.:raise ValueError('Estimator discontinuity')
   output=Odometry();output.header.stamp=m.header.stamp;output.header.frame_id='odom';output.child_frame_id='chassis';output.pose.pose.position.x,output.pose.pose.position.y,output.pose.pose.position.z=map(float,position);output.pose.pose.orientation.x,output.pose.pose.orientation.y,output.pose.pose.orientation.z,output.pose.pose.orientation.w=map(float,orientation.as_quat());output.twist.twist.linear.x,output.twist.twist.linear.y,output.twist.twist.linear.z=map(float,linear);output.twist.twist.angular.x,output.twist.twist.angular.y,output.twist.twist.angular.z=map(float,angular)
   cov=np.array(m.pose.covariance).reshape(6,6);jac=np.zeros((6,6));jac[:3,:3]=jac[3:,3:]=self.alignment.as_matrix();output.pose.covariance=(jac@cov@jac.T).ravel().tolist()
   self.pub.publish(output);t=TransformStamped();t.header=output.header;t.child_frame_id='chassis';t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,position);t.transform.rotation=output.pose.pose.orientation;self.tf.sendTransform(t)
   self.previous=(ts,position,orientation);self.count+=1;self.stream.write(json.dumps(dict(stamp=ts,xyz=position.tolist(),xyzw=orientation.as_quat().tolist(),correction_age_s=ts-self.correction,pose_step_m=pose_step,rotation_step_rad=rotation_step,native_velocity_world=native_velocity.tolist(),raw_body_linear=raw_body_linear.tolist(),navigation_linear=linear.tolist(),twist_projection=self.twist_projection,angular_native_imu=angular.tolist()))+'\n');self.stream.flush()
  except Exception as e:
   self.fault=repr(e);(self.root/'ODOM_FAULT.json').write_text(json.dumps(dict(error=self.fault,output_stopped=True,last_stamp=locals().get('ts'),pose_step_m=locals().get('pose_step'),rotation_step_rad=locals().get('rotation_step')))+'\n')
p=argparse.ArgumentParser();p.add_argument('--case',type=Path,required=True);a=p.parse_args();rclpy.init();node=EstimatedOdometry(a.case)
try:rclpy.spin(node)
except KeyboardInterrupt:pass
finally:
 node.stream.close();(a.case/'ODOM_RESULT.json').write_text(json.dumps(dict(samples=node.count,fault=node.fault,gravity_ready=node.alignment is not None,truth_input=False),indent=2)+'\n');node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
