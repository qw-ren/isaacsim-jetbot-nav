"""Read-only ROS IMU/clock/odom evidence capture; no command or TF publisher."""
import argparse,json,time,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=float,default=14);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
 rclpy.init();n=rclpy.create_node('go2_lio_sensor_observer',parameter_overrides=[Parameter('use_sim_time',value=True)]);rows=[];odom=[];clouds=[];clock=[None];edges=[]
 def imu(m):rows.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'clock':clock[0],'frame':m.header.frame_id,'acc':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],'gyro':[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],'orientation_covariance_0':m.orientation_covariance[0]})
 def od(m):odom.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'yaw':math.atan2(2*(m.pose.pose.orientation.w*m.pose.pose.orientation.z+m.pose.pose.orientation.x*m.pose.pose.orientation.y),1-2*(m.pose.pose.orientation.y**2+m.pose.pose.orientation.z**2)),'vx':m.twist.twist.linear.x,'wz':m.twist.twist.angular.z})
 def static(m):edges.extend([[t.header.frame_id,t.child_frame_id,[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]] for t in m.transforms])
 subs=[n.create_subscription(Imu,'/lio/imu',imu,qos_profile_sensor_data),n.create_subscription(Odometry,'/unitree_go2/odom',od,qos_profile_sensor_data),n.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data),n.create_subscription(PointCloud2,'/unitree_go2/lidar/point_cloud',lambda m:clouds.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'points':m.width*m.height,'fields':[f.name for f in m.fields]}),qos_profile_sensor_data),n.create_subscription(TFMessage,'/tf_static',static,QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))]
 result={'commands_sent':False,'lio_started':False}
 try:
  deadline=time.monotonic()+100
  while not (rows and odom and clock[0] is not None) and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
  if not rows:raise RuntimeError('No native IMU messages')
  start=rows[-1]['stamp'];deadline=time.monotonic()+120
  while rows[-1]['stamp']-start<a.seconds and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
 except Exception as e:result['error']=str(e)
 finally:
  (a.output/'telemetry.json').write_text(json.dumps({'imu':rows,'odom':odom,'clouds':clouds,'static_tf':edges},indent=2)+'\n')
  if len(rows)>1:
   dt=np.diff([x['stamp'] for x in rows]);result.update(samples=len(rows),duration_s=rows[-1]['stamp']-rows[0]['stamp'],measured_sim_hz=(len(rows)-1)/(rows[-1]['stamp']-rows[0]['stamp']),median_dt_s=float(np.median(dt)),max_dt_s=float(dt.max()),min_dt_s=float(dt.min()),max_imu_minus_clock_s=max(x['stamp']-x['clock'] for x in rows if x['clock'] is not None))
  result['imu_publishers']=[x.node_name for x in n.get_publishers_info_by_topic('/lio/imu')];result['checks']={'no_error':'error' not in result,'measured_200hz':190<=result.get('measured_sim_hz',0)<=210,'strictly_new_measurement_time':len(rows)>1 and result['min_dt_s']>0,'physical_step_spacing':len(rows)>1 and .0049<=result['min_dt_s']<=result['max_dt_s']<=.0051,'capture_duration':result.get('duration_s',0)>=a.seconds,'frame':bool(rows) and all(x['frame']=='unitree_go2/imu_link' for x in rows),'finite':bool(rows) and bool(np.isfinite(np.asarray([x['acc']+x['gyro'] for x in rows])).all()),'no_truth_orientation_fusion':bool(rows) and all(x['orientation_covariance_0']==-1 for x in rows),'one_imu_source':len(result['imu_publishers'])==1,'base_imu_static_identity':any(x[0:2]==['unitree_go2/base_link','unitree_go2/imu_link'] and x[2]==[0.,0.,0.] and x[3]==[0.,0.,0.,1.] for x in edges),'existing_xyz_cloud_preserved':bool(clouds) and all(x['fields']==['x','y','z'] for x in clouds)}
  result['passed']=all(result['checks'].values());(a.output/'imu_stream_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
 return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
