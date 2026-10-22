"""Native physical-step IMU and bounded RTX raw audit; no LIO/odom/velocity output."""
import csv,json,math
from pathlib import Path
import numpy as np
from isaacsim.sensors.physics import IMUSensor,_sensor
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


_spawned_imus=[]
def spawn_go2_with_native_imu(prim_path,cfg,**kwargs):
    # Register the native sensor before the first physics-scene initialization.
    from isaaclab.sim import spawn_from_usd
    robot=spawn_from_usd(prim_path,cfg,**kwargs)
    sensor=IMUSensor(str(robot.GetPath())+'/base/lio_imu',frequency=200,translation=np.zeros(3),orientation=np.array([1.,0.,0.,0.]),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1)
    _spawned_imus.append(sensor)
    return robot

class NativeImuAdapter:
    def __init__(self,env,node,raw_lidars,artifacts):
        self.env=env.unwrapped;self.node=node;self.raw_lidars=raw_lidars;self.root=Path(artifacts)/'lio_sensors';self.root.mkdir(parents=True,exist_ok=False)
        self.path='/World/envs/env_0/Go2/base/lio_imu';self.sensor=IMUSensor(self.path,frequency=200,translation=np.zeros(3),orientation=np.array([1.,0.,0.,0.]),linear_acceleration_filter_size=1,angular_velocity_filter_size=1,orientation_filter_size=1)
        self.interface=_sensor.acquire_imu_sensor_interface();self.publisher=node.create_publisher(Imu,'/lio/imu',200)
        self.tf=StaticTransformBroadcaster(node);t=TransformStamped();t.header.frame_id='unitree_go2/base_link';t.child_frame_id='unitree_go2/imu_link';t.transform.rotation.w=1.;self.tf.sendTransform(t)
        self.stream=(self.root/'imu_physical_steps.csv').open('x',newline='');self.csv=csv.writer(self.stream);self.csv.writerow(['stamp','sim_time','ax','ay','az','gx','gy','gz','qx','qy','qz','qw','ax_no_gravity','ay_no_gravity','az_no_gravity'])
        self.last=-1.;self.count=0;self.duplicates=0;self.invalid=0;self.raw_saved=set();self.original_step=None
    def attach(self):
        self.sensor.initialize();self.original_step=self.env.sim.step
        def step(*args,**kwargs):
            value=self.original_step(*args,**kwargs);self.sample();return value
        self.env.sim.step=step
        (self.root/'contract.json').write_text(json.dumps({'imu_prim':self.path,'frame_id':'unitree_go2/imu_link','parent_frame':'unitree_go2/base_link','mount_xyz':[0,0,0],'mount_wxyz':[1,0,0,0],'requested_frequency_hz':200,'physics_dt':float(self.env.physics_dt),'sampling':'after each actual SimulationContext.step; publish only strictly new native sensor time','use_latest_data':True,'read_gravity':True,'orientation_published':False,'covariance':'unknown zero covariance; ideal simulator measurement, not calibrated noise','raw_rtx':'diagnostic only; no constructed time/ring published','lio_odom_output':False},indent=2)+'\n')
    def sample(self):
        r=self.interface.get_sensor_reading(self.path,use_latest_data=True,read_gravity=True)
        if not r.is_valid:self.invalid+=1;return
        stamp=float(r.time)
        if stamp==self.last:self.duplicates+=1;return
        if stamp<self.last:raise RuntimeError('Native IMU time went backwards')
        values=[r.lin_acc_x,r.lin_acc_y,r.lin_acc_z,r.ang_vel_x,r.ang_vel_y,r.ang_vel_z]
        if not all(math.isfinite(v) for v in [stamp,*values]):raise RuntimeError('Nonfinite native IMU data')
        self.last=stamp;self.count+=1
        m=Imu();ns=round(stamp*1e9);m.header.stamp.sec=ns//1000000000;m.header.stamp.nanosec=ns%1000000000;m.header.frame_id='unitree_go2/imu_link';m.orientation.w=1.;m.orientation_covariance[0]=-1.
        m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,values[:3]);m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z=map(float,values[3:]);self.publisher.publish(m)
        g=self.interface.get_sensor_reading(self.path,use_latest_data=True,read_gravity=False)
        self.csv.writerow([stamp,float(self.env.sim.current_time),*values,*list(r.orientation),g.lin_acc_x,g.lin_acc_y,g.lin_acc_z])
        if self.count%200==0:self.stream.flush()
    def audit_lidar(self):
        sim=float(self.env.sim.current_time)
        # A few raw render frames in stationary/forward/braking windows only.
        window=int(sim)
        if window not in [1,3,4,5,6,7,8,9] or window in self.raw_saved:return
        for i,a in enumerate(self.raw_lidars):
            raw=a.get_data();arrays={};meta={'sim_time':sim,'keys':list(raw),'mode':'raw RTX read node; temporal semantics unvalidated'}
            for key,value in raw.items():
                try:
                    arr=np.asarray(value)
                    if arr.dtype.kind in 'biuf' and arr.size:
                        arrays[key]=arr.copy();meta[key]={'shape':list(arr.shape),'dtype':str(arr.dtype),'min':float(np.min(arr)),'max':float(np.max(arr))}
                except (TypeError,ValueError):pass
            np.savez_compressed(self.root/f'raw_{window}_{i}.npz',**arrays);(self.root/f'raw_{window}_{i}.json').write_text(json.dumps(meta,indent=2)+'\n')
        self.raw_saved.add(window)
    def close(self):
        if self.original_step:self.env.sim.step=self.original_step
        self.stream.close();(self.root/'capture_summary.json').write_text(json.dumps({'samples':self.count,'duplicate_native_time_not_republished':self.duplicates,'invalid_native_readings':self.invalid,'last_stamp':self.last},indent=2)+'\n')
