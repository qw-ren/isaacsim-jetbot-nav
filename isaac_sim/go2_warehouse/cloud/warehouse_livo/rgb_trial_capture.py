"""Publish native rendered RGB with its acquisition time; no truth pose input."""
import csv
import json
from pathlib import Path

import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_matrix(wxyz):
    w, x, y, z = np.asarray(wxyz, dtype=float) / np.linalg.norm(wxyz)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class RgbTrialCapture:
    def __init__(self, camera, node, env, output):
        self.camera, self.node, self.env = camera, node, env.unwrapped
        self.root = Path(output)/'rgb'
        self.root.mkdir(exist_ok=False)
        self.publisher = node.create_publisher(Image, '/livo/image_raw', 30)
        self.info_publisher = node.create_publisher(CameraInfo, '/livo/camera_info', 10)
        self.transform = StaticTransformBroadcaster(node)
        self.old_frame = -1
        self.old_stamp = -1.
        self.last_selected = -1.
        self.count = 0
        self.ready = False
        self.stream = (self.root/'frames.csv').open('x', newline='')
        self.csv = csv.writer(self.stream)
        self.csv.writerow(['render_frame','render_time_s','simulation_time_s','published','width','height'])

    def initialize_contract(self):
        self.k = np.asarray(self.camera.get_intrinsics_matrix(), dtype=float)
        self.width,self.height=map(int,self.camera.get_resolution())
        position, orientation = self.camera.get_local_pose(camera_axes='ros')
        position, orientation = np.asarray(position), np.asarray(orientation)
        rotation = quaternion_matrix(orientation)
        # Independent axis sanity check: optical +z forward, +x right, +y down.
        expected = np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        if not np.allclose(rotation, expected, atol=1e-5):
            raise RuntimeError('Native camera optical axes do not match declared front mount')
        if not np.allclose(position, [.4,0.,.2], atol=1e-5):
            raise RuntimeError('Unexpected camera installation translation')
        if self.k[0,0]<=0 or self.k[1,1]<=0:
            raise RuntimeError('Invalid native camera intrinsic matrix')
        # The IMU is mounted at the base origin; LiDAR has identity rotation.
        rcl = rotation.T
        pcl = rcl @ (np.array([.2,0.,.2])-position)
        self.contract = dict(camera_prim=str(self.camera.prim_path),
            frame_id='livo/camera_optical', parent='unitree_go2/base_link',
            width=self.width,height=self.height,k=self.k.tolist(), body_from_optical_R=rotation.tolist(),
            body_from_optical_T=position.tolist(), body_from_optical_wxyz=orientation.tolist(),
            Rcl=rcl.tolist(), Pcl=pcl.tolist(), distortion='native pinhole, zero coefficients',
            timestamp='Camera.get_current_frame rendering_time; not node wall clock',
            sampling='unique rendered frames selected at >=0.099s acquisition interval',
            truth_pose_used=False, rolling_shutter_simulated=False)
        (self.root/'camera_contract.json').write_text(json.dumps(self.contract,indent=2)+'\n')
        t = TransformStamped()
        t.header.frame_id='unitree_go2/base_link';t.child_frame_id='livo/camera_optical'
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,position)
        t.transform.rotation.w,t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z=map(float,orientation)
        self.transform.sendTransform(t)
        self.ready=True

    def tick(self):
        frame=self.camera.get_current_frame()
        rgba=np.asarray(frame.get('rgba',[]))
        if rgba.ndim!=3 or rgba.shape[2]<3:return
        if not self.ready:self.initialize_contract()
        fid=int(frame['rendering_frame']);stamp=float(frame['rendering_time'])
        if fid<=0 or stamp<=0 or rgba.shape[:2]!=(self.height,self.width):return
        if fid==self.old_frame:return
        if stamp<=self.old_stamp:raise RuntimeError('Camera acquisition time did not increase')
        self.old_frame,self.old_stamp=fid,stamp
        now=float(self.env.sim.current_time)
        if stamp>now+.021 or now-stamp>.15:raise RuntimeError('Camera timestamp outside simulation sampling window')
        selected=stamp-self.last_selected>=.099
        height,width=rgba.shape[:2]
        self.csv.writerow([fid,stamp,now,selected,width,height])
        if not selected:return
        self.last_selected=stamp
        rgb=np.ascontiguousarray(rgba[:,:,:3],dtype=np.uint8)
        m=Image();ns=round(stamp*1e9)
        m.header.stamp.sec=ns//1000000000;m.header.stamp.nanosec=ns%1000000000
        m.header.frame_id='livo/camera_optical';m.height=height;m.width=width
        m.encoding='rgb8';m.is_bigendian=0;m.step=width*3;m.data=rgb.tobytes()
        self.publisher.publish(m)
        info=CameraInfo();info.header=m.header;info.height=height;info.width=width
        info.distortion_model='plumb_bob';info.d=[0.]*5;info.k=self.k.reshape(-1).tolist()
        info.r=np.eye(3).reshape(-1).tolist();info.p=np.c_[self.k,np.zeros(3)].reshape(-1).tolist()
        self.info_publisher.publish(info)
        self.count+=1
        if self.count in [1,30,100]:
            from PIL import Image as PillowImage
            PillowImage.fromarray(rgb).save(self.root/f'frame_{self.count:04d}.png')
        if self.count%10==0:
            self.stream.flush()
            (self.root/'PROGRESS.json').write_text(json.dumps(dict(images=self.count,last_stamp_s=stamp))+'\n')

    def close(self):
        self.stream.close()
        (self.root/'SUMMARY.json').write_text(json.dumps(dict(images=self.count,last_stamp_s=self.old_stamp,contract_ready=self.ready))+'\n')
