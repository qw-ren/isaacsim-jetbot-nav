"""Explicit coarse capture-time PointXYZIRT trial, isolated from production navigation."""
import json,math
from array import array
from pathlib import Path
import numpy as np
import omni.graph.core as og
from sensor_msgs.msg import PointCloud2,PointField
from plane_audit import PlaneAuditAdapter

DTYPE=np.dtype({'names':['x','y','z','intensity','time','ring'],'formats':['<f4','<f4','<f4','<f4','<f4','<u2'],'offsets':[0,4,8,12,16,20],'itemsize':24})
class Frame20msTrialAdapter(PlaneAuditAdapter):
    debug_windows=[1,3,5,7]
    def attach(self):
        gate_path=Path(str(getattr(self.node.cfg,'lio_frame_trial_gate','')))
        gate=json.loads(gate_path.read_text())
        if not gate.get('frame20ms_trial_allowed') or gate.get('production_odom_switch_allowed') or gate.get('topic')!='/lio/points_frame20ms':raise RuntimeError('Missing explicit isolated 20ms trial qualification')
        super().attach()
        self.trial_root=self.root/'frame20ms_trial';self.trial_root.mkdir(exist_ok=False)
        (self.trial_root/'contract.json').write_text(json.dumps(gate,indent=2)+'\n')
        self.cloud_pub=self.node.create_publisher(PointCloud2,'/lio/points_frame20ms',5)
        self.packet_group=[];self.last_frame=-1;self.last_capture=None;self.last_native=None;self.clouds=0;self.discontinuities=0;self.saved=set()
        self.cloud_log=(self.trial_root/'scans.jsonl').open('x')
    def audit_lidar(self):
        super().audit_lidar()
        data=self.raw_lidars[0].get_data()
        if not data or int(data.get('timestampNs',0))<=0:return
        frame=int(data['frameId'])
        if frame==self.last_frame:return
        self.last_frame=frame
        dispatcher=og.Controller().node('/Render/PostProcess/SDGPipeline/PostProcessDispatcher')
        reference=int(dispatcher.get_attribute('outputs:referenceTimeNumerator').get());capture=float(self.core.get_sim_time_at_swh_frame(reference))
        native=int(data['timestampNs'])/1e9
        if abs(capture-float(self.env.sim.current_time))>1e-6:raise RuntimeError('Unmatched render/physics capture time')
        if self.last_capture is not None:
            dt=capture-self.last_capture;ndt=native-self.last_native
            if dt<=0:raise RuntimeError('Nonmonotonic packet capture')
            if abs(dt-.02)>1e-6 or abs(ndt-dt)>1e-6:self.packet_group=[];self.discontinuities+=1
        self.last_capture=capture;self.last_native=native
        distance=np.asarray(data['distances']);az=np.deg2rad(data['azimuths']);el=np.deg2rad(data['elevations']);ring=np.asarray(data['channels']);intensity=np.asarray(data['intensities'])
        valid=np.isfinite(distance)&np.isfinite(az)&np.isfinite(el)&np.isfinite(intensity)&(distance>.3)&(distance<30)&(ring<32)
        d=distance[valid];az=az[valid];el=el[valid];cloud=np.zeros(len(d),dtype=DTYPE)
        cloud['x']=d*np.cos(el)*np.cos(az);cloud['y']=d*np.cos(el)*np.sin(az);cloud['z']=d*np.sin(el);cloud['intensity']=intensity[valid];cloud['ring']=ring[valid]
        self.packet_group.append((capture,frame,cloud))
        if len(self.packet_group)<5:return
        group=self.packet_group;self.packet_group=[];begin=group[0][0];end=group[-1][0]
        if abs(end-begin-.08)>1e-6:raise RuntimeError('Bad five-packet scan duration')
        for time,_,part in group:part['time']=time-begin
        points=np.zeros(sum(len(x[2]) for x in group),dtype=DTYPE);cursor=0
        for _,_,part in group:points[cursor:cursor+len(part)]=part;cursor+=len(part)
        if points.dtype.itemsize!=24 or points.nbytes!=len(points)*24:raise RuntimeError('Point stride changed during assembly')
        self.clouds+=1
        msg=PointCloud2();ns=round(begin*1e9);msg.header.stamp.sec=ns//1000000000;msg.header.stamp.nanosec=ns%1000000000;msg.header.frame_id='unitree_go2/lidar_frame';msg.height=1;msg.width=len(points);msg.is_bigendian=False;msg.point_step=24;msg.row_step=24*len(points);msg.is_dense=True
        msg.fields=[PointField(name=n,offset=o,datatype=PointField.UINT16 if n=='ring' else PointField.FLOAT32,count=1) for n,o in [('x',0),('y',4),('z',8),('intensity',12),('time',16),('ring',20)]];msg.data=array('B',points.tobytes());self.cloud_pub.publish(msg)
        hist=np.histogram(np.arctan2(points['y'],points['x']),bins=np.linspace(-math.pi,math.pi,73))[0]
        row={'scan':self.clouds,'stamp':begin,'end':end,'packet_frames':[x[1] for x in group],'points':len(points),'ring_range':[int(points['ring'].min()),int(points['ring'].max())],'unique_time_offsets_s':np.unique(points['time']).astype(float).tolist(),'azimuth_bins_nonempty':int((hist>0).sum()),'nominal_packet_interval_s':.02,'strict_uncertainty_bound_validated':False,'discontinuities':self.discontinuities}
        self.cloud_log.write(json.dumps(row)+'\n');self.cloud_log.flush()
        window=int(begin)
        if window in self.debug_windows and window not in self.saved:np.savez_compressed(self.trial_root/f'points_{window}.npz',points=points,stamp=begin);self.saved.add(window)
    def close(self):
        if hasattr(self,'cloud_log'):
            self.cloud_log.close();(self.trial_root/'summary.json').write_text(json.dumps({'scans':self.clouds,'packet_discontinuities':self.discontinuities,'production_tf_output':False,'native_deltaTimes_used':False},indent=2)+'\n')
        super().close()
