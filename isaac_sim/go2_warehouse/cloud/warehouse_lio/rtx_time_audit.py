"""Bounded native RTX/clock/pose audit. No velocity, point-cloud or odometry output."""
import json
from pathlib import Path
import numpy as np
import omni.graph.core as og
import omni.timeline
from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
from native_imu_adapter import NativeImuAdapter

class TimeAuditAdapter(NativeImuAdapter):
    maximum_frames=1800
    archive_every=25
    def attach(self):
        super().attach()
        self.audit_root=self.root/'time_audit'
        self.audit_root.mkdir(exist_ok=False)
        self.frames=(self.audit_root/'frames.jsonl').open('x')
        self.core=_isaacsim_core_nodes.acquire_interface()
        self.timeline=omni.timeline.get_timeline_interface()
        self.seen=set()
        (self.audit_root/'contract.json').write_text(json.dumps({
            'purpose':'validate time and pose only; no LIO input approved',
            'reference_time_method':'Isaac Sim 4.5 lidar_rtx.py _data_acquisition_callback',
            'ground_truth':'evaluation only; never published to LIO',
            'maximum_unique_frames':self.maximum_frames,'point_archives_every_n_frames':self.archive_every,
            'native_timestamp_semantics':'start of frame; native transform end and transformStart start',
            'source':'installed isaacsim.sensors.rtx OgnIsaacReadRTXLidarData.rst'},indent=2)+'\n')
    def audit_lidar(self):
        if len(self.seen)>=self.maximum_frames:return
        for index,annotator in enumerate(self.raw_lidars):
            data=annotator.get_data()
            if not data or 'frameId' not in data:continue
            fid=int(data['frameId']);key=(index,fid)
            if key in self.seen:continue
            self.seen.add(key)
            state=self.env.scene['unitree_go2'].data.root_state_w[0].detach().cpu().numpy().copy()
            row={'index':index,'frame_id':fid,'sim_time':float(self.env.sim.current_time),
                 'imu_stamp':self.last,'timeline_time':float(self.timeline.get_current_time()),
                 'core_sim_time':float(self.core.get_sim_time()),
                 'core_monotonic_time':float(self.core.get_sim_time_monotonic()),
                 'timestamp_ns':int(data['timestampNs']),
                 'transform':np.asarray(data['transform']).tolist(),
                 'transform_start':np.asarray(data['transformStart']).tolist(),
                 'root_state_w_xyz_wxyz_vel':state.tolist(),
                 'points':int(np.asarray(data['distances']).size)}
            try:
                dispatcher=og.Controller().node('/Render/PostProcess/SDGPipeline/PostProcessDispatcher')
                reference=int(dispatcher.get_attribute('outputs:referenceTimeNumerator').get())
                row['dispatcher_reference']=reference
                row['render_sim_time']=float(self.core.get_sim_time_at_swh_frame(reference))
                row['render_monotonic_time']=float(self.core.get_sim_time_monotonic_at_swh_frame(reference))
                row['frame_id_sim_time']=float(self.core.get_sim_time_at_swh_frame(fid))
            except Exception as exc:row['history_query_error']=str(exc)
            self.frames.write(json.dumps(row)+'\n');self.frames.flush()
            if len(self.seen)%self.archive_every==0:
                self.archive_frame(data,index,fid)
    def archive_frame(self,data,index,fid):
        arrays={}
        for k,v in data.items():
            try:
                a=np.asarray(v)
                if a.dtype.kind in 'biuf' and a.size:arrays[k]=a.copy()
            except (ValueError,TypeError):pass
        scan=self.node.lidar_annotators[index].get_data() if hasattr(self.node,'lidar_annotators') else None
        if isinstance(scan,dict) and 'data' in scan:arrays['scan_buffer_world']=np.asarray(scan['data']).copy()
        np.savez_compressed(self.audit_root/f'frame_{index}_{fid}.npz',**arrays)
    def close(self):
        if hasattr(self,'frames'):self.frames.close()
        super().close()
